use crate::core::math::ikd_tree::{IkdTreePoint, BoxPointType, PointVector};
use crate::core::state::SlamContext;

/// Minimum number of points required for ESIKF update
const MIN_POINTS_FOR_UPDATE: usize = 100;

/// Measurement noise covariance for point-to-plane residuals
const LASER_POINT_COV: f64 = 0.001;

/// Maximum iterations for ESIKF
const NUM_MAX_ITERATIONS: usize = 4;

/// Voxel filter size for downsampling
const VOXEL_FILTER_SIZE: f64 = 0.5;

/// Initialization time (seconds) before enabling EKF updates
const INIT_TIME: f64 = 0.1;

impl SlamContext {
    /// Main processing function implementing FAST-LIO2 pipeline
    /// 
    /// Pipeline:
    /// 1. Get synchronized LiDAR + IMU data
    /// 2. IMU forward propagation (done in process_next via undistort_pcl)
    /// 3. Downsample point cloud
    /// 4. If map is empty, initialize map
    /// 5. Otherwise, run ESIKF update (point-to-plane ICP)
    /// 6. Add corrected points to map
    pub async fn process(&mut self) {
        match self.process_next().await {
            Ok(group) => {
                // convert to PointVector, undistort already done in `process_next`
                let feats_undistort: PointVector = group.points.iter().flat_map(|p| {
                    p.to_point_vector()
                }).collect();
                
                if feats_undistort.is_empty() {
                    tracing::warn!("No points in scan, skipping");
                    return;
                }
                
                // tracing::info!("Processing scan with {} undistorted points", feats_undistort.len());
                
                // Downsample point cloud
                let feats_down_body = voxel_downsample(&feats_undistort, VOXEL_FILTER_SIZE);
                let feats_down_size = feats_down_body.len();
                
                // tracing::debug!("After downsampling: {} points", feats_down_size);
                
                if feats_down_size < MIN_POINTS_FOR_UPDATE {
                    tracing::warn!("Too few points after downsampling: {}", feats_down_size);
                    return;
                }
                
                // check if map is empty
                if self.ikd_tree.is_empty() {
                    // Initialize map with first scan
                    self.initialize_map(&feats_down_body);
                    return;
                }
                
                // Check if we're past initialization time
                let time_since_start = group.lidar_begin_time - self.first_lidar_time;
                let ekf_inited = time_since_start > INIT_TIME;
                
                if !ekf_inited {
                    tracing::info!("Still in initialization period ({:.3}s)", time_since_start);
                    // Just add points to map without ESIKF update
                    let feats_world = self.transform_to_world(&feats_down_body);
                    self.ikd_tree.add_points(&feats_world, true);
                    return;
                }
                
                // ========== ESIKF Update ==========
                let update_success = self.kf.update_iterated(
                    LASER_POINT_COV,
                    &feats_down_body,
                    &self.ikd_tree,
                    NUM_MAX_ITERATIONS,
                    false,  // Don't estimate extrinsic for now
                );
                
                if !update_success {
                    tracing::warn!("ESIKF update failed");
                    return;
                }
                
                // ========== Update Local Map ==========
                // Update FOV and remove out-of-range points
                self.update_local_map_fov();
                
                // Transform points to world frame using corrected pose
                let feats_world = self.transform_to_world(&feats_down_body);
                
                // Add to map with downsampling
                let added = self.ikd_tree.add_points(&feats_world, true);
                
                tracing::debug!(
                    "Added {} points to map. Total map size: {}",
                    added, self.ikd_tree.validnum()
                );
            }
            Err(e) => {
                tracing::warn!("Processing error: {}", e);
            }
        }
    }
    
    /// Initialize the map with the first scan
    fn initialize_map(&mut self, feats_down_body: &PointVector) {
        tracing::info!("Initializing map with {} points", feats_down_body.len());
        
        // Transform to world frame
        let feats_world = self.transform_to_world(feats_down_body);
        
        // Build initial tree
        self.ikd_tree.build(feats_world);
        
        // Initialize local map bounds around current position
        let pos = self.kf.get_x().pos;
        self.local_map_bounds = BoxPointType::from_center_halfsize(
            &IkdTreePoint::new(pos[0], pos[1], pos[2]),
            self.cube_len / 2.0,
        );
        self.local_map_initialized = true;
        
        tracing::info!("Map initialized with {} points", self.ikd_tree.validnum());
    }
    
    /// Transform points from body (LiDAR) frame to world frame
    fn transform_to_world(&self, feats_body: &PointVector) -> PointVector {
        let state = self.kf.get_x();
        let rot = state.rot.matrix();
        let offset_r = state.offset_r_l_i.matrix();
        let offset_t = &state.offset_t_l_i;
        let pos = &state.pos;
        
        feats_body.iter().map(|p| {
            let p_body = sophus::nalgebra::Vector3::new(p.x, p.y, p.z);
            // p_world = R * (R_L_I * p_body + T_L_I) + pos
            let p_imu = offset_r * p_body + offset_t;
            let p_world = rot * p_imu + pos;
            IkdTreePoint::new(p_world[0], p_world[1], p_world[2])
        }).collect()
    }
    
    /// Update local map FOV - remove points that are too far from current position
    /// 
    /// This implements the sliding window map management from FAST-LIO2
    fn update_local_map_fov(&mut self) {
        if !self.local_map_initialized {
            return;
        }
        
        let pos = self.kf.get_x().pos;
        let pos_lid = IkdTreePoint::new(pos[0], pos[1], pos[2]);
        
        // Check distance to local map boundaries
        let mov_threshold = 1.5;  // Movement threshold multiplier
        let det_range = self.det_range;
        
        let mut need_move = false;
        for i in 0..3 {
            let coord = match i {
                0 => pos_lid.x,
                1 => pos_lid.y,
                _ => pos_lid.z,
            };
            let dist_min = (coord - self.local_map_bounds.vertex_min[i]).abs();
            let dist_max = (coord - self.local_map_bounds.vertex_max[i]).abs();
            
            if dist_min <= mov_threshold * det_range || dist_max <= mov_threshold * det_range {
                need_move = true;
                break;
            }
        }
        
        if !need_move {
            return;
        }
        
        // Calculate boxes to remove and update local map bounds
        let mov_dist = ((self.cube_len - 2.0 * mov_threshold * det_range) * 0.5 * 0.9)
            .max(det_range * (mov_threshold - 1.0));
        
        let mut boxes_to_remove: Vec<BoxPointType> = Vec::new();
        let mut new_bounds = self.local_map_bounds;
        
        for i in 0..3 {
            let coord = match i {
                0 => pos_lid.x,
                1 => pos_lid.y,
                _ => pos_lid.z,
            };
            let dist_min = (coord - self.local_map_bounds.vertex_min[i]).abs();
            let dist_max = (coord - self.local_map_bounds.vertex_max[i]).abs();
            
            if dist_min <= mov_threshold * det_range {
                // Need to move towards negative direction
                new_bounds.vertex_max[i] -= mov_dist;
                new_bounds.vertex_min[i] -= mov_dist;
                
                let mut remove_box = self.local_map_bounds;
                remove_box.vertex_min[i] = self.local_map_bounds.vertex_max[i] - mov_dist;
                boxes_to_remove.push(remove_box);
            } else if dist_max <= mov_threshold * det_range {
                // Need to move towards positive direction
                new_bounds.vertex_max[i] += mov_dist;
                new_bounds.vertex_min[i] += mov_dist;
                
                let mut remove_box = self.local_map_bounds;
                remove_box.vertex_max[i] = self.local_map_bounds.vertex_min[i] + mov_dist;
                boxes_to_remove.push(remove_box);
            }
        }
        
        self.local_map_bounds = new_bounds;
        
        // Remove points outside the new local map
        if !boxes_to_remove.is_empty() {
            let removed = self.ikd_tree.delete_point_boxes(&boxes_to_remove);
            tracing::debug!("Removed {} points from map (FOV update)", removed);
        }
    }
}

/// Simple voxel downsampling
/// 
/// Keeps only one point per voxel (the first one encountered)
fn voxel_downsample(points: &PointVector, voxel_size: f64) -> PointVector {
    use std::collections::HashMap;
    
    let mut voxel_map: HashMap<(i64, i64, i64), IkdTreePoint> = HashMap::new();
    
    for point in points {
        let voxel_idx = (
            (point.x / voxel_size).floor() as i64,
            (point.y / voxel_size).floor() as i64,
            (point.z / voxel_size).floor() as i64,
        );
        
        // Keep first point in each voxel
        voxel_map.entry(voxel_idx).or_insert(*point);
    }
    
    voxel_map.into_values().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_longtime_run() {
        let mut ctx: SlamContext = crate::frontend::sensor::sensor_init(&crate::config::CONFIG.get().unwrap()).await.unwrap();
        for i in 0..50 {
            ctx.process().await;
            if i == 0 {
                ctx.ikd_tree.export_tree_to_txt("logs/test_longtime_run_tree_start.txt").await.unwrap();
                println!("Initial tree exported.");
            }
            if i == 49 {
                ctx.ikd_tree.export_tree_to_txt("logs/test_longtime_run_tree_end.txt").await.unwrap();
                println!("Final tree exported.");
            }
        }
    }
}