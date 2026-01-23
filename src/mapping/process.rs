use crate::core::math::ikd_tree::{IkdTreePoint, BoxPointType, PointVector};
use crate::core::state::SlamContext;
use serde::{Serialize, Deserialize};
use std::net::SocketAddr;
use tokio::net::UdpSocket;
use tokio::sync::broadcast;

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
                // Convert to IkdTreePoint format (points are now in body frame, undistorted)
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
                
                // Check if this is the first scan (map is empty)
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

// ============================================================================
// State Publishing
// ============================================================================

/// Publishable state snapshot containing pose and map statistics
/// 
/// This struct is serializable and can be sent over network or saved to file
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SlamStateSnapshot {
    /// Timestamp of the snapshot (seconds since first LiDAR scan)
    pub timestamp: f64,
    /// Position in world frame [x, y, z] (meters)
    pub position: [f64; 3],
    /// Rotation quaternion [w, x, y, z] (world frame)
    pub rotation_quat: [f64; 4],
    /// Rotation matrix (row-major, 3x3 flattened)
    pub rotation_matrix: [f64; 9],
    /// Velocity in world frame [vx, vy, vz] (m/s)
    pub velocity: [f64; 3],
    /// Gyroscope bias [bx, by, bz] (rad/s)
    pub gyro_bias: [f64; 3],
    /// Accelerometer bias [bx, by, bz] (m/s²)
    pub accel_bias: [f64; 3],
    /// Gravity vector estimate [gx, gy, gz] (m/s²)
    pub gravity: [f64; 3],
    /// LiDAR to IMU rotation quaternion [w, x, y, z]
    pub lidar_imu_rot_quat: [f64; 4],
    /// LiDAR to IMU translation [x, y, z] (meters)
    pub lidar_imu_trans: [f64; 3],
    /// Number of valid points in the map
    pub map_point_count: i32,
    /// Total tree size (including deleted points)
    pub map_tree_size: i32,
    /// Local map bounds [min_x, min_y, min_z, max_x, max_y, max_z]
    pub local_map_bounds: [f64; 6],
    /// Whether the map has been initialized
    pub map_initialized: bool,
}

/// Map point cloud data for publishing
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MapPointCloud {
    /// Number of points
    pub point_count: usize,
    /// Flattened point coordinates [x0, y0, z0, x1, y1, z1, ...]
    pub points: Vec<f64>,
}

/// Channel-based state publisher for in-process subscribers
pub struct StatePublisher {
    sender: broadcast::Sender<SlamStateSnapshot>,
}

impl StatePublisher {
    /// Create a new state publisher with specified channel capacity
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity);
        Self { sender }
    }

    /// Get a subscriber that receives state updates
    pub fn subscribe(&self) -> broadcast::Receiver<SlamStateSnapshot> {
        self.sender.subscribe()
    }

    /// Publish a state snapshot to all subscribers
    pub fn publish(&self, state: SlamStateSnapshot) -> Result<usize, broadcast::error::SendError<SlamStateSnapshot>> {
        self.sender.send(state)
    }
}

impl SlamContext {
    /// Get a snapshot of the current SLAM state
    /// 
    /// Returns a serializable struct containing pose, velocity, biases,
    /// and map statistics that can be published over network or saved.
    pub fn get_state_snapshot(&self) -> SlamStateSnapshot {
        let state = self.kf.get_x();
        let rot_matrix = state.rot.matrix();
        
        // Convert rotation matrix to quaternion
        // Using the standard matrix-to-quaternion conversion
        let quat = rotation_matrix_to_quaternion(&rot_matrix);
        let offset_quat = rotation_matrix_to_quaternion(&state.offset_r_l_i.matrix());
        
        SlamStateSnapshot {
            timestamp: 0.0, // Will be set by caller if needed
            position: [state.pos[0], state.pos[1], state.pos[2]],
            rotation_quat: quat,
            rotation_matrix: [
                rot_matrix[(0, 0)], rot_matrix[(0, 1)], rot_matrix[(0, 2)],
                rot_matrix[(1, 0)], rot_matrix[(1, 1)], rot_matrix[(1, 2)],
                rot_matrix[(2, 0)], rot_matrix[(2, 1)], rot_matrix[(2, 2)],
            ],
            velocity: [state.vel[0], state.vel[1], state.vel[2]],
            gyro_bias: [state.bg[0], state.bg[1], state.bg[2]],
            accel_bias: [state.ba[0], state.ba[1], state.ba[2]],
            gravity: [state.grav[0], state.grav[1], state.grav[2]],
            lidar_imu_rot_quat: offset_quat,
            lidar_imu_trans: [state.offset_t_l_i[0], state.offset_t_l_i[1], state.offset_t_l_i[2]],
            map_point_count: self.ikd_tree.validnum(),
            map_tree_size: self.ikd_tree.size(),
            local_map_bounds: [
                self.local_map_bounds.vertex_min[0],
                self.local_map_bounds.vertex_min[1],
                self.local_map_bounds.vertex_min[2],
                self.local_map_bounds.vertex_max[0],
                self.local_map_bounds.vertex_max[1],
                self.local_map_bounds.vertex_max[2],
            ],
            map_initialized: self.local_map_initialized,
        }
    }

    /// Publish state snapshot via UDP to specified address
    /// 
    /// Serializes the state as JSON and sends it via UDP.
    /// Useful for real-time visualization or integration with other systems.
    /// 
    /// # Arguments
    /// * `socket` - Bound UDP socket to send from
    /// * `target` - Target address to send to
    /// 
    /// # Example
    /// ```ignore
    /// let socket = UdpSocket::bind("0.0.0.0:0").await?;
    /// let target: SocketAddr = "127.0.0.1:9000".parse()?;
    /// ctx.publish_state_udp(&socket, target).await?;
    /// ```
    pub async fn publish_state_udp(
        &self,
        socket: &UdpSocket,
        target: SocketAddr,
    ) -> anyhow::Result<()> {
        let snapshot = self.get_state_snapshot();
        let json = serde_json::to_vec(&snapshot)?;
        socket.send_to(&json, target).await?;
        Ok(())
    }

    /// Publish state snapshot via UDP as compact binary format
    /// 
    /// Sends only position (3 x f64) and quaternion (4 x f64) = 56 bytes
    /// More efficient for high-frequency publishing.
    pub async fn publish_pose_udp_binary(
        &self,
        socket: &UdpSocket,
        target: SocketAddr,
    ) -> anyhow::Result<()> {
        let state = self.kf.get_x();
        let quat = rotation_matrix_to_quaternion(&state.rot.matrix());
        
        // Pack as: [x, y, z, qw, qx, qy, qz] (7 x f64 = 56 bytes)
        let mut buffer = [0u8; 56];
        buffer[0..8].copy_from_slice(&state.pos[0].to_le_bytes());
        buffer[8..16].copy_from_slice(&state.pos[1].to_le_bytes());
        buffer[16..24].copy_from_slice(&state.pos[2].to_le_bytes());
        buffer[24..32].copy_from_slice(&quat[0].to_le_bytes());
        buffer[32..40].copy_from_slice(&quat[1].to_le_bytes());
        buffer[40..48].copy_from_slice(&quat[2].to_le_bytes());
        buffer[48..56].copy_from_slice(&quat[3].to_le_bytes());
        
        socket.send_to(&buffer, target).await?;
        Ok(())
    }

    /// Get map points as a publishable point cloud
    /// 
    /// # Arguments
    /// * `max_points` - Maximum number of points to return (None for all)
    /// 
    /// # Returns
    /// MapPointCloud struct containing flattened point coordinates
    pub fn get_map_point_cloud(&self, max_points: Option<usize>) -> MapPointCloud {
        let all_points = self.ikd_tree.flatten_all();
        let points_to_use = match max_points {
            Some(max) if all_points.len() > max => &all_points[..max],
            _ => &all_points,
        };
        
        let flattened: Vec<f64> = points_to_use
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();
        
        MapPointCloud {
            point_count: points_to_use.len(),
            points: flattened,
        }
    }

    /// Publish map point cloud via UDP (JSON format)
    /// 
    /// Note: For large maps, consider using `publish_map_udp_binary` or
    /// subsampling the map with `max_points`.
    pub async fn publish_map_udp(
        &self,
        socket: &UdpSocket,
        target: SocketAddr,
        max_points: Option<usize>,
    ) -> anyhow::Result<()> {
        let map = self.get_map_point_cloud(max_points);
        let json = serde_json::to_vec(&map)?;
        socket.send_to(&json, target).await?;
        Ok(())
    }

    /// Export current state to JSON string
    pub fn state_to_json(&self) -> anyhow::Result<String> {
        let snapshot = self.get_state_snapshot();
        Ok(serde_json::to_string_pretty(&snapshot)?)
    }

    /// Export current state to JSON file
    pub async fn export_state_to_file(&self, path: &str) -> anyhow::Result<()> {
        let json = self.state_to_json()?;
        tokio::fs::write(path, json).await?;
        Ok(())
    }
}

/// Convert rotation matrix to quaternion [w, x, y, z]
fn rotation_matrix_to_quaternion(m: &sophus::nalgebra::Matrix3<f64>) -> [f64; 4] {
    // Algorithm from "Converting a Rotation Matrix to a Quaternion" by Mike Day
    let trace = m[(0, 0)] + m[(1, 1)] + m[(2, 2)];
    
    if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).sqrt();
        [
            0.25 / s,
            (m[(2, 1)] - m[(1, 2)]) * s,
            (m[(0, 2)] - m[(2, 0)]) * s,
            (m[(1, 0)] - m[(0, 1)]) * s,
        ]
    } else if m[(0, 0)] > m[(1, 1)] && m[(0, 0)] > m[(2, 2)] {
        let s = 2.0 * (1.0 + m[(0, 0)] - m[(1, 1)] - m[(2, 2)]).sqrt();
        [
            (m[(2, 1)] - m[(1, 2)]) / s,
            0.25 * s,
            (m[(0, 1)] + m[(1, 0)]) / s,
            (m[(0, 2)] + m[(2, 0)]) / s,
        ]
    } else if m[(1, 1)] > m[(2, 2)] {
        let s = 2.0 * (1.0 + m[(1, 1)] - m[(0, 0)] - m[(2, 2)]).sqrt();
        [
            (m[(0, 2)] - m[(2, 0)]) / s,
            (m[(0, 1)] + m[(1, 0)]) / s,
            0.25 * s,
            (m[(1, 2)] + m[(2, 1)]) / s,
        ]
    } else {
        let s = 2.0 * (1.0 + m[(2, 2)] - m[(0, 0)] - m[(1, 1)]).sqrt();
        [
            (m[(1, 0)] - m[(0, 1)]) / s,
            (m[(0, 2)] + m[(2, 0)]) / s,
            (m[(1, 2)] + m[(2, 1)]) / s,
            0.25 * s,
        ]
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