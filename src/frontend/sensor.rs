//! Sensor initialization
//! 
//! For MID360 only
use crate::{
    config::AppConfig,
    io::lidar_driver::*,
    utils::structs::{ImuData, ImuPose, ImuProcess, PointCloudXYZI, PointXYZI}
};
use crate::utils::structs::MeasureGroup;
use crate::core::math::ikfom::{EsEkfom, InputIkfom, G_M_S2, MAX_INI_COUNT};
use crate::core::math::ikd_tree::{IkdTree, IkdTreePoint};
use crate::core::state::SlamContext;
use sophus::nalgebra::{SMatrix, Vector3};
use sophus::lie::Rotation3F64;
use std::sync::{Arc, Mutex};

/// Convert rotation matrix to Euler angles (roll, pitch, yaw) in radians
fn rotation_to_euler(rot: &Rotation3F64) -> Vector3<f64> {
    let m = rot.matrix();
    let pitch = (-m[(2, 0)]).asin();
    let roll;
    let yaw;
    
    if pitch.cos().abs() > 1e-6 {
        roll = m[(2, 1)].atan2(m[(2, 2)]);
        yaw = m[(1, 0)].atan2(m[(0, 0)]);
    } else {
        // Gimbal lock
        roll = 0.0;
        yaw = m[(0, 1)].atan2(m[(1, 1)]);
    }
    
    Vector3::new(roll, pitch, yaw)
}

impl SlamContext {
    /// Process next measurement group through the pipeline
    /// 
    /// This method:
    /// 1. Gets the next packet from the sensor bridge
    /// 2. Performs time synchronization check
    /// 3. Performs IMU forward propagation
    /// 
    /// # Returns
    /// The processed MeasureGroup, or an error
    pub async fn process_next(&mut self) -> anyhow::Result<MeasureGroup> {
        let mut group = {
            let mut bridge = self.bridge.lock().unwrap();
            bridge.next_packet().await?
        };
        
        // Time sync check
        time_sync(&mut group)?;
        
        // Undistort and forward propagate
        match self.imu_processor.undistort_pcl(&mut group, &mut self.kf) {
            Ok(_) => {}
            Err(e) => {
                tracing::warn!("Undistort point cloud failed: {}", e);
            }
        }

        tracing::debug!("Estimated Position: {:?}", self.kf.get_x().pos);
        
        Ok(group)
    }
    
    /// Get current state estimate
    pub fn get_state(&self) -> &crate::core::math::ikfom::StateIkfom {
        self.kf.get_x()
    }
    
    /// Check if IMU is initialized
    pub fn is_initialized(&self) -> bool {
        !self.imu_processor.imu_need_init_
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)]
    use super::*;
    // #[tokio::test]
    async fn test_save_one_frame() {
        assert_eq!(&crate::config::CONFIG.get().unwrap().lidar.max_boundary, &5.0);
        let mut ctx: SlamContext = crate::frontend::sensor::sensor_init(&crate::config::CONFIG.get().unwrap()).await.unwrap();
        ctx.ikd_tree.export_tree_to_txt("logs/test_tree.txt").await.unwrap();
        match ctx.process_next().await {
            Ok(group) => {
                let pcl: Vec<PointXYZI> = group.points.iter().flat_map(|p| p.points.clone()).collect();
                let cloud = PointCloudXYZI::from_points(pcl, group.lidar_begin_time);
                cloud.save_to_txt("logs/test_output.txt").unwrap();
            }
            Err(e) => {
                // println!("Processing error: {}", e);
            }
        }
    }

    #[allow(unused_imports)]
    use super::*;
    // #[tokio::test]
    async fn test_longtime_run() {
        let mut ctx: SlamContext = crate::frontend::sensor::sensor_init(&crate::config::CONFIG.get().unwrap()).await.unwrap();
        for i in 0..1000 {
            match ctx.process_next().await {
                Ok(_group) => {
                    if i == 0 {
                        ctx.ikd_tree.export_tree_to_txt("logs/test_longtime_run_tree_start.txt").await.unwrap();
                        println!("Initial tree exported.");
                    }
                    if i == 999 {
                        ctx.ikd_tree.export_tree_to_txt("logs/test_longtime_run_tree_end.txt").await.unwrap();
                        println!("Final tree exported.");
                    }
                }
                Err(e) => {

                }
            }
        }
    }

    use super::*;
    // #[tokio::test]
    async fn test_pcl_boundary() {
        let mut ctx: SlamContext = crate::frontend::sensor::sensor_init(&crate::config::CONFIG.get().unwrap()).await.unwrap();
        match ctx.process_next().await {
            Ok(group) => {
                let new_scan = group.points.iter().flat_map(|p| {
                    p.to_point_vector()
                }).collect();
                let config_max_boundary = crate::config::CONFIG.get().unwrap().lidar.max_boundary;
                ctx.ikd_tree.add_points(&new_scan, false);
                let box_point = ctx.ikd_tree.tree_range();
                let string = format!(
                    "Map boundary box: min({:.2}, {:.2}, {:.2}), max({:.2}, {:.2}, {:.2})\nConfig max boundary: {:.2}\n",
                    box_point.vertex_min[0], box_point.vertex_min[1], box_point.vertex_min[2],
                    box_point.vertex_max[0], box_point.vertex_max[1], box_point.vertex_max[2],
                    config_max_boundary
                );

                let _path = "logs/test_boundary.txt";
                std::fs::write("logs/test_boundary.txt", string).unwrap();
                assert_eq!(box_point.vertex_min[0].abs() < config_max_boundary, true);
                assert_eq!(box_point.vertex_min[1].abs() < config_max_boundary, true);
                assert_eq!(box_point.vertex_min[2].abs() < config_max_boundary, true);
                assert_eq!(box_point.vertex_max[0].abs() < config_max_boundary, true);
                assert_eq!(box_point.vertex_max[1].abs() < config_max_boundary, true);
                assert_eq!(box_point.vertex_max[2].abs() < config_max_boundary, true);
            }
            Err(e) => {
                println!("Processing error: {}", e);
            }
        }
    }
}

/// Check and correct time synchronization between LIDAR and IMU data
fn time_sync(
    payload: &mut MeasureGroup,
) -> anyhow::Result<()> {
    if payload.imus.is_empty() || payload.points.is_empty() {
        return Ok(());
    }

    let imu_start_time = payload.imus.first().unwrap().timestamp;
    let imu_end_time = payload.imus.last().unwrap().timestamp;
    let lidar_begin_time = payload.lidar_begin_time;
    let lidar_end_time = payload.lidar_end_time;

    // time overlap check
    if (imu_start_time > lidar_end_time) || (imu_end_time < lidar_begin_time) {
        return Err(anyhow::Error::msg("No overlapping time between IMU and LIDAR data"));
    }

    // time diff check
    let imu_mid_time = (imu_start_time + imu_end_time) / 2.0;
    let lidar_mid_time = (lidar_begin_time + lidar_end_time) / 2.0;
    let time_diff = imu_mid_time - lidar_mid_time;
    if time_diff.abs() > 0.1 {
        return Err(anyhow::Error::msg(format!(
            "Time difference too large: {}s",
            time_diff
        )));
    }

    Ok(())
}

pub async fn sensor_init(config: &AppConfig) -> anyhow::Result<SlamContext> {
    // TODO: code review
    tracing::info!("Initializing MID360, data source: {:?}", config.lidar.data_source);

    let arc_bridge: Arc<Mutex<dyn SlamBridge>>;

    match config.lidar.data_source {
        crate::config::DataSource::Udp => {
            let bridge = UdpBridge::new(
                &config.lidar.lidar_bind_addr,
                &config.lidar.imu_bind_addr,
                &config.lidar.frame_time,
            ).await?;
            arc_bridge = Arc::new(Mutex::new(bridge));
        }
        crate::config::DataSource::Ros => {
            unimplemented!("ROS data source is not implemented yet");
        }
    }

    // time synchronization test
    {
        let mut test_bridge = arc_bridge.lock().unwrap();
        let mut sync_result: Vec<bool> = Vec::new();
        for _ in 0..10 {
            let mut group = test_bridge.next_packet().await?;
            match time_sync(&mut group) {
                Ok(_) => {
                    sync_result.push(true);
                }
                Err(e) => {
                    tracing::warn!("Time synchronization test failed: {}", e);
                    sync_result.push(false);
                }
            }
        }
        if !sync_result.contains(&true) {
            return Err(anyhow::Error::msg("Time synchronization test failed after 10 attempts"));
        }
        tracing::info!("Time synchronization test passed with {} fails", sync_result.iter().filter(|&&x| x == false).count());
    }

    // imu init
    let mut kf = EsEkfom::new();
    let mut imu_processor = ImuProcess::new();
    let mut first_lidar_time = 0.0;

    {
        let mut init_bridge = arc_bridge.lock().unwrap();
        loop {
            let group = init_bridge.next_packet().await?;
            if first_lidar_time == 0.0 {
                first_lidar_time = group.lidar_begin_time;
            }
            if imu_processor.imu_init(&group, &mut kf) {
                break;
            }
        }
    }

    // build initial idk-tree
    let mut ikd_tree = IkdTree::new(
        config.ikd_tree.delete_criterion_param,
        config.ikd_tree.balance_criterion_param,
        config.ikd_tree.downsample_size
    );
    {
        let mut init_bridge = arc_bridge.lock().unwrap();
        let group = init_bridge.next_packet().await?;
        let new_scan: Vec<IkdTreePoint> = group.points.iter().flat_map(|p| {
            p.to_point_vector()
        }).collect();
        tracing::info!("Building initial ikd-tree with {} points", new_scan.len());
        // sensor keep still during imu initialization, so no undistortion needed
        ikd_tree.add_points(&new_scan, true);

    }

    Ok(SlamContext {
        bridge: arc_bridge,
        imu_processor,
        kf,
        ikd_tree,
        first_lidar_time,
        local_map_bounds: crate::core::math::ikd_tree::BoxPointType::default(),
        local_map_initialized: false,
        cube_len: 200.0,  // Default local map cube size
        det_range: 100.0,  // Default detection range
    })
}

/// Initialize sensor and return only the bridge (legacy API)
/// 
/// Prefer using `sensor_init` which returns the full `SlamContext`
pub async fn sensor_init_bridge_only(config: &AppConfig) -> anyhow::Result<Arc<Mutex<dyn SlamBridge>>> {
    let ctx = sensor_init(config).await?;
    Ok(ctx.bridge)
}
    
impl ImuProcess {
    /// IMU initialization: use the average of initial IMU frames to initialize state
    /// 
    /// This is equivalent to `IMU_init` in C++ S-FAST_LIO.
    /// Uses running average to compute mean acceleration and gyroscope values,
    /// then initializes gravity direction and gyroscope bias.
    /// 
    /// # Arguments
    /// * `payload` - MeasureGroup containing IMU data and LiDAR timestamps
    /// * `kf_state` - Error-state EKF on manifold for state estimation
    /// 
    /// # Returns
    /// * `true` if initialization is complete (enough iterations)
    /// * `false` if more iterations needed
    pub fn imu_init(
        &mut self,
        payload: &MeasureGroup,
        kf_state: &mut EsEkfom,
    ) -> bool {
        if payload.imus.is_empty() {
            return false;
        }

        // First frame handling
        if self.b_first_frame_ {
            self.reset();
            self.init_iter_num = 1;
            self.b_first_frame_ = false;
            
            // Use first IMU measurement as initial mean
            let first_imu = &payload.imus[0];
            self.mean_acc = first_imu.acc;
            self.mean_gyr = first_imu.gyr;
            self.cov_acc = Vector3::zeros();
            self.cov_gyr = Vector3::zeros();
            self.first_lidar_time = payload.lidar_begin_time;
        }

        for imu in &payload.imus {
            self.update_imu_statistics(imu);
        }

        // Initialize state from computed statistics
        let mut init_state = kf_state.get_x().clone();
        
        // Gravity initialization:
        // The accelerometer measures the reaction to gravity: acc ≈ -g (in body frame)
        // When sensor is stationary and level, mean_acc ≈ [0, 0, G] (pointing up)
        // We need to:
        // 1. Compute initial rotation that aligns measured gravity with world Z-down
        // 2. Set gravity vector in world frame as [0, 0, -G]
        let acc_norm = self.mean_acc.norm();
        if acc_norm > 1e-6 {
            // Measured gravity direction in body frame (normalized, pointing opposite to gravity)
            let grav_body_normalized = self.mean_acc / acc_norm;
            
            // Compute rotation from body frame to world frame
            // This rotation aligns -grav_body_normalized with grav_world_dir [0, 0, -1]
            // i.e., R * (-grav_body_normalized) = [0, 0, -1]
            // So R * grav_body_normalized = [0, 0, 1]
            let target_dir = Vector3::new(0.0, 0.0, 1.0);
            
            // Compute rotation using Rodrigues' formula
            let v = grav_body_normalized.cross(&target_dir);
            let c = grav_body_normalized.dot(&target_dir);
            
            let init_rot = if c > 0.9999 {
                // Already aligned
                Rotation3F64::identity()
            } else if c < -0.9999 {
                // Opposite direction, rotate 180 degrees around X axis
                Rotation3F64::exp(Vector3::new(std::f64::consts::PI, 0.0, 0.0))
            } else {
                // General case: axis-angle rotation
                let axis = v / v.norm();
                let angle = c.acos();
                Rotation3F64::exp(axis * angle)
            };
            
            init_state.rot = init_rot;
            
            // Gravity in world frame is always [0, 0, -G]
            init_state.grav = Vector3::new(0.0, 0.0, -G_M_S2);
        }
        
        // Gyroscope bias: use mean gyroscope as initial bias
        init_state.bg = self.mean_gyr;
        
        // Set LiDAR-IMU extrinsic calibration
        init_state.offset_t_l_i = self.lidar_t_wrt_imu;
        init_state.offset_r_l_i = self.lidar_r_wrt_imu.clone();
        
        kf_state.change_x(init_state);

        // Initialize covariance matrix P (24x24)
        let mut init_p = SMatrix::<f64, 24, 24>::identity();
        
        // Position covariance (indices 6-8)
        init_p[(6, 6)] = 0.00001;
        init_p[(7, 7)] = 0.00001;
        init_p[(8, 8)] = 0.00001;
        
        // Velocity covariance (indices 9-11)
        init_p[(9, 9)] = 0.00001;
        init_p[(10, 10)] = 0.00001;
        init_p[(11, 11)] = 0.00001;
        
        // Gyroscope bias covariance (indices 15-17)
        init_p[(15, 15)] = 0.0001;
        init_p[(16, 16)] = 0.0001;
        init_p[(17, 17)] = 0.0001;
        
        // Accelerometer bias covariance (indices 18-20)
        init_p[(18, 18)] = 0.001;
        init_p[(19, 19)] = 0.001;
        init_p[(20, 20)] = 0.001;
        
        // Gravity covariance (indices 21-23)
        init_p[(21, 21)] = 0.00001;
        init_p[(22, 22)] = 0.00001;
        init_p[(23, 23)] = 0.00001;
        
        kf_state.change_p(init_p);

        // Save last IMU for next iteration
        self.last_imu = payload.imus.last().cloned();

        // Check if initialization is complete
        if self.init_iter_num > MAX_INI_COUNT {
            // Scale covariance by gravity ratio
            // let scale = (G_M_S2 / acc_norm).powi(2);
            // self.cov_acc *= scale;
            
            // Use configured scale values
            self.cov_acc = self.cov_acc_scale;
            self.cov_gyr = self.cov_gyr_scale;
            
            self.imu_need_init_ = false;
            tracing::info!("IMU initialization complete after {} iterations", self.init_iter_num);
            tracing::info!("  Mean acc (body): [{:.4}, {:.4}, {:.4}], norm: {:.4}",
                self.mean_acc[0], self.mean_acc[1], self.mean_acc[2], self.mean_acc.norm());
            tracing::info!("  Gravity estimate (world): [{:.4}, {:.4}, {:.4}]", 
                kf_state.get_x().grav[0],
                kf_state.get_x().grav[1],
                kf_state.get_x().grav[2]);
            let rot_euler = rotation_to_euler(&kf_state.get_x().rot);
            tracing::info!("  Initial rotation (RPY deg): [{:.2}, {:.2}, {:.2}]",
                rot_euler[0].to_degrees(), rot_euler[1].to_degrees(), rot_euler[2].to_degrees());
            tracing::info!("  Gyro bias: [{:.6}, {:.6}, {:.6}]",
                kf_state.get_x().bg[0],
                kf_state.get_x().bg[1],
                kf_state.get_x().bg[2]);
            return true;
        }

        false
    }

    fn update_imu_statistics(&mut self, imu: &ImuData) {
        let cur_acc = imu.acc;
        let cur_gyr = imu.gyr;
        let n = self.init_iter_num as f64;
        let new_n = n + 1.0;

        let old_mean_acc = self.mean_acc;
        let old_mean_gyr = self.mean_gyr;

        // Update mean: mean = (x + n * mean) / (n + 1)
        self.mean_acc = (cur_acc + n * self.mean_acc) / new_n;
        self.mean_gyr = (cur_gyr + n * self.mean_gyr) / new_n;

        // Update covariance using Welford's algorithm
        let delta_acc = cur_acc - old_mean_acc;
        let delta_gyr = cur_gyr - old_mean_gyr;
        let delta_acc_new = cur_acc - self.mean_acc;
        let delta_gyr_new = cur_gyr - self.mean_gyr;

        self.cov_acc = n / new_n * self.cov_acc + delta_acc.component_mul(&delta_acc_new) / new_n;
        self.cov_gyr = n / new_n * self.cov_gyr + delta_gyr.component_mul(&delta_gyr_new) / new_n;

        self.init_iter_num += 1;
    }

    /// Undistort point cloud using IMU forward propagation
    /// 
    /// This is equivalent to `UndistortPcl` in C++ S-FAST_LIO.
    /// 
    /// Updates the provided `payload` point cloud in place
    /// 
    /// Algorithm:
    /// 1. Prepend last IMU from previous frame to current IMU queue
    /// 2. Sort point cloud by timestamp
    /// 3. Forward propagate through all IMU measurements, storing intermediate poses
    /// 4. Backward propagate to compensate each point's motion distortion
    /// 
    /// # Arguments
    /// * `payload` - MeasureGroup containing IMU data and point cloud
    /// * `kf_state` - Kalman filter state (will be updated)
    pub fn undistort_pcl(
        &mut self,
        payload: &mut MeasureGroup,
        kf_state: &mut EsEkfom,
    ) -> anyhow::Result<()> {
        // Build IMU queue: prepend last IMU from previous frame
        let mut v_imu: Vec<ImuData> = Vec::with_capacity(payload.imus.len() + 1);
        if let Some(ref last) = self.last_imu {
            v_imu.push(last.clone());
        }
        v_imu.extend(payload.imus.iter().cloned());
        
        if v_imu.len() < 2 {
            return Err(anyhow::Error::msg("Not enough IMU data for undistortion"));
        }
        
        let imu_end_time = v_imu.last().unwrap().timestamp;
        let pcl_beg_time = payload.lidar_begin_time;
        let pcl_end_time = payload.lidar_end_time;
        
        // Collect and sort all points by timestamp
        let mut pcl_out: Vec<PointXYZI> = payload.points
            .iter()
            .flat_map(|pc| pc.points.clone())
            .collect();
        pcl_out.sort_by(|a, b| a.timestamp.partial_cmp(&b.timestamp).unwrap_or(std::cmp::Ordering::Equal));
        
        if pcl_out.is_empty() {
            return Err(anyhow::Error::msg("No points in point cloud for undistortion"));
        }
        
        // Get initial state from KF
        let mut imu_state = kf_state.get_x().clone();
        
        // Store intermediate poses for backward propagation
        let mut imu_poses: Vec<ImuPose> = Vec::with_capacity(v_imu.len());
        imu_poses.push(ImuPose::new(
            0.0,
            self.acc_s_last,
            self.angvel_last,
            imu_state.vel,
            imu_state.pos,
            imu_state.rot.matrix(),
        ));
        
        // ========== Forward Propagation ==========
        let mut input = InputIkfom::default();
        
        for i in 0..(v_imu.len() - 1) {
            let head = &v_imu[i];
            let tail = &v_imu[i + 1];
            
            // Skip if tail timestamp is before last lidar end time
            if tail.timestamp < self.last_lidar_end_time_ {
                continue;
            }
            
            // Midpoint integration for angular velocity and acceleration
            let angvel_avr = (head.gyr + tail.gyr) * 0.5;
            let acc_avr_raw = (head.acc + tail.acc) * 0.5;

            // Scale acceleration by gravity ratio (normalize to actual gravity)
            let acc_avr = acc_avr_raw * G_M_S2 / self.mean_acc.norm();

            // Normal input
            input.acc = acc_avr;
            input.gyro = angvel_avr;
            
            // Compute dt (handle case where head is before last lidar end)
            let dt = if head.timestamp < self.last_lidar_end_time_ {
                tail.timestamp - self.last_lidar_end_time_
            } else {
                tail.timestamp - head.timestamp
            };
            
            if dt <= 0.0 {
                continue;
            }
            
            // Update Q matrix with current covariances
            self.q.fixed_view_mut::<3, 3>(0, 0).fill_diagonal(self.cov_gyr[0]);
            self.q.fixed_view_mut::<3, 3>(3, 3).fill_diagonal(self.cov_acc[0]);
            self.q.fixed_view_mut::<3, 3>(6, 6).fill_diagonal(self.cov_gyr_bias[0]);
            self.q.fixed_view_mut::<3, 3>(9, 9).fill_diagonal(self.cov_acc_bias[0]);
            
            // Forward propagate Kalman filter
            kf_state.predict(dt, &self.q, &input);
            
            // Update state
            imu_state = kf_state.get_x().clone();
            
            // Update last angular velocity (bias-corrected)
            self.angvel_last = tail.gyr - imu_state.bg;
            
            // Update last world-frame acceleration
            // acc_s_last = R * (acc_scaled - ba) + grav
            let acc_scaled = tail.acc * G_M_S2 / self.mean_acc.norm();
            let rot_matrix = imu_state.rot.matrix();
            self.acc_s_last = rot_matrix * (acc_scaled - imu_state.ba) + imu_state.grav;
            
            // Store pose for backward propagation
            let offset_t = tail.timestamp - pcl_beg_time;
            imu_poses.push(ImuPose::new(
                offset_t,
                self.acc_s_last,
                self.angvel_last,
                imu_state.vel,
                imu_state.pos,
                rot_matrix,
            ));
            // tracing::debug!("IMU pose at t={:.6}s: pos=[{:.4}, {:.4}, {:.4}], vel=[{:.4}, {:.4}, {:.4}]",
            //     offset_t,
            //     imu_state.pos[0], imu_state.pos[1], imu_state.pos[2],
            //     imu_state.vel[0], imu_state.vel[1], imu_state.vel[2]);
        }
        
        // Propagate to lidar end time
        let dt_final = (pcl_end_time - imu_end_time).abs();
        if dt_final > 0.0 {
            kf_state.predict(dt_final, &self.q, &input);
            imu_state = kf_state.get_x().clone();
        }
        
        // Save state for next frame
        self.last_imu = payload.imus.last().cloned();
        self.last_lidar_end_time_ = pcl_end_time;
        
        // ========== Backward Propagation (Point Undistortion) ==========
        if imu_poses.len() < 2 {
            return Err(anyhow::Error::msg("Not enough IMU poses for undistortion"));
        }
        
        let mut it_pcl = pcl_out.len() - 1;
        
        // Iterate through IMU poses in reverse
        for kp_idx in (1..imu_poses.len()).rev() {
            let head = &imu_poses[kp_idx - 1];
            let tail = &imu_poses[kp_idx];
            
            let r_imu = head.rot;
            let vel_imu = head.vel;
            let pos_imu = head.pos;
            let acc_imu = tail.acc;
            let angvel_avr = tail.gyr;
            
            // Process points within this IMU interval
            // Note: pcl_out[].timestamp is absolute time, convert to offset from pcl_beg_time
            while it_pcl > 0 && (pcl_out[it_pcl].timestamp - pcl_beg_time) > head.offset_time {
                // dt = point's offset time - head's offset time
                let pt_offset_time = pcl_out[it_pcl].timestamp - pcl_beg_time;
                let dt = pt_offset_time - head.offset_time;
                
                // Compute rotation at point time: R_i = R_head * exp(omega * dt)
                let omega_dt = angvel_avr * dt;
                let delta_rot = Rotation3F64::exp(omega_dt);
                let r_i = r_imu * delta_rot.matrix();
                
                // Translation from point time to end time (world frame)
                // T_ei = pos_at_point_time - pos_at_end_time
                let t_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos;
                
                // Transform point to end-of-frame position
                // Formula: P_compensate = R_L_I^T * (R_end^T * (R_i * (R_L_I * P_i + T_L_I) + T_ei) - T_L_I)
                // 
                // This compensates for the motion during the scan:
                // 1. Transform point from LiDAR to IMU frame: R_L_I * P_i + T_L_I
                // 2. Rotate to world frame at point capture time: R_i * (...)
                // 3. Add translation offset: ... + T_ei
                // 4. Rotate back to IMU frame at end time: R_end^T * (...)
                // 5. Remove IMU-LiDAR offset: ... - T_L_I
                // 6. Rotate to LiDAR frame: R_L_I^T * (...)
                let offset_r = imu_state.offset_r_l_i.matrix();
                let offset_t = imu_state.offset_t_l_i;
                let rot_end = imu_state.rot.matrix();
                
                // Point position in LiDAR frame
                let p_i = Vector3::new(
                    pcl_out[it_pcl].pos[0] as f64,
                    pcl_out[it_pcl].pos[1] as f64,
                    pcl_out[it_pcl].pos[2] as f64,
                );
                
                // Apply the complete transformation in one expression (matching C++ exactly):
                // P_compensate = R_L_I^T * (R_end^T * (R_i * (R_L_I * P_i + T_L_I) + T_ei) - T_L_I)
                let p_compensate = offset_r.transpose() * 
                    (rot_end.transpose() * (r_i * (offset_r * p_i + offset_t) + t_ei) - offset_t);
                
                pcl_out[it_pcl].pos[0] = p_compensate[0] as f32;
                pcl_out[it_pcl].pos[1] = p_compensate[1] as f32;
                pcl_out[it_pcl].pos[2] = p_compensate[2] as f32;
                
                if it_pcl == 0 {
                    break;
                }
                it_pcl -= 1;
            }
        }
        
        let width = pcl_out.len() as u32;
        let undistorted_points: PointCloudXYZI = PointCloudXYZI {
            points: pcl_out,
            height: 1,
            width: width,
            timestamp: pcl_beg_time,
            is_dense: true,
        };

        payload.points = vec![undistorted_points];

        Ok(())
    }
}