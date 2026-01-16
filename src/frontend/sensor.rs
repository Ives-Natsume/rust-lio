//! Sensor initialization
//! 
//! For MID360 only
use crate::{io::lidar_driver::*, utils::structs::ImuProcess};
use crate::utils::structs::MeasureGroup;
use crate::config::LidarConfig;
use crate::core::math::ikfom::{EsEkfom, G_M_S2, MAX_INI_COUNT};
use crate::core::state::SlamContext;
use sophus::nalgebra::SMatrix;
use std::sync::{Arc, Mutex};

impl SlamContext {
    /// Process next measurement group through the pipeline
    /// 
    /// This method:
    /// 1. Gets the next packet from the sensor bridge
    /// 2. Performs time synchronization check
    /// 3. (TODO) Performs IMU forward propagation
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
        
        // TODO: Add IMU forward propagation here
        // self.imu_processor.process(&group, &mut self.kf);
        
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

/// Check and correct time synchronization between LIDAR and IMU data
fn time_sync(
    payload: &mut MeasureGroup,
) -> anyhow::Result<()> {
    if payload.imus.is_empty() || payload.points.points.is_empty() {
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

pub async fn sensor_init(config: &LidarConfig) -> anyhow::Result<SlamContext> {
    tracing::info!("Initializing MID360, data source: {:?}", config.data_source);

    let arc_bridge: Arc<Mutex<dyn SlamBridge>>;

    match config.data_source {
        crate::config::DataSource::Udp => {
            let bridge = UdpBridge::new(
                &config.lidar_bind_addr,
                &config.imu_bind_addr,
                &config.frame_time,
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

    {
        let mut init_bridge = arc_bridge.lock().unwrap();
        loop {
            let group = init_bridge.next_packet().await?;
            if imu_processor.imu_init(&group, &mut kf) {
                break;
            }
        }
    }

    Ok(SlamContext {
        bridge: arc_bridge,
        imu_processor,
        kf,
    })
}

/// Initialize sensor and return only the bridge (legacy API)
/// 
/// Prefer using `sensor_init` which returns the full `SlamContext`
pub async fn sensor_init_bridge_only(config: &LidarConfig) -> anyhow::Result<Arc<Mutex<dyn SlamBridge>>> {
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
            self.first_lidar_time = payload.lidar_begin_time;
        }

        // Compute running mean and covariance using Welford's algorithm
        for imu in &payload.imus {
            let cur_acc = imu.acc;
            let cur_gyr = imu.gyr;
            let n = self.init_iter_num as f64;

            // Update mean: mean += (x - mean) / N
            self.mean_acc += (cur_acc - self.mean_acc) / n;
            self.mean_gyr += (cur_gyr - self.mean_gyr) / n;

            // Update covariance using online algorithm
            // cov = cov * (N-1)/N + (x - mean)^2 / N
            let acc_diff = cur_acc - self.mean_acc;
            let gyr_diff = cur_gyr - self.mean_gyr;
            
            self.cov_acc = self.cov_acc * (n - 1.0) / n 
                + acc_diff.component_mul(&acc_diff) / n;
            self.cov_gyr = self.cov_gyr * (n - 1.0) / n 
                + gyr_diff.component_mul(&gyr_diff) / n / n * (n - 1.0);

            self.init_iter_num += 1;
        }

        // Initialize state from computed statistics
        let mut init_state = kf_state.get_x().clone();
        
        // Gravity: normalize mean_acc to unit vector, multiply by G
        // grav = -mean_acc / |mean_acc| * G
        let acc_norm = self.mean_acc.norm();
        if acc_norm > 1e-6 {
            init_state.grav = -self.mean_acc / acc_norm * G_M_S2;
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
            let scale = (G_M_S2 / acc_norm).powi(2);
            self.cov_acc *= scale;
            
            // Use configured scale values
            self.cov_acc = self.cov_acc_scale;
            self.cov_gyr = self.cov_gyr_scale;
            
            self.imu_need_init_ = false;
            tracing::info!("IMU initialization complete after {} iterations", self.init_iter_num);
            tracing::info!("  Gravity estimate: [{:.4}, {:.4}, {:.4}]", 
                kf_state.get_x().grav[0], 
                kf_state.get_x().grav[1], 
                kf_state.get_x().grav[2]);
            tracing::info!("  Gyro bias: [{:.6}, {:.6}, {:.6}]",
                kf_state.get_x().bg[0],
                kf_state.get_x().bg[1],
                kf_state.get_x().bg[2]);
            return true;
        }

        false
    }
}