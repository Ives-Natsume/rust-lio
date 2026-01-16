use crate::{io::lidar_driver::*, utils::structs::ImuProcess};
use crate::core::math::ikfom::{EsEkfom};
use std::sync::{Arc, Mutex};

/// Processing context that bundles all stateful SLAM components
/// 
/// This struct owns the sensor bridge and processing state,
/// providing a unified interface for the SLAM pipeline.
pub struct SlamContext {
    /// Sensor data bridge (LiDAR + IMU)
    pub bridge: Arc<Mutex<dyn SlamBridge>>,
    /// IMU processor with calibration and state
    pub imu_processor: ImuProcess,
    /// Error-state Kalman filter
    pub kf: EsEkfom,
}