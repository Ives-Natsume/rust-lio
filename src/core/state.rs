use crate::{io::lidar_driver::*, utils::structs::ImuProcess};
use crate::core::math::ikfom::{EsEkfom};
use crate::core::math::ikd_tree::{IkdTree, BoxPointType};
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
    /// idk-tree for mapping
    pub ikd_tree: IkdTree,
    /// First LiDAR timestamp (for initialization timing)
    pub first_lidar_time: f64,
    /// Local map bounds for sliding window
    pub local_map_bounds: BoxPointType,
    /// Whether local map has been initialized
    pub local_map_initialized: bool,
    /// Local map cube side length
    pub cube_len: f64,
    /// Detection range for FOV management
    pub det_range: f64,
}