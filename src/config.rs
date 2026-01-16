use config;
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DataSource {
    Udp,
    Ros,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LidarConfig {
    pub data_source: DataSource,
    pub frame_time: u64,
    pub lidar_bind_addr: String,
    pub imu_bind_addr: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppConfig {
    pub lidar: LidarConfig,
    pub log_level: String,
    pub log_path: String,
}