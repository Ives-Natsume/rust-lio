use std::sync::OnceLock;
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
    pub max_boundary: f64,          // LIDAR point max distance boundary
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppConfig {
    pub lidar: LidarConfig,
    pub ikd_tree: IkdTreeConfig,
    pub log_level: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IkdTreeConfig {
    pub delete_criterion_param: f64,
    pub balance_criterion_param: f64,
    pub downsample_size: f64,
}

pub static CONFIG: OnceLock<AppConfig> = OnceLock::new();

pub fn read_config() -> anyhow::Result<()> {
    let path = "config.toml";
    let config_str = std::fs::read_to_string(path)?;
    let config: AppConfig = match toml::from_str(&config_str) {
        Ok(cfg) => cfg,
        Err(e) => {
            eprintln!("Failed to parse config file {}: {}", path, e);
            AppConfig {
                lidar: LidarConfig {
                    data_source: DataSource::Udp,
                    frame_time: 50,
                    lidar_bind_addr: "0.0.0.0:56301".to_string(),
                    imu_bind_addr: "0.0.0.0:56401".to_string(),
                    max_boundary: 5.0,
                },
                ikd_tree: IkdTreeConfig {
                    delete_criterion_param: 0.5,
                    balance_criterion_param: 0.6,
                    downsample_size: 0.05,
                },
                log_level: "info".to_string(),
            }
        }
    };

    CONFIG.set(config.clone()).unwrap();

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_read_config() {
        read_config().unwrap();
        let config = CONFIG.get().unwrap();
        println!("Config: {:?}", config);
        assert!(matches!(config.lidar.data_source, DataSource::Udp | DataSource::Ros));
    }
}