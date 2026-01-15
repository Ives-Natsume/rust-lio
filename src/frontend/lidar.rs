use crate::io::lidar_driver::*;
use crate::utils::structs::MeasureGroup;
use crate::config::LidarConfig;
use core::time;
use std::sync::{Arc, Mutex};

/// Check and correct time synchronization between LIDAR and IMU data
pub fn time_sync(
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

pub async fn init_lidar(config: &LidarConfig) -> anyhow::Result<Arc<Mutex<UdpBridge>>> {
    tracing::info!("Initializing LIDAR");
    let lidar_addr = config.lidar_bind_addr.as_str();
    let imu_addr = config.imu_bind_addr.as_str();

    let bridge = UdpBridge::new(lidar_addr, imu_addr).await?;
    let arc_bridge = Arc::new(Mutex::new(bridge));

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

    Ok(arc_bridge)
}