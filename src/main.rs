use flume;
use rust_lio::io::lidar_driver::*;
use rust_lio::utils::structs::MeasureGroup;
use std::thread;
use rust_lio::utils::logging;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let _logging_guard = logging::init_logging("logs", "rust-lio-");
    let (tx, rx) = flume::unbounded::<MeasureGroup>();

    let lidar_config = rust_lio::config::LidarConfig {
        data_source: rust_lio::config::DataSource::Udp,
        lidar_bind_addr: "0.0.0.0:56301".to_string(),
        imu_bind_addr: "0.0.0.0:56401".to_string(),
    };

    let bridge = rust_lio::frontend::lidar::init_lidar(&lidar_config).await?;

    thread::spawn(move || {
        some_function(rx);
    });

    loop {
        let group = bridge.lock().unwrap().next_packet().await?;
        if let Err(e) = tx.send(group) {
            tracing::error!("Failed to send packet to processing thread: {}", e);
        }
    }
}

/// Test only
fn some_function(receiver: flume::Receiver<MeasureGroup>) {
    tracing::info!("Starting some_function thread");
    while let Ok(group) = receiver.recv() {
        tracing::info!("Received MeasureGroup: {} points, {} IMU measurements, timestamp {}", 
            group.points.points.len(), group.imus.len(), group.lidar_end_time);
        // Process the group...
    }
}
