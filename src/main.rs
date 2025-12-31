use flume;
use rust_lio::io::lidar_driver::*;
use rust_lio::utils::structs::MeasureGroup;
use std::thread;
use rust_lio::utils::logging;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let _logging_guard = logging::init_logging("logs", "rust-lio-");
    tracing::info!("Starting UDP to MeasureGroup bridge");

    let (tx, rx) = flume::unbounded::<MeasureGroup>();

    thread::spawn(move || {
        some_function(rx);
    });

    let lidar_addr: &str = "0.0.0.0:56301";
    let imu_addr: &str = "0.0.0.0:56401";
    let mut bridge = UdpBridge::new(lidar_addr, imu_addr).await?;

    loop {
        let group = bridge.next_packet().await?;
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
