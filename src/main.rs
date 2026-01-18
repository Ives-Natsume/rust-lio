use flume;
use rust_lio::utils::structs::MeasureGroup;
use rust_lio::utils::logging;
use rust_lio::config::{CONFIG, read_config};
use std::thread;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    read_config()?;

    let _logging_guard = logging::init_logging("logs", "rust-lio", &CONFIG.get().unwrap().log_level);
    let (tx, rx) = flume::unbounded::<MeasureGroup>();

    // Initialize and get the full SLAM context
    let mut ctx = rust_lio::frontend::sensor::sensor_init(&CONFIG.get().unwrap().lidar).await?;
    
    tracing::info!("SLAM context initialized, IMU ready: {}", ctx.is_initialized());
    tracing::info!("Initial gravity estimate: {:?}", ctx.get_state().grav);

    thread::spawn(move || {
        some_function(rx);
    });

    loop {
        // Use context's process_next for integrated processing
        match ctx.process_next().await {
            Ok(group) => {
                if let Err(e) = tx.send(group) {
                    tracing::error!("Failed to send packet to processing thread: {}", e);
                }
            }
            Err(e) => {
                tracing::warn!("Processing error: {}", e);
            }
        }
    }
}

/// Test only
fn some_function(receiver: flume::Receiver<MeasureGroup>) {
    tracing::info!("Starting some_function thread");
    while let Ok(group) = receiver.recv() {
        tracing::info!("Received MeasureGroup: {} packets, {} IMU measurements, timestamp {}", 
            group.points.len(), group.imus.len(), group.lidar_end_time);
        // Process the group...
    }
}
