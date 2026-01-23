use flume;
use rust_lio::utils::structs::MeasureGroup;
use rust_lio::utils::logging;
use rust_lio::frontend::sensor::sensor_init;
use rust_lio::config::{CONFIG, read_config};
use rust_lio::core::state::SlamContext;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    read_config()?;

    let _logging_guard = logging::init_logging("logs", "rust-lio", &CONFIG.get().unwrap().log_level);
    // let (_tx, _rx) = flume::unbounded::<MeasureGroup>();

    // Initialize and get the full SLAM context
    let mut ctx: SlamContext = sensor_init(&CONFIG.get().unwrap()).await?;
    
    tracing::info!("SLAM context initialized, IMU ready: {}", ctx.is_initialized());
    tracing::info!("Initial gravity estimate: {:?}", ctx.get_state().grav);
    tracing::info!("Initial ikd_tree size: {}", ctx.ikd_tree.size());

    use tokio::time::{Duration};
    let run_duration = Duration::from_secs(20);
    let now = tokio::time::Instant::now();
    let socket = tokio::net::UdpSocket::bind("0.0.0.0:0").await?;
    let target: std::net::SocketAddr = "127.0.0.1:9000".parse()?;
    while tokio::time::Instant::now() - now < run_duration {
        ctx.process().await;
        ctx.publish_state_udp(&socket, target).await?;
    }

    ctx.ikd_tree.export_tree_to_txt("logs/long_run_tree.txt").await.unwrap();
    
    Ok(())
}

/// Test only
fn _some_function(receiver: flume::Receiver<MeasureGroup>) {
    tracing::info!("Starting some_function thread");
    while let Ok(group) = receiver.recv() {
        tracing::info!("Received MeasureGroup: {} packets, {} IMU measurements, timestamp {}", 
            group.points.len(), group.imus.len(), group.lidar_end_time);
        // Process the group...
    }
}
