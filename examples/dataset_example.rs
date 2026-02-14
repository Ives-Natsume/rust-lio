//! Example: Using Park Dataset Adapter
//! 
//! This example demonstrates how to use the park dataset adapter
//! to test SLAM algorithms with offline data.

use rust_lio::io::dataset_adapter::ParkDatasetBridge;
use rust_lio::io::lidar_driver::SlamBridge;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    // Create dataset bridge
    let dataset_path = "reference/dataset/park";
    let fps = 10.0; // MID360 typical rate
    
    println!("Loading park dataset from: {}", dataset_path);
    let mut bridge = ParkDatasetBridge::new(dataset_path, fps)?;
    
    // Process first 10 frames as example
    let mut frame_count = 0;
    let max_frames = 10;
    
    while bridge.has_next() && frame_count < max_frames {
        // Get next frame
        let group = bridge.next_packet().await?;
        
        // Print frame information
        println!("\n=== Frame {} ===", frame_count);
        println!("  Time range: {:.3} - {:.3} seconds", 
                 group.lidar_begin_time, group.lidar_end_time);
        println!("  Point clouds: {}", group.points.len());
        
        if !group.points.is_empty() {
            let pcl = &group.points[0];
            println!("  Points in first cloud: {}", pcl.points.len());
            
            // Print first few points
            if pcl.points.len() > 0 {
                println!("  Sample points:");
                for (i, point) in pcl.points.iter().take(3).enumerate() {
                    println!("    Point {}: pos=({:.3}, {:.3}, {:.3}), intensity={:.1}, time={:.6}",
                             i, point.pos.x, point.pos.y, point.pos.z, 
                             point.intensity, point.timestamp);
                }
            }
        }
        
        println!("  IMU measurements: {}", group.imus.len());
        println!("  Progress: {:.1}%", bridge.progress());
        
        frame_count += 1;
    }
    
    println!("\n✓ Successfully processed {} frames", frame_count);
    println!("Total frames available: {}", 
             if bridge.has_next() { "more than processed" } else { "all processed" });
    
    Ok(())
}
