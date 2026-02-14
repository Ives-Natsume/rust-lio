//! Dataset Adapter for Park Dataset from HKU-MARS HBA
//! 
//! Reads offline PCD files and pose.json to simulate real-time data stream
//! Dataset format: https://github.com/hku-mars/HBA

use std::fs::{self, File};
use std::io::{BufReader, Read, Cursor, BufRead};
use std::path::{Path, PathBuf};
use byteorder::{LittleEndian, ReadBytesExt};
use sophus::nalgebra::Vector3;
use crate::utils::structs::*;
use crate::io::lidar_driver::SlamBridge;

/// Pose data from pose.json
/// Format: tx ty tz qw qx qy qz (translation + quaternion)
#[derive(Debug, Clone)]
pub struct PoseData {
    pub translation: Vector3<f64>,
    pub quaternion: [f64; 4], // [qw, qx, qy, qz]
}

/// Park Dataset Bridge - implements SlamBridge for offline dataset
pub struct ParkDatasetBridge {
    dataset_path: PathBuf,
    poses: Vec<PoseData>,
    current_index: usize,
    frame_interval: f64,  // Time interval between frames (seconds)
    start_time: f64,      // Simulated start time
}

impl ParkDatasetBridge {
    /// Create a new ParkDatasetBridge
    /// 
    /// # Arguments
    /// * `dataset_path` - Path to the park dataset directory (containing pcd/ and pose.json)
    /// * `fps` - Frames per second (default: 10 for MID360)
    pub fn new<P: AsRef<Path>>(dataset_path: P, fps: f64) -> anyhow::Result<Self> {
        let dataset_path = dataset_path.as_ref().to_path_buf();
        
        // Check if dataset directory exists
        if !dataset_path.exists() {
            return Err(anyhow::anyhow!("Dataset path does not exist: {:?}", dataset_path));
        }

        // Load poses from pose.json
        let poses = Self::load_poses(&dataset_path)?;
        
        tracing::info!("Loaded {} poses from park dataset", poses.len());
        
        Ok(Self {
            dataset_path,
            poses,
            current_index: 0,
            frame_interval: 1.0 / fps,
            start_time: 0.0,
        })
    }

    /// Load pose data from pose.json
    fn load_poses<P: AsRef<Path>>(dataset_path: P) -> anyhow::Result<Vec<PoseData>> {
        let pose_file = dataset_path.as_ref().join("pose.json");
        let content = fs::read_to_string(&pose_file)
            .map_err(|e| anyhow::anyhow!("Failed to read pose.json: {}", e))?;
        
        let mut poses = Vec::new();
        for line in content.lines() {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 7 {
                continue;
            }
            
            let tx = parts[0].parse::<f64>()?;
            let ty = parts[1].parse::<f64>()?;
            let tz = parts[2].parse::<f64>()?;
            let qw = parts[3].parse::<f64>()?;
            let qx = parts[4].parse::<f64>()?;
            let qy = parts[5].parse::<f64>()?;
            let qz = parts[6].parse::<f64>()?;
            
            poses.push(PoseData {
                translation: Vector3::new(tx, ty, tz),
                quaternion: [qw, qx, qy, qz],
            });
        }
        
        Ok(poses)
    }

    /// Load a single PCD file
    /// Supports binary PCD format (0.7) with fields: x y z intensity normal_x normal_y normal_z curvature
    fn load_pcd<P: AsRef<Path>>(pcd_path: P) -> anyhow::Result<PointCloudXYZI> {
        let file = File::open(&pcd_path)
            .map_err(|e| anyhow::anyhow!("Failed to open PCD file: {}", e))?;
        let mut reader = BufReader::new(file);
        
        // Parse header
        let mut header_lines = Vec::new();
        let mut line = String::new();
        let mut num_points = 0usize;
        let mut data_format = String::new();
        
        loop {
            line.clear();
            let bytes_read = reader.read_line(&mut line)?;
            if bytes_read == 0 {
                break;
            }
            
            let trimmed = line.trim();
            if trimmed.starts_with("POINTS ") {
                num_points = trimmed.split_whitespace()
                    .nth(1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0);
            } else if trimmed.starts_with("DATA ") {
                data_format = trimmed.split_whitespace()
                    .nth(1)
                    .unwrap_or("")
                    .to_string();
                break;
            }
            
            header_lines.push(line.clone());
        }
        
        if num_points == 0 {
            return Ok(PointCloudXYZI::new());
        }
        
        // Read binary point data
        let mut points = Vec::with_capacity(num_points);
        
        if data_format == "binary" {
            let mut buffer = Vec::new();
            reader.read_to_end(&mut buffer)?;
            let mut cursor = Cursor::new(buffer);
            
            // Each point: x(4) y(4) z(4) intensity(4) normal_x(4) normal_y(4) normal_z(4) curvature(4) = 32 bytes
            
            for _ in 0..num_points {
                let x = cursor.read_f32::<LittleEndian>()?;
                let y = cursor.read_f32::<LittleEndian>()?;
                let z = cursor.read_f32::<LittleEndian>()?;
                let intensity = cursor.read_f32::<LittleEndian>()?;
                
                // Skip normal and curvature (4 * 4 = 16 bytes)
                cursor.set_position(cursor.position() + 16);
                
                points.push(PointXYZI {
                    pos: Vector3::new(x, y, z),
                    intensity,
                    timestamp: 0.0, // Will be set later
                });
            }
        } else {
            return Err(anyhow::anyhow!("Only binary PCD format is supported"));
        }
        
        Ok(PointCloudXYZI::from_points(points, 0.0))
    }

    /// Check if more frames are available
    pub fn has_next(&self) -> bool {
        self.current_index < self.poses.len()
    }

    /// Get current progress percentage
    pub fn progress(&self) -> f64 {
        if self.poses.is_empty() {
            return 100.0;
        }
        (self.current_index as f64 / self.poses.len() as f64) * 100.0
    }
}

#[async_trait::async_trait]
impl SlamBridge for ParkDatasetBridge {
    async fn next_packet(&mut self) -> anyhow::Result<MeasureGroup> {
        if !self.has_next() {
            return Err(anyhow::anyhow!("No more frames in dataset"));
        }

        let idx = self.current_index;
        
        // Load PCD file
        let pcd_path = self.dataset_path.join("pcd").join(format!("{}.pcd", idx));
        let mut pointcloud = Self::load_pcd(&pcd_path)?;
        
        // Calculate timestamp
        let frame_time = self.start_time + (idx as f64 * self.frame_interval);
        pointcloud.timestamp = frame_time;
        
        // Update point timestamps (spread across frame interval)
        if !pointcloud.points.is_empty() {
            let dt = self.frame_interval / pointcloud.points.len() as f64;
            for (i, point) in pointcloud.points.iter_mut().enumerate() {
                point.timestamp = frame_time + (i as f64 * dt);
            }
        }

        let imu_data: Vec<ImuData> = Vec::new();
        
        // Create MeasureGroup
        // Note: Park dataset doesn't include IMU data, so we provide empty IMU vector
        let measure_group = MeasureGroup {
            lidar_begin_time: frame_time,
            lidar_end_time: frame_time + self.frame_interval,
            points: vec![pointcloud],
            imus: imu_data, // Provide interpolated IMU data
        };
        
        self.current_index += 1;
        
        tracing::info!(
            "Loaded frame {}/{} ({:.1}%)", 
            idx, 
            self.poses.len(), 
            self.progress()
        );
        
        Ok(measure_group)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_load_park_dataset() {
        // This test requires the actual dataset to be present
        let dataset_path = "reference/dataset/park";
        if !Path::new(dataset_path).exists() {
            println!("Park dataset not found, skipping test");
            return;
        }

        let mut bridge = ParkDatasetBridge::new(dataset_path, 10.0).unwrap();
        assert!(bridge.has_next());
        
        let group = bridge.next_packet().await.unwrap();
        assert!(!group.points.is_empty());
        assert!(group.lidar_begin_time >= 0.0);
        
        println!("First frame loaded: {} points", group.points[0].points.len());
    }

    #[test]
    fn test_load_poses() {
        let dataset_path = "reference/dataset/park";
        if !Path::new(dataset_path).exists() {
            println!("Park dataset not found, skipping test");
            return;
        }

        let poses = ParkDatasetBridge::load_poses(dataset_path).unwrap();
        assert!(!poses.is_empty());
        println!("Loaded {} poses", poses.len());
        
        // Check first pose
        println!("First pose: translation={:?}, quaternion={:?}", 
            poses[0].translation, poses[0].quaternion);
    }
}
