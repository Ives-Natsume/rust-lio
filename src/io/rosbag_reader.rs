// ROS Bag File Reader for Livox MID360 Dataset
// Extracts IMU and LiDAR point cloud data from ROS bag files

use std::path::Path;
use anyhow::{Result, Context};
use rosbag::{RosBag, ChunkRecord};
use std::io::{Cursor, Read};
use byteorder::{LittleEndian, ReadBytesExt};
use sophus::nalgebra::Vector3;
use crate::utils::structs::{ImuData, PointCloudXYZI, PointXYZI};

/// Parse ROS string from binary data (4-byte length + string data)
fn parse_ros_string<R: std::io::Read>(reader: &mut R) -> Result<String> {
    let length = reader.read_u32::<LittleEndian>()?;
    let mut string_data = vec![0u8; length as usize];
    reader.read_exact(&mut string_data)?;
    Ok(String::from_utf8_lossy(&string_data).trim_end_matches('\0').to_string())
}

/// Parse sensor_msgs/Imu message from ROS1 binary format
/// 
/// Message structure:
/// - Header (seq, stamp, frame_id)
/// - Orientation (quaternion: x,y,z,w - 4 doubles)
/// - Orientation covariance (9 doubles)
/// - Angular velocity (x,y,z - 3 doubles)
/// - Angular velocity covariance (9 doubles)
/// - Linear acceleration (x,y,z - 3 doubles)
/// - Linear acceleration covariance (9 doubles)
pub fn parse_imu_message(data: &[u8]) -> Result<ImuData> {
    let mut cursor = Cursor::new(data);
    
    // Header
    let _seq = cursor.read_u32::<LittleEndian>()?;
    let stamp_sec = cursor.read_u32::<LittleEndian>()?;
    let stamp_nsec = cursor.read_u32::<LittleEndian>()?;
    let _frame_id = parse_ros_string(&mut cursor)?;
    
    // Orientation (quaternion) - skip (4 doubles = 32 bytes)
    cursor.set_position(cursor.position() + 32);
    
    // Orientation covariance - skip (9 doubles = 72 bytes)
    cursor.set_position(cursor.position() + 72);
    
    // Angular velocity (gyroscope)
    let gyr_x = cursor.read_f64::<LittleEndian>()?;
    let gyr_y = cursor.read_f64::<LittleEndian>()?;
    let gyr_z = cursor.read_f64::<LittleEndian>()?;
    
    // Angular velocity covariance - skip (9 doubles = 72 bytes)
    cursor.set_position(cursor.position() + 72);
    
    // Linear acceleration (accelerometer)
    let acc_x = cursor.read_f64::<LittleEndian>()?;
    let acc_y = cursor.read_f64::<LittleEndian>()?;
    let acc_z = cursor.read_f64::<LittleEndian>()?;
    
    // Convert timestamp to seconds
    let timestamp = stamp_sec as f64 + stamp_nsec as f64 / 1_000_000_000.0;
    
    Ok(ImuData {
        timestamp,
        gyr: Vector3::new(gyr_x, gyr_y, gyr_z),
        acc: Vector3::new(acc_x, acc_y, acc_z),
    })
}

/// PointCloud2 field information
#[derive(Debug, Clone)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

/// Parse sensor_msgs/PointCloud2 message from ROS1 binary format
/// 
/// Message structure:
/// - Header (seq, stamp, frame_id)
/// - height (uint32)
/// - width (uint32)
/// - fields[] (array of PointField)
/// - is_bigendian (bool/uint8)
/// - point_step (uint32)
/// - row_step (uint32)
/// - data[] (uint8 array)
/// - is_dense (bool/uint8)
pub fn parse_pointcloud2_message(data: &[u8]) -> Result<PointCloudXYZI> {
    let mut cursor = Cursor::new(data);
    
    // Header
    let _seq = cursor.read_u32::<LittleEndian>()?;
    let stamp_sec = cursor.read_u32::<LittleEndian>()?;
    let stamp_nsec = cursor.read_u32::<LittleEndian>()?;
    let _frame_id = parse_ros_string(&mut cursor)?;
    
    // PointCloud2 fields
    let height = cursor.read_u32::<LittleEndian>()?;
    let width = cursor.read_u32::<LittleEndian>()?;
    
    // Fields array
    let num_fields = cursor.read_u32::<LittleEndian>()?;
    let mut fields = Vec::with_capacity(num_fields as usize);
    
    for _ in 0..num_fields {
        let field_name = parse_ros_string(&mut cursor)?;
        let field_offset = cursor.read_u32::<LittleEndian>()?;
        let datatype = cursor.read_u8()?;
        let count = cursor.read_u32::<LittleEndian>()?;
        
        fields.push(PointField {
            name: field_name,
            offset: field_offset,
            datatype,
            count,
        });
    }
    
    let _is_bigendian = cursor.read_u8()?;
    let point_step = cursor.read_u32::<LittleEndian>()?;
    let _row_step = cursor.read_u32::<LittleEndian>()?;
    
    // Data array
    let data_length = cursor.read_u32::<LittleEndian>()?;
    let mut point_data = vec![0u8; data_length as usize];
    cursor.read_exact(&mut point_data)?;
    
    let _is_dense = cursor.read_u8()?;
    
    // Convert timestamp to seconds
    let timestamp = stamp_sec as f64 + stamp_nsec as f64 / 1_000_000_000.0;
    
    // Parse points based on field layout
    let num_points = if point_step > 0 {
        data_length / point_step
    } else {
        0
    };
    
    let mut points = Vec::with_capacity(num_points as usize);
    
    // Find field offsets for x, y, z, intensity
    let x_offset = fields.iter().find(|f| f.name == "x").map(|f| f.offset).unwrap_or(0);
    let y_offset = fields.iter().find(|f| f.name == "y").map(|f| f.offset).unwrap_or(4);
    let z_offset = fields.iter().find(|f| f.name == "z").map(|f| f.offset).unwrap_or(8);
    let intensity_offset = fields.iter()
        .find(|f| f.name == "intensity" || f.name == "i")
        .map(|f| f.offset)
        .unwrap_or(12);
    
    // Parse each point
    for i in 0..num_points {
        let point_offset = (i * point_step) as usize;
        
        if point_offset + point_step as usize > point_data.len() {
            break;
        }
        
        let mut point_cursor = Cursor::new(&point_data[point_offset..]);
        
        // Read x, y, z (typically float32)
        point_cursor.set_position(x_offset as u64);
        let x = point_cursor.read_f32::<LittleEndian>()?;
        
        point_cursor.set_position(y_offset as u64);
        let y = point_cursor.read_f32::<LittleEndian>()?;
        
        point_cursor.set_position(z_offset as u64);
        let z = point_cursor.read_f32::<LittleEndian>()?;
        
        // Read intensity (could be float32 or uint8)
        point_cursor.set_position(intensity_offset as u64);
        let intensity = if let Some(intensity_field) = fields.iter().find(|f| f.name == "intensity" || f.name == "i") {
            match intensity_field.datatype {
                7 => point_cursor.read_f32::<LittleEndian>()? as f64, // float32
                2 => point_cursor.read_u8()? as f64,                  // uint8
                _ => 0.0,
            }
        } else {
            0.0
        };
        
        points.push(PointXYZI {
            pos: Vector3::new(x, y, z),
            intensity: intensity as f32,
            timestamp,
        });
    }
    
    Ok(PointCloudXYZI {
        timestamp,
        width,
        height,
        points,
        is_dense: true,
    })
}

/// ROS bag reader for Livox datasets
pub struct RosbagReader {
    bag: RosBag,
    imu_topic: String,
    lidar_topic: String,
}

impl RosbagReader {
    /// Open a ROS bag file
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let bag = RosBag::new(path.as_ref())
            .with_context(|| format!("Failed to open ROS bag: {:?}", path.as_ref()))?;
        
        println!("=== ROS Bag Successfully Opened ===");
        println!("Path: {:?}", path.as_ref());
        
        Ok(RosbagReader {
            bag,
            imu_topic: "/eve/livox/imu".to_string(),
            lidar_topic: "/eve/lidar3d".to_string(),
        })
    }
    
    /// Set IMU topic name
    pub fn set_imu_topic(&mut self, topic: &str) -> &mut Self {
        self.imu_topic = topic.to_string();
        self
    }
    
    /// Set LiDAR topic name  
    pub fn set_lidar_topic(&mut self, topic: &str) -> &mut Self {
        self.lidar_topic = topic.to_string();
        self
    }
    
    /// Read IMU and PointCloud messages from bag (limited count for testing)
    pub fn read_messages(&self, max_count: Option<usize>) -> Result<Vec<RosbagMessage>> {
        let mut messages = Vec::new();
        let mut imu_count = 0;
        let mut pc_count = 0;
        let mut total_processed = 0;
        
        let limit = max_count.unwrap_or(usize::MAX);
        
        for record_result in self.bag.chunk_records() {
            if let Ok(ChunkRecord::Chunk(chunk)) = record_result {
                for msg_result in chunk.messages() {
                    if let Ok(msg) = msg_result {
                        // MessageRecord should have connection() method to get topic info
                        // and msg() method to get data
                        // Let's try different approaches based on the API
                        
                        // Approach: iterate through connections to match topics
                        for conn in self.bag.connections().iter() {
                            if conn.topic == self.imu_topic {
                                // Try parsing as IMU
                                if let Ok(data) = msg.msg() {
                                    match parse_imu_message(data) {
                                        Ok(imu) => {
                                            imu_count += 1;
                                            messages.push(RosbagMessage::Imu(imu));
                                        }
                                        Err(e) => {
                                            if imu_count < 3 {
                                                eprintln!("Failed to parse IMU message {}: {}", imu_count, e);
                                            }
                                        }
                                    }
                                }
                            } else if conn.topic == self.lidar_topic {
                                // Try parsing as PointCloud
                                if let Ok(data) = msg.msg() {
                                    match parse_pointcloud2_message(data) {
                                        Ok(cloud) => {
                                            pc_count += 1;
                                            messages.push(RosbagMessage::PointCloud(cloud));
                                        }
                                        Err(e) => {
                                            if pc_count < 3 {
                                                eprintln!("Failed to parse PointCloud2 message {}: {}", pc_count, e);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        
                        total_processed += 1;
                        if total_processed >= limit {
                            break;
                        }
                    }
                }
                
                if total_processed >= limit {
                    break;
                }
            }
        }
        
        println!("Parsed {} IMU messages and {} PointCloud messages from {} total",
            imu_count, pc_count, total_processed);
        
        // Sort by timestamp
        messages.sort_by(|a, b| {
            let ta = match a {
                RosbagMessage::Imu(imu) => imu.timestamp,
                RosbagMessage::PointCloud(pc) => pc.timestamp,
            };
            let tb = match b {
                RosbagMessage::Imu(imu) => imu.timestamp,
                RosbagMessage::PointCloud(pc) => pc.timestamp,
            };
            ta.partial_cmp(&tb).unwrap_or(std::cmp::Ordering::Equal)
        });
        
        Ok(messages)
    }
}

/// ROS bag message types  
#[derive(Debug, Clone)]
pub enum RosbagMessage {
    Imu(ImuData),
    PointCloud(PointCloudXYZI),
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_parse_rosbag() {
        let path = "reference/dataset/livox_loop_2025-02-06-18-24-40.bag";
        if !std::path::Path::new(path).exists() {
            println!("Test file not found: {}", path);
            return;
        }
        
        match RosbagReader::open(path) {
            Ok(reader) => {
                println!("\n=== Testing ROS Bag Parser ===");
                
                match reader.read_messages(Some(1000)) {  // Only parse first 1000 messages for testing
                    Ok(messages) => {
                        println!("\n=== Parse Results ===");
                        println!("Total messages parsed: {}", messages.len());
                        
                        let imu_count = messages.iter().filter(|m| matches!(m, RosbagMessage::Imu(_))).count();
                        let pc_count = messages.iter().filter(|m| matches!(m, RosbagMessage::PointCloud(_))).count();
                        
                        println!("IMU messages: {}", imu_count);
                        println!("PointCloud messages: {}", pc_count);
                        
                        // Show first few samples
                        println!("\n=== First 5 IMU Samples ===");
                        for (i, msg) in messages.iter().filter(|m| matches!(m, RosbagMessage::Imu(_))).take(5).enumerate() {
                            if let RosbagMessage::Imu(imu) = msg {
                                println!("IMU {}: t={:.6}s, gyro=({:.4}, {:.4}, {:.4}), acc=({:.4}, {:.4}, {:.4})",
                                    i, imu.timestamp,
                                    imu.gyr.x, imu.gyr.y, imu.gyr.z,
                                    imu.acc.x, imu.acc.y, imu.acc.z);
                            }
                        }
                        
                        println!("\n=== First 3 PointCloud Samples ===");
                        for (i, msg) in messages.iter().filter(|m| matches!(m, RosbagMessage::PointCloud(_))).take(3).enumerate() {
                            if let RosbagMessage::PointCloud(pc) = msg {
                                println!("PointCloud {}: t={:.6}s, points={}, size={}x{}",
                                    i, pc.timestamp, pc.points.len(), pc.width, pc.height);
                                if !pc.points.is_empty() {
                                    let p = &pc.points[0];
                                    println!("  First point: ({:.3}, {:.3}, {:.3}), intensity={:.1}",
                                        p.pos.x, p.pos.y, p.pos.z, p.intensity);
                                }
                            }
                        }
                    }
                    Err(e) => {
                        println!("Failed to read messages: {}", e);
                        panic!("Test failed");
                    }
                }
            }
            Err(e) => {
                println!("Failed to open ROS bag: {}", e);
                panic!("Test failed");
            }
        }
    }
}
