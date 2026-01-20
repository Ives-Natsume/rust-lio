//! For Livox Mid360
//! 
//! Receives data from bridges implementing `SlamBridge` trait
use std::io::{Cursor, Read};
use tokio::net::UdpSocket;
use byteorder::{LittleEndian, ReadBytesExt};
use sophus::nalgebra::Vector3;
use std::sync::{Arc, atomic::{AtomicU64, Ordering}};
use std::time::{SystemTime, UNIX_EPOCH};
use std::thread;
use crate::utils::structs::*;
use crate::config::CONFIG;

#[async_trait::async_trait]
pub trait SlamBridge: Send + Sync {
    async fn next_packet(&mut self) -> anyhow::Result<MeasureGroup>;
}

enum RawPacket {
    Lidar(PointCloudXYZI),
    Imu(ImuData),
}

pub struct UdpBridge {
    lidar_socket: UdpSocket,
    imu_socket: UdpSocket,
    lidar_recv_buffer: [u8; 65535], // Max UDP size
    imu_recv_buffer: [u8; 65535],   // Max UDP size
    frame_time: u64,                // in milliseconds
    imu_buffer: Vec<ImuData>,
    lidar_buffer: Vec<PointCloudXYZI>,
    last_update_time: Arc<AtomicU64>,
}

impl UdpBridge {
    pub async fn new(lidar_bind_addr: &str, imu_bind_addr: &str, frame_time: &u64) -> anyhow::Result<Self> {
        let lidar_socket = UdpSocket::bind(lidar_bind_addr).await?;
        let imu_socket = UdpSocket::bind(imu_bind_addr).await?;
        
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
        let last_update_time = Arc::new(AtomicU64::new(now));
        
        let monitor_time = last_update_time.clone();
        thread::spawn(move || {
            monitor_lidar_status(monitor_time);
        });

        Ok(UdpBridge {
            lidar_socket,
            imu_socket,
            lidar_recv_buffer: [0u8; 65535],
            imu_recv_buffer: [0u8; 65535],
            frame_time: frame_time.clone(),
            imu_buffer: Vec::new(),
            lidar_buffer: Vec::new(),
            last_update_time,
        })
    }
}

#[async_trait::async_trait]
impl SlamBridge for UdpBridge {
    async fn next_packet(&mut self) -> anyhow::Result<MeasureGroup> {
        loop {
            let mut packet_type = None;
            
            tokio::select! {
                res = self.lidar_socket.recv_from(&mut self.lidar_recv_buffer) => {
                    let (len, _) = res?;
                    match rawdata_decoder(&self.lidar_recv_buffer[..len]) {
                        Ok(RawPacket::Lidar(cloud)) => {
                            self.lidar_buffer.push(cloud);
                            
                            let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
                            self.last_update_time.store(now, Ordering::Relaxed);
                            
                            packet_type = Some("lidar");
                        },
                        Ok(_) => {},
                        Err(e) => tracing::warn!("Lidar decode error: {}", e),
                    }
                }
                res = self.imu_socket.recv_from(&mut self.imu_recv_buffer) => {
                    let (len, _) = res?;
                    match rawdata_decoder(&self.imu_recv_buffer[..len]) {
                        Ok(RawPacket::Imu(imu)) => {
                            self.imu_buffer.push(imu);
                        },
                        Ok(_) => {},
                        Err(e) => tracing::warn!("IMU decode error: {}", e),
                    }
                }
            }

            // MeasureGroup package
            if let Some("lidar") = packet_type {
                if let (Some(first), Some(last)) = (self.lidar_buffer.first(), self.lidar_buffer.last()) {
                    let duration = last.timestamp - first.timestamp;
                    if duration >= self.frame_time as f64 / 1000.0 {
                        // Construct MeasureGroup
                        let lidar_begin_time = first.timestamp;
                        let lidar_end_time = last.timestamp;
                        
                        let mut pcl_packet_queue = Vec::new();
                        for cloud in self.lidar_buffer.drain(..) {
                            pcl_packet_queue.push(cloud);
                        }

                        // Extract relevant IMU data
                        // We need IMU data covering [begin_time, end_time]
                        // Ideally slightly more for interpolation
                        let mut frame_imus = Vec::new();
                        let mut remaining_imus = Vec::new();
                        
                        for imu in self.imu_buffer.drain(..) {
                            if imu.timestamp < lidar_begin_time - 0.01 {
                                // Too old, discard
                                continue;
                            } else if imu.timestamp <= lidar_end_time + 0.01 {
                                frame_imus.push(imu);
                            } else {
                                // Future data, keep for next frame
                                remaining_imus.push(imu);
                            }
                        }
                        self.imu_buffer = remaining_imus;
                        // TODO: Consider adding boundary IMU data for better interpolation

                        return Ok(MeasureGroup {
                            lidar_begin_time,
                            lidar_end_time,
                            points: pcl_packet_queue,
                            imus: frame_imus,
                        });
                    }
                }
            }
        }
    }
}

/// Decode raw lidar UDP data into RawPacket
/// 
/// Doc: [Livox Mid360 UDP Protocol](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html#protocol-format)
fn rawdata_decoder(data: &[u8]) -> anyhow::Result<RawPacket> {
    const HEADER_SIZE: usize = 36;
    const POINT_SIZE: usize = 14;

    if data.len() < HEADER_SIZE {
        return Err(anyhow::anyhow!("Data too short for header"));
    }

    let mut cursor = Cursor::new(data);

    let _version = cursor.read_u8()?;                            // 0: 协议版本
    let length = cursor.read_u16::<LittleEndian>()?;            // 1-2: UDP 包长度
    let _time_interval = cursor.read_u16::<LittleEndian>()?;    // 3-4: 时间间隔
    let dot_num = cursor.read_u16::<LittleEndian>()?;           // 5-6: data包含点云数量
    let _udp_cnt = cursor.read_u16::<LittleEndian>()?;          // 7-8: UDP包计数
    let _frame_cnt = cursor.read_u8()?;                          // 9: 帧计数
    let data_type = cursor.read_u8()?;                           // 10: 数据类型
    let _time_type = cursor.read_u8()?;                          // 11: 时间戳类型

    // 12-23: 保留字段 - 读取12字节
    let mut reserved = vec![0u8; 12];
    cursor.read_exact(&mut reserved)?;

    let _crc32 = cursor.read_u32::<LittleEndian>()?;            // 24-27: CRC32校验码
    let timestamp = cursor.read_u64::<LittleEndian>()?;         // 28-35: 时间戳
    let timestamp_sec = timestamp as f64 * 1e-9;

    let length_usize = length as usize;
    if data.len() < length_usize {
        return Err(anyhow::anyhow!("Data too short for payload"));
    }

    let payload = &data[HEADER_SIZE..length_usize];
    let mut payload_cursor = Cursor::new(payload);

    match data_type {
        0x01 => {
            // Lidar Data
            if payload.len() != dot_num as usize * POINT_SIZE {
                return Err(anyhow::anyhow!("Payload size does not match point count"));
            }
            
            let mut points = Vec::with_capacity(dot_num as usize);
            let min_boundary = 0.1f32;
            let max_boundary = CONFIG.get().unwrap().lidar.max_boundary as f32;

            // let mut temp_counter = 0;
            // let mut x_farest_dist = 0.0f32;

            for _ in 0..dot_num {
                let x = payload_cursor.read_i32::<LittleEndian>()? as f32 * 0.001;
                let y = payload_cursor.read_i32::<LittleEndian>()? as f32 * 0.001;
                let z = payload_cursor.read_i32::<LittleEndian>()? as f32 * 0.001;
                let intensity = payload_cursor.read_u8()? as f32;
                let _tag = payload_cursor.read_u8()?;

                if x.abs() < min_boundary && y.abs() < min_boundary && z.abs() < min_boundary {
                    // temp_counter += 1;
                    continue;
                }

                if x.abs() > max_boundary || y.abs() > max_boundary || z.abs() > max_boundary {
                    // temp_counter += 1;
                    // if x.abs() > x_farest_dist {
                    //     x_farest_dist = x.abs();
                    // }
                    continue;
                }

                points.push(PointXYZI {
                    pos: Vector3::new(x, y, z),
                    intensity,
                    timestamp: timestamp_sec,
                });
            }
            
            let pointcloud = PointCloudXYZI::from_points(points, timestamp_sec);

            // tracing::debug!("Filtered out {} points outside boundary, max x distance: {}", temp_counter, x_farest_dist);
            
            Ok(RawPacket::Lidar(pointcloud))
        },
        0x00 => {
            // IMU Data
            if payload.len() < 24 {
                 return Err(anyhow::anyhow!("Payload too short for IMU data"));
            }

            let gyr_x = payload_cursor.read_f32::<LittleEndian>()?;
            let gyr_y = payload_cursor.read_f32::<LittleEndian>()?;
            let gyr_z = payload_cursor.read_f32::<LittleEndian>()?;
            let acc_x = payload_cursor.read_f32::<LittleEndian>()?;
            let acc_y = payload_cursor.read_f32::<LittleEndian>()?;
            let acc_z = payload_cursor.read_f32::<LittleEndian>()?;

            Ok(RawPacket::Imu(ImuData {
                timestamp: timestamp_sec,
                acc: Vector3::new(acc_x as f64, acc_y as f64, acc_z as f64),
                gyr: Vector3::new(gyr_x as f64, gyr_y as f64, gyr_z as f64),
            }))
        },
        _ => {
            Err(anyhow::anyhow!("Unknown or not supported data type: {}", data_type))
        }
    }
}

fn monitor_lidar_status(last_update_time: Arc<AtomicU64>) {
    loop {
        thread::sleep(std::time::Duration::from_secs(1));
        let last = last_update_time.load(Ordering::Relaxed);
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();

        if now > last + 1 {
            tracing::warn!("Lidar stream cut off! No data for {} seconds.", now - last);
        }
    }
}
