// LVX2 File Format Reader
// Based on official Livox SDK2 format specification
// Reference: docs/LVX2解码方法文档.md

use std::fs::File;
use std::io::{self, BufReader, Read, Seek, SeekFrom};
use std::path::Path;
use byteorder::{LittleEndian, ReadBytesExt};

// Magic constants
const LVX2_MAGIC: &[u8; 16] = b"livox_tech\0\0\0\0\0\0";
const LVX2_MAGIC_CODE: u32 = 0xAC0EA767;
const LVX2_VERSION_MAJOR: u8 = 2;

// Size constants
const PUBLIC_HEADER_SIZE: usize = 24;
const PRIVATE_HEADER_SIZE: usize = 5;
const DEVICE_INFO_SIZE: usize = 63;
const FRAME_HEADER_SIZE: usize = 24;
const PACKAGE_HEADER_SIZE: usize = 27;

/// LVX2 Public Header (24 bytes)
#[derive(Debug, Clone)]
pub struct Lvx2PublicHeader {
    pub signature: [u8; 16],
    pub ver_a: u8,
    pub ver_b: u8,
    pub ver_c: u8,
    pub ver_d: u8,
    pub magic_code: u32,
}

/// LVX2 Private Header (5 bytes)
#[derive(Debug, Clone)]
pub struct Lvx2PrivateHeader {
    pub duration: u32,        // Frame duration in milliseconds (typically 50ms)
    pub device_count: u8,     // Number of devices
}

/// Device Information (63 bytes)
#[derive(Debug, Clone)]
pub struct Lvx2DeviceInfo {
    pub lidar_sn: String,           // 16 bytes
    pub hub_sn: String,             // 16 bytes
    pub lidar_id: u32,              // 4 bytes (from IP address)
    pub lidar_type: u8,             // 1 byte (247 = Mid-360)
    pub device_type: u8,            // 1 byte (9)
    pub enable_extrinsic: bool,     // 1 byte
    pub offset_roll: f32,           // 4 bytes (degrees)
    pub offset_pitch: f32,          // 4 bytes (degrees)
    pub offset_yaw: f32,            // 4 bytes (degrees)
    pub offset_x: f32,              // 4 bytes (meters)
    pub offset_y: f32,              // 4 bytes (meters)
    pub offset_z: f32,              // 4 bytes (meters)
}

/// Frame Header (24 bytes)
#[derive(Debug, Clone)]
pub struct Lvx2FrameHeader {
    pub current_offset: u64,    // Current frame position in file
    pub next_offset: u64,       // Next frame position
    pub frame_index: u64,       // Frame index (0, 1, 2, ...)
}

/// LVX2 Package Header (27 bytes)
#[derive(Debug, Clone)]
pub struct Lvx2PackageHeader {
    pub version: u8,            // Protocol version
    pub lidar_id: u32,          // LiDAR ID
    pub lidar_type: u8,         // LiDAR type in package (typically 8)
    pub timestamp_type: u8,     // 0=no sync, 1=PTP, 2=GPS
    pub timestamp: u64,         // Timestamp in nanoseconds
    pub udp_count: u16,         // UDP packet counter
    pub data_type: u8,          // 0=IMU, 1/2/3=Point Cloud
    pub length: u32,            // Data length
    pub frame_count: u8,        // Frame counter
    pub reserved: [u8; 4],      // Reserved (all zeros)
}

/// LVX2 Package containing header and data
#[derive(Debug, Clone)]
pub struct Lvx2Package {
    pub header: Lvx2PackageHeader,
    pub data: Vec<u8>,
}

/// IMU Data (24 bytes = 6 floats)
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub gyro_x: f32,    // rad/s
    pub gyro_y: f32,    // rad/s
    pub gyro_z: f32,    // rad/s
    pub acc_x: f32,     // g
    pub acc_y: f32,     // g
    pub acc_z: f32,     // g
}

/// Point Cloud Data Type 1 (14 bytes per point - Cartesian 32-bit)
#[derive(Debug, Clone, Copy)]
pub struct PointType1 {
    pub x: i32,             // mm
    pub y: i32,             // mm
    pub z: i32,             // mm
    pub reflectivity: u8,
    pub tag: u8,
}

/// Main LVX2 Reader
pub struct Lvx2Reader {
    reader: BufReader<File>,
    pub public_header: Lvx2PublicHeader,
    pub private_header: Lvx2PrivateHeader,
    pub devices: Vec<Lvx2DeviceInfo>,
    current_frame_offset: u64,
    file_size: u64,
}

impl Lvx2Reader {
    /// Open and parse LVX2 file
    pub fn open<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let file = File::open(&path)?;
        let file_size = file.metadata()?.len();
        let mut reader = BufReader::new(file);

        // Read public header (24 bytes)
        let public_header = Self::read_public_header(&mut reader)?;
        
        // Read private header (5 bytes)
        let private_header = Self::read_private_header(&mut reader)?;
        
        // Read device information (63 bytes * device_count)
        let devices = Self::read_device_info(&mut reader, private_header.device_count)?;
        
        // Current position should be at first frame
        let current_frame_offset = reader.stream_position()?;

        Ok(Lvx2Reader {
            reader,
            public_header,
            private_header,
            devices,
            current_frame_offset,
            file_size,
        })
    }

    /// Read and validate public header (24 bytes)
    fn read_public_header<R: Read>(reader: &mut R) -> io::Result<Lvx2PublicHeader> {
        let mut signature = [0u8; 16];
        reader.read_exact(&mut signature)?;

        if &signature != LVX2_MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Invalid LVX2 signature: expected 'livox_tech', got {:?}", 
                    String::from_utf8_lossy(&signature))
            ));
        }

        let ver_a = reader.read_u8()?;
        let ver_b = reader.read_u8()?;
        let ver_c = reader.read_u8()?;
        let ver_d = reader.read_u8()?;
        
        if ver_a != LVX2_VERSION_MAJOR {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Unsupported LVX2 version: {}.{}.{}.{}", ver_a, ver_b, ver_c, ver_d)
            ));
        }

        let magic_code = reader.read_u32::<LittleEndian>()?;
        if magic_code != LVX2_MAGIC_CODE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Invalid magic code: expected 0x{:X}, got 0x{:X}", 
                    LVX2_MAGIC_CODE, magic_code)
            ));
        }

        Ok(Lvx2PublicHeader {
            signature,
            ver_a,
            ver_b,
            ver_c,
            ver_d,
            magic_code,
        })
    }

    /// Read private header (5 bytes)
    fn read_private_header<R: Read>(reader: &mut R) -> io::Result<Lvx2PrivateHeader> {
        let duration = reader.read_u32::<LittleEndian>()?;
        let device_count = reader.read_u8()?;

        Ok(Lvx2PrivateHeader {
            duration,
            device_count,
        })
    }

    /// Read device information (63 bytes * device_count)
    fn read_device_info<R: Read>(reader: &mut R, device_count: u8) -> io::Result<Vec<Lvx2DeviceInfo>> {
        let mut devices = Vec::with_capacity(device_count as usize);

        for _ in 0..device_count {
            // Read lidar_sn (16 bytes)
            let mut lidar_sn_bytes = [0u8; 16];
            reader.read_exact(&mut lidar_sn_bytes)?;
            let lidar_sn = String::from_utf8_lossy(&lidar_sn_bytes)
                .trim_end_matches('\0')
                .to_string();

            // Read hub_sn (16 bytes)
            let mut hub_sn_bytes = [0u8; 16];
            reader.read_exact(&mut hub_sn_bytes)?;
            let hub_sn = String::from_utf8_lossy(&hub_sn_bytes)
                .trim_end_matches('\0')
                .to_string();

            let lidar_id = reader.read_u32::<LittleEndian>()?;
            let lidar_type = reader.read_u8()?;
            let device_type = reader.read_u8()?;
            let enable_extrinsic = reader.read_u8()? != 0;
            let offset_roll = reader.read_f32::<LittleEndian>()?;
            let offset_pitch = reader.read_f32::<LittleEndian>()?;
            let offset_yaw = reader.read_f32::<LittleEndian>()?;
            let offset_x = reader.read_f32::<LittleEndian>()?;
            let offset_y = reader.read_f32::<LittleEndian>()?;
            let offset_z = reader.read_f32::<LittleEndian>()?;

            devices.push(Lvx2DeviceInfo {
                lidar_sn,
                hub_sn,
                lidar_id,
                lidar_type,
                device_type,
                enable_extrinsic,
                offset_roll,
                offset_pitch,
                offset_yaw,
                offset_x,
                offset_y,
                offset_z,
            });
        }

        Ok(devices)
    }

    /// Read next frame from file
    pub fn read_next_frame(&mut self) -> io::Result<Option<Vec<Lvx2Package>>> {
        // Check if we're at EOF
        if self.current_frame_offset >= self.file_size {
            return Ok(None);
        }

        // Seek to current frame position
        self.reader.seek(SeekFrom::Start(self.current_frame_offset))?;

        // Read frame header (24 bytes)
        let frame_header = Self::read_frame_header(&mut self.reader)?;

        // Check for end of file (last frame has next_offset == current_offset)
        if frame_header.next_offset == frame_header.current_offset {
            return Ok(None);
        }

        // Calculate frame data size
        let frame_data_size = (frame_header.next_offset - frame_header.current_offset - FRAME_HEADER_SIZE as u64) as usize;

        // Read all packages in this frame
        let mut packages = Vec::new();
        let mut bytes_read = 0usize;

        while bytes_read < frame_data_size {
            match Self::read_package(&mut self.reader) {
                Ok(package) => {
                    bytes_read += PACKAGE_HEADER_SIZE + package.data.len();
                    packages.push(package);
                }
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
                Err(e) => return Err(e),
            }
        }

        // Update position for next frame
        self.current_frame_offset = frame_header.next_offset;

        Ok(Some(packages))
    }

    /// Read frame header (24 bytes)
    fn read_frame_header<R: Read>(reader: &mut R) -> io::Result<Lvx2FrameHeader> {
        let current_offset = reader.read_u64::<LittleEndian>()?;
        let next_offset = reader.read_u64::<LittleEndian>()?;
        let frame_index = reader.read_u64::<LittleEndian>()?;

        Ok(Lvx2FrameHeader {
            current_offset,
            next_offset,
            frame_index,
        })
    }

    /// Read package header and data
    fn read_package<R: Read>(reader: &mut R) -> io::Result<Lvx2Package> {
        // Read package header (27 bytes)
        let version = reader.read_u8()?;
        let lidar_id = reader.read_u32::<LittleEndian>()?;
        let lidar_type = reader.read_u8()?;
        let timestamp_type = reader.read_u8()?;
        let timestamp = reader.read_u64::<LittleEndian>()?;
        let udp_count = reader.read_u16::<LittleEndian>()?;
        let data_type = reader.read_u8()?;
        let length = reader.read_u32::<LittleEndian>()?;
        let frame_count = reader.read_u8()?;
        
        let mut reserved = [0u8; 4];
        reader.read_exact(&mut reserved)?;

        let header = Lvx2PackageHeader {
            version,
            lidar_id,
            lidar_type,
            timestamp_type,
            timestamp,
            udp_count,
            data_type,
            length,
            frame_count,
            reserved,
        };

        // Read data segment
        let mut data = vec![0u8; length as usize];
        reader.read_exact(&mut data)?;

        Ok(Lvx2Package { header, data })
    }

    /// Parse IMU data from package (data_type == 0)
    pub fn parse_imu_data(package: &Lvx2Package) -> io::Result<ImuData> {
        if package.header.data_type != 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Package is not IMU data (data_type != 0)"
            ));
        }

        if package.data.len() != 24 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Invalid IMU data length: expected 24, got {}", package.data.len())
            ));
        }

        let mut cursor = std::io::Cursor::new(&package.data);
        
        Ok(ImuData {
            gyro_x: cursor.read_f32::<LittleEndian>()?,
            gyro_y: cursor.read_f32::<LittleEndian>()?,
            gyro_z: cursor.read_f32::<LittleEndian>()?,
            acc_x: cursor.read_f32::<LittleEndian>()?,
            acc_y: cursor.read_f32::<LittleEndian>()?,
            acc_z: cursor.read_f32::<LittleEndian>()?,
        })
    }

    /// Parse point cloud data type 1 (data_type == 1, 14 bytes per point)
    pub fn parse_point_cloud_type1(package: &Lvx2Package) -> io::Result<Vec<PointType1>> {
        if package.header.data_type != 1 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Package is not point cloud type 1 (data_type != 1)"
            ));
        }

        if package.data.len() % 14 != 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Invalid point cloud data length: {} is not multiple of 14", package.data.len())
            ));
        }

        let point_count = package.data.len() / 14;
        let mut points = Vec::with_capacity(point_count);
        let mut cursor = std::io::Cursor::new(&package.data);

        for _ in 0..point_count {
            points.push(PointType1 {
                x: cursor.read_i32::<LittleEndian>()?,
                y: cursor.read_i32::<LittleEndian>()?,
                z: cursor.read_i32::<LittleEndian>()?,
                reflectivity: cursor.read_u8()?,
                tag: cursor.read_u8()?,
            });
        }

        Ok(points)
    }

    /// Get total header size
    pub fn get_header_size() -> usize {
        PUBLIC_HEADER_SIZE + PRIVATE_HEADER_SIZE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_open_lvx2_file() {
        let path = "reference/dataset/Indoor_sampledata.lvx2";
        if !std::path::Path::new(path).exists() {
            println!("Test file not found: {}", path);
            return;
        }

        match Lvx2Reader::open(path) {
            Ok(reader) => {
                println!("\n=== LVX2 File Successfully Opened ===");
                println!("Public Header:");
                println!("  Version: {}.{}.{}.{}", 
                    reader.public_header.ver_a,
                    reader.public_header.ver_b,
                    reader.public_header.ver_c,
                    reader.public_header.ver_d);
                println!("  Magic Code: 0x{:X}", reader.public_header.magic_code);
                
                println!("\nPrivate Header:");
                println!("  Duration: {}ms", reader.private_header.duration);
                println!("  Device Count: {}", reader.private_header.device_count);
                
                println!("\nDevices:");
                for (i, device) in reader.devices.iter().enumerate() {
                    println!("  Device {}:", i);
                    println!("    LiDAR SN: {}", device.lidar_sn);
                    println!("    LiDAR Type: {}", device.lidar_type);
                    println!("    Device Type: {}", device.device_type);
                }
                
                println!("\nFirst frame offset: 0x{:X}", reader.current_frame_offset);
                println!("Expected: 0x{:X} (24+5+63*{})", 
                    PUBLIC_HEADER_SIZE + PRIVATE_HEADER_SIZE + DEVICE_INFO_SIZE * reader.private_header.device_count as usize,
                    reader.private_header.device_count);
            }
            Err(e) => {
                println!("Failed to open LVX2: {}", e);
                panic!("Test failed");
            }
        }
    }

    #[test]
    fn test_read_frames() {
        let path = "reference/dataset/Indoor_sampledata.lvx2";
        if !std::path::Path::new(path).exists() {
            println!("Test file not found: {}", path);
            return;
        }

        let mut reader = Lvx2Reader::open(path).expect("Failed to open file");
        
        println!("\n=== Reading Frames ===");
        let mut frame_count = 0;
        let mut imu_count = 0;
        let mut point_count = 0;

        while let Ok(Some(packages)) = reader.read_next_frame() {
            frame_count += 1;
            
            for package in &packages {
                if package.header.data_type == 0 {
                    // IMU data
                    imu_count += 1;
                    if imu_count <= 3 {
                        if let Ok(imu) = Lvx2Reader::parse_imu_data(package) {
                            println!("Frame {}, IMU: gyro=({:.3}, {:.3}, {:.3}), acc=({:.3}, {:.3}, {:.3}), ts={}ns",
                                frame_count, imu.gyro_x, imu.gyro_y, imu.gyro_z,
                                imu.acc_x, imu.acc_y, imu.acc_z, package.header.timestamp);
                        }
                    }
                } else if package.header.data_type == 1 {
                    // Point cloud type 1
                    point_count += 1;
                    if point_count <= 3 {
                        if let Ok(points) = Lvx2Reader::parse_point_cloud_type1(package) {
                            println!("Frame {}, PointCloud: {} points, first point=({}, {}, {}), ts={}ns",
                                frame_count, points.len(), 
                                points[0].x, points[0].y, points[0].z,
                                package.header.timestamp);
                        }
                    }
                }
            }

            // if frame_count >= 10 {
            //     break;
            // }
        }

        println!("\n=== Summary ===");
        println!("Processed {} frames", frame_count);
        println!("Found {} IMU packages", imu_count);
        println!("Found {} point cloud packages", point_count);
    }
}
