use crate::core::state::SlamContext;
use serde::{Serialize, Deserialize};
use std::net::SocketAddr;
use tokio::net::UdpSocket;
use tokio::sync::broadcast;

// ============================================================================
// State Publishing
// ============================================================================

/// Publishable state snapshot containing pose and map statistics
/// 
/// This struct is serializable and can be sent over network or saved to file
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SlamStateSnapshot {
    /// Timestamp of the snapshot (seconds since first LiDAR scan)
    pub timestamp: f64,
    /// Position in world frame [x, y, z] (meters)
    pub position: [f64; 3],
    /// Rotation quaternion [w, x, y, z] (world frame)
    pub rotation_quat: [f64; 4],
    /// Rotation matrix (row-major, 3x3 flattened)
    pub rotation_matrix: [f64; 9],
    /// Velocity in world frame [vx, vy, vz] (m/s)
    pub velocity: [f64; 3],
    /// Gyroscope bias [bx, by, bz] (rad/s)
    pub gyro_bias: [f64; 3],
    /// Accelerometer bias [bx, by, bz] (m/s²)
    pub accel_bias: [f64; 3],
    /// Gravity vector estimate [gx, gy, gz] (m/s²)
    pub gravity: [f64; 3],
    /// LiDAR to IMU rotation quaternion [w, x, y, z]
    pub lidar_imu_rot_quat: [f64; 4],
    /// LiDAR to IMU translation [x, y, z] (meters)
    pub lidar_imu_trans: [f64; 3],
    /// Number of valid points in the map
    pub map_point_count: i32,
    /// Total tree size (including deleted points)
    pub map_tree_size: i32,
    /// Local map bounds [min_x, min_y, min_z, max_x, max_y, max_z]
    pub local_map_bounds: [f64; 6],
    /// Whether the map has been initialized
    pub map_initialized: bool,
}

/// Map point cloud data for publishing
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MapPointCloud {
    /// Number of points
    pub point_count: usize,
    /// Flattened point coordinates [x0, y0, z0, x1, y1, z1, ...]
    pub points: Vec<f64>,
}

/// Channel-based state publisher for in-process subscribers
pub struct StatePublisher {
    sender: broadcast::Sender<SlamStateSnapshot>,
}

impl StatePublisher {
    /// Create a new state publisher with specified channel capacity
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity);
        Self { sender }
    }

    /// Get a subscriber that receives state updates
    pub fn subscribe(&self) -> broadcast::Receiver<SlamStateSnapshot> {
        self.sender.subscribe()
    }

    /// Publish a state snapshot to all subscribers
    pub fn publish(&self, state: SlamStateSnapshot) -> Result<usize, broadcast::error::SendError<SlamStateSnapshot>> {
        self.sender.send(state)
    }
}

impl SlamContext {
    /// Get a snapshot of the current SLAM state
    /// 
    /// Returns a serializable struct containing pose, velocity, biases,
    /// and map statistics that can be published over network or saved.
    pub fn get_state_snapshot(&self) -> SlamStateSnapshot {
        let state = self.kf.get_x();
        let rot_matrix = state.rot.matrix();
        
        // Convert rotation matrix to quaternion
        // Using the standard matrix-to-quaternion conversion
        let quat = rotation_matrix_to_quaternion(&rot_matrix);
        let offset_quat = rotation_matrix_to_quaternion(&state.offset_r_l_i.matrix());
        
        SlamStateSnapshot {
            timestamp: 0.0, // Will be set by caller if needed
            position: [state.pos[0], state.pos[1], state.pos[2]],
            rotation_quat: quat,
            rotation_matrix: [
                rot_matrix[(0, 0)], rot_matrix[(0, 1)], rot_matrix[(0, 2)],
                rot_matrix[(1, 0)], rot_matrix[(1, 1)], rot_matrix[(1, 2)],
                rot_matrix[(2, 0)], rot_matrix[(2, 1)], rot_matrix[(2, 2)],
            ],
            velocity: [state.vel[0], state.vel[1], state.vel[2]],
            gyro_bias: [state.bg[0], state.bg[1], state.bg[2]],
            accel_bias: [state.ba[0], state.ba[1], state.ba[2]],
            gravity: [state.grav[0], state.grav[1], state.grav[2]],
            lidar_imu_rot_quat: offset_quat,
            lidar_imu_trans: [state.offset_t_l_i[0], state.offset_t_l_i[1], state.offset_t_l_i[2]],
            map_point_count: self.ikd_tree.validnum(),
            map_tree_size: self.ikd_tree.size(),
            local_map_bounds: [
                self.local_map_bounds.vertex_min[0],
                self.local_map_bounds.vertex_min[1],
                self.local_map_bounds.vertex_min[2],
                self.local_map_bounds.vertex_max[0],
                self.local_map_bounds.vertex_max[1],
                self.local_map_bounds.vertex_max[2],
            ],
            map_initialized: self.local_map_initialized,
        }
    }

    /// Publish state snapshot via UDP to specified address
    /// 
    /// Serializes the state as JSON and sends it via UDP.
    /// Useful for real-time visualization or integration with other systems.
    /// 
    /// # Arguments
    /// * `socket` - Bound UDP socket to send from
    /// * `target` - Target address to send to
    /// 
    /// # Example
    /// ```ignore
    /// let socket = UdpSocket::bind("0.0.0.0:0").await?;
    /// let target: SocketAddr = "127.0.0.1:9000".parse()?;
    /// ctx.publish_state_udp(&socket, target).await?;
    /// ```
    pub async fn publish_state_udp(
        &self,
        socket: &UdpSocket,
        target: SocketAddr,
    ) -> anyhow::Result<()> {
        let snapshot = self.get_state_snapshot();
        let json = serde_json::to_vec(&snapshot)?;
        socket.send_to(&json, target).await?;
        Ok(())
    }

    /// Publish state snapshot via UDP as compact binary format
    /// 
    /// Sends only position (3 x f64) and quaternion (4 x f64) = 56 bytes
    /// More efficient for high-frequency publishing.
    pub async fn publish_pose_udp_binary(
        &self,
        socket: &UdpSocket,
        target: SocketAddr,
    ) -> anyhow::Result<()> {
        let state = self.kf.get_x();
        let quat = rotation_matrix_to_quaternion(&state.rot.matrix());
        
        // Pack as: [x, y, z, qw, qx, qy, qz] (7 x f64 = 56 bytes)
        let mut buffer = [0u8; 56];
        buffer[0..8].copy_from_slice(&state.pos[0].to_le_bytes());
        buffer[8..16].copy_from_slice(&state.pos[1].to_le_bytes());
        buffer[16..24].copy_from_slice(&state.pos[2].to_le_bytes());
        buffer[24..32].copy_from_slice(&quat[0].to_le_bytes());
        buffer[32..40].copy_from_slice(&quat[1].to_le_bytes());
        buffer[40..48].copy_from_slice(&quat[2].to_le_bytes());
        buffer[48..56].copy_from_slice(&quat[3].to_le_bytes());
        
        socket.send_to(&buffer, target).await?;
        Ok(())
    }

    /// Get map points as a publishable point cloud
    /// 
    /// # Arguments
    /// * `max_points` - Maximum number of points to return (None for all)
    /// 
    /// # Returns
    /// MapPointCloud struct containing flattened point coordinates
    pub fn get_map_point_cloud(&self, max_points: Option<usize>) -> MapPointCloud {
        let all_points = self.ikd_tree.flatten_all();
        let points_to_use = match max_points {
            Some(max) if all_points.len() > max => &all_points[..max],
            _ => &all_points,
        };
        
        let flattened: Vec<f64> = points_to_use
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();
        
        MapPointCloud {
            point_count: points_to_use.len(),
            points: flattened,
        }
    }

    /// Publish map point cloud via UDP (JSON format)
    /// 
    /// Note: For large maps, consider using `publish_map_udp_binary` or
    /// subsampling the map with `max_points`.
    pub async fn publish_map_udp(
        &self,
        socket: &UdpSocket,
        target: SocketAddr,
        max_points: Option<usize>,
    ) -> anyhow::Result<()> {
        let map = self.get_map_point_cloud(max_points);
        let json = serde_json::to_vec(&map)?;
        socket.send_to(&json, target).await?;
        Ok(())
    }

    /// Export current state to JSON string
    pub fn state_to_json(&self) -> anyhow::Result<String> {
        let snapshot = self.get_state_snapshot();
        Ok(serde_json::to_string_pretty(&snapshot)?)
    }

    /// Export current state to JSON file
    pub async fn export_state_to_file(&self, path: &str) -> anyhow::Result<()> {
        let json = self.state_to_json()?;
        tokio::fs::write(path, json).await?;
        Ok(())
    }
}

/// Convert rotation matrix to quaternion [w, x, y, z]
fn rotation_matrix_to_quaternion(m: &sophus::nalgebra::Matrix3<f64>) -> [f64; 4] {
    // Algorithm from "Converting a Rotation Matrix to a Quaternion" by Mike Day
    let trace = m[(0, 0)] + m[(1, 1)] + m[(2, 2)];
    
    if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).sqrt();
        [
            0.25 / s,
            (m[(2, 1)] - m[(1, 2)]) * s,
            (m[(0, 2)] - m[(2, 0)]) * s,
            (m[(1, 0)] - m[(0, 1)]) * s,
        ]
    } else if m[(0, 0)] > m[(1, 1)] && m[(0, 0)] > m[(2, 2)] {
        let s = 2.0 * (1.0 + m[(0, 0)] - m[(1, 1)] - m[(2, 2)]).sqrt();
        [
            (m[(2, 1)] - m[(1, 2)]) / s,
            0.25 * s,
            (m[(0, 1)] + m[(1, 0)]) / s,
            (m[(0, 2)] + m[(2, 0)]) / s,
        ]
    } else if m[(1, 1)] > m[(2, 2)] {
        let s = 2.0 * (1.0 + m[(1, 1)] - m[(0, 0)] - m[(2, 2)]).sqrt();
        [
            (m[(0, 2)] - m[(2, 0)]) / s,
            (m[(0, 1)] + m[(1, 0)]) / s,
            0.25 * s,
            (m[(1, 2)] + m[(2, 1)]) / s,
        ]
    } else {
        let s = 2.0 * (1.0 + m[(2, 2)] - m[(0, 0)] - m[(1, 1)]).sqrt();
        [
            (m[(1, 0)] - m[(0, 1)]) / s,
            (m[(0, 2)] + m[(2, 0)]) / s,
            (m[(1, 2)] + m[(2, 1)]) / s,
            0.25 * s,
        ]
    }
}