use nalgebra::{Vector3};
use std::vec::Vec;

//  the preintegrated Lidar states at the time of IMU measurements in a frame
///
/// a.k.a. Pose6D.msg in FAST-LIO2 
pub struct RosPose6D {
    pub offset_time: f64,   // the offset time of IMU measurement w.r.t the first lidar point
    pub acc: Vector3<f64>,  // the preintegrated total acceleration (global frame) at the Lidar origin
    pub gyr: Vector3<f64>,  // the unbiased angular velocity (body frame) at the Lidar origin
    pub vel: Vector3<f64>,  // the preintegrated velocity (global frame) at the Lidar origin
    pub pos: Vector3<f64>,  // the preintegrated position (global frame) at the Lidar origin
    pub rot: Vec<f64>,      // the preintegrated rotation (global frame) at the Lidar origin
}

/// Point type with intensity, FLU coordinate system
/// 
/// Notice the `pos` field is a nalgebra `Vector3<f32>`,
/// which differs from [`RosPose6D.pos`](crate::utils::structs::RosPose6D)
/// 
/// a.k.a [`pcl::PointXYZINormal`](https://pointclouds.org/documentation/structpcl_1_1_point_x_y_z_i_normal.html)
/// 
/// [`PointXYZIProto`](crate::io::slam_proto::PointXYZIProto) differs from this struct in field definitions.
pub struct PointXYZI {
    pub pos: nalgebra::Vector3<f32>,
    pub intensity: f32,
}

/// Point cloud structure with intensity, FLU coordinate system
///
/// a.k.a [`pcl::PointCloud<PointXYZINormal>`](https://pointclouds.org/documentation/classpcl_1_1_point_cloud.html)
pub struct PointCloudXYZI {
    pub width: u32,                 // number of points per row
    pub height: u32,                // number of rows
    pub points: Vec<PointXYZI>,     // point data
    pub is_dense: bool,             // whether there are invalid points
}

impl PointCloudXYZI {
    pub fn new() -> Self {
        PointCloudXYZI {
            width: 0,
            height: 0,
            points: Vec::new(),
            is_dense: true,
        }
    }

    /// Create a PointCloudXYZI from a vector of PointType
    /// 
    /// For Lidar usage (unorganized point cloud), height is set to 1
    pub fn from_points(points: Vec<PointXYZI>) -> Self {
        let num_points = points.len() as u32;
        PointCloudXYZI {
            width: num_points,
            height: 1,
            points,
            is_dense: true,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

pub type PointVector = Vec<PointXYZI>;

pub struct MeasureGroup {
    pub lidar_begin_time: f64,
    pub lidar_end_time: f64,
    pub points: PointCloudXYZI,
    pub _imu_poses: Vec<RosPose6D>,
}