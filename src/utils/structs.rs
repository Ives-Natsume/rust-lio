use sophus::nalgebra::{Vector3, SMatrix};
use std::vec::Vec;
use sophus::lie::Rotation3F64;

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
/// [Livox Point Cloud Data Format](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html#point-cloud-imu-data-protocol)
#[derive(Clone, Debug)]
pub struct PointXYZI {
    pub pos: nalgebra::Vector3<f32>,
    pub intensity: f32,
}

/// Point cloud structure with intensity, FLU coordinate system
///
/// a.k.a [`pcl::PointCloud<PointXYZINormal>`](https://pointclouds.org/documentation/classpcl_1_1_point_cloud.html)
#[derive(Clone, Debug)]
pub struct PointCloudXYZI {
    pub timestamp: f64,             // timestamp of the point cloud (seconds)
    pub width: u32,                 // number of points per row
    pub height: u32,                // number of rows
    pub points: Vec<PointXYZI>,     // point data
    pub is_dense: bool,             // whether there are invalid points
}

impl PointCloudXYZI {
    pub fn new() -> Self {
        PointCloudXYZI {
            timestamp: 0.0,
            width: 0,
            height: 0,
            points: Vec::new(),
            is_dense: true,
        }
    }

    /// Create a PointCloudXYZI from a vector of PointType
    /// 
    /// For Lidar usage (unorganized point cloud), height is set to 1
    pub fn from_points(points: Vec<PointXYZI>, timestamp: f64) -> Self {
        let num_points = points.len() as u32;
        PointCloudXYZI {
            timestamp,
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

/// IMU data structure
/// 
/// [Livox IMU Data Format](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html#point-cloud-imu-data-protocol)
#[derive(Clone, Debug)]
pub struct ImuData {
    pub timestamp: f64,
    pub acc: Vector3<f64>,
    pub gyr: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct MeasureGroup {
    pub lidar_begin_time: f64,
    pub lidar_end_time: f64,
    pub points: PointCloudXYZI,
    pub imus: Vec<ImuData>,
}

/// Based on [S-FAST_LIO](https://github.com/zlwang7/S-FAST_LIO.git)
#[derive(Clone, Debug)]
pub struct ImuProcess{
    // noise covariances
    pub cov_acc: Vector3<f64>,          // accelerometer noise covariance
    pub cov_gyr: Vector3<f64>,          // gyroscope noise covariance
    pub cov_acc_scale: Vector3<f64>,    // accelerometer scale factor noise covariance
    pub cov_gyr_scale: Vector3<f64>,    // gyroscope scale factor noise covariance
    pub cov_acc_bias: Vector3<f64>,     // accelerometer bias random walk noise covariance
    pub cov_gyr_bias: Vector3<f64>,     // gyroscope bias random walk noise covariance
    pub q: SMatrix<f64, 12, 12>,     // IMU noise covariance matrix

    // extrinsic calibration
    pub lidar_r_wrt_imu: Rotation3F64,  // rotation from IMU frame to Lidar frame
    pub lidar_t_wrt_imu: Vector3<f64>,  // translation from IMU frame to Lidar frame

    // state variables
    pub last_imu: Option<ImuData>,      // last IMU measurement
    pub angvel_last: Vector3<f64>,      // last angular velocity
    pub acc_s_last: Vector3<f64>,       // last specific force

    // timestamp & other
    pub first_lidar_time: f64,          // first lidar point time in the current frame
    pub start_timestamp: f64,           // start timestamp of the current frame
    pub last_lidar_end_time_: f64,      // last lidar end time
    pub b_first_frame_: bool,           // flag for first frame
    pub imu_need_init_: bool,           // flag for IMU initialization
    pub init_iter_num: u32,             // number of iterations for IMU initialization
    pub mean_acc: Vector3<f64>,         // mean acceleration for IMU initialization
    pub mean_gyr: Vector3<f64>,         // mean gyroscope for IMU initialization
}

impl Default for ImuProcess {
    fn default() -> Self {
        Self {
            cov_acc: Vector3::new(0.1, 0.1, 0.1),
            cov_gyr: Vector3::new(0.1, 0.1, 0.1),
            cov_acc_scale: Vector3::zeros(),
            cov_gyr_scale: Vector3::zeros(),
            cov_acc_bias: Vector3::new(0.0001, 0.0001, 0.0001),
            cov_gyr_bias: Vector3::new(0.0001, 0.0001, 0.0001),
            q: crate::core::math::ikfom::process_noise_cov(),
            lidar_r_wrt_imu: Rotation3F64::identity(),
            lidar_t_wrt_imu: Vector3::zeros(),
            last_imu: None,
            angvel_last: Vector3::zeros(),
            acc_s_last: Vector3::zeros(),
            first_lidar_time: 0.0,
            start_timestamp: -1.0,
            last_lidar_end_time_: 0.0,
            b_first_frame_: true,
            imu_need_init_: true,
            init_iter_num: 1,
            mean_acc: Vector3::new(0.0, 0.0, -1.0),
            mean_gyr: Vector3::zeros(),
        }
    }
}

impl ImuProcess {
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset IMU processor to initial state
    pub fn reset(&mut self) {
        self.mean_acc = Vector3::new(0.0, 0.0, -1.0);
        self.mean_gyr = Vector3::zeros();
        self.angvel_last = Vector3::zeros();
        self.imu_need_init_ = true;
        self.start_timestamp = -1.0;
        self.init_iter_num = 1;
        self.last_imu = None;
    }

    /// Set extrinsic calibration and noise parameters
    pub fn set_param(
        &mut self,
        transl: Vector3<f64>,
        rot: Rotation3F64,
        gyr_scale: Vector3<f64>,
        acc_scale: Vector3<f64>,
        gyr_bias: Vector3<f64>,
        acc_bias: Vector3<f64>,
    ) {
        self.lidar_t_wrt_imu = transl;
        self.lidar_r_wrt_imu = rot;
        self.cov_gyr_scale = gyr_scale;
        self.cov_acc_scale = acc_scale;
        self.cov_gyr_bias = gyr_bias;
        self.cov_acc_bias = acc_bias;
    }
}