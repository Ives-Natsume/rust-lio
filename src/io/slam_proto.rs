use prost::Message;

/// Notice that the field differs from [`PointXYZI`](crate::utils::structs::PointXYZI)
/// 
/// a.k.a [`pcl::PointXYZINormal`](https://pointclouds.org/documentation/structpcl_1_1_point_x_y_z_i_normal.html)
#[derive(Clone, PartialEq, prost::Message)]
pub struct PointXYZIProto {
    #[prost(float, tag = "1")]
    pub x: f32,
    #[prost(float, tag = "2")]
    pub y: f32,
    #[prost(float, tag = "3")]
    pub z: f32,
    #[prost(float, tag = "4")]
    pub intensity: f32,
}

#[derive(Clone, PartialEq, prost::Message)]
pub struct PointCloudXYZIProto {
    #[prost(uint64, tag = "1")]
    pub timestamp: u64,
    #[prost(message, repeated, tag = "2")]
    pub points: Vec<PointXYZIProto>,
}

/// Not finished yet
#[derive(Clone, PartialEq, prost::Message)]
pub struct ImuDataProto {

}

#[derive(Clone, PartialEq, prost::Message)]
pub struct SensorPacket {
    #[prost(oneof="SensorData", tags="1, 2")]
    pub data: Option<SensorData>,
}

#[derive(Clone, PartialEq, prost::Oneof)]
pub enum SensorData {
    #[prost(message, tag = "1")]
    LidarData(PointCloudXYZIProto),
    #[prost(message, tag = "2")]
    ImuData(ImuDataProto),
}