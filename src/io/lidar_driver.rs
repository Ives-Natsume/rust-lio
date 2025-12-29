//! For Livox Mid360
//! 
//! Use Proto for data transmittion
//! 
//! Receives data from bridges implementing `SlamBridge` trait
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use prost::Message;
use super::slam_proto::*;

#[async_trait::async_trait]
pub trait SlamBridge: Send + Sync {
    async fn next_packet(&mut self) -> anyhow::Result<SensorPacket>;
}

pub struct UdpBridge {
    socket: UdpSocket,
    buffer: [u8; 65535], // Max UDP size
}

impl UdpBridge {
    pub async fn new(bind_addr: &str) -> anyhow::Result<Self> {
        let socket = UdpSocket::bind(bind_addr).await?;
        Ok(UdpBridge {
            socket,
            buffer: [0u8; 65535],
        })
    }
}

#[async_trait::async_trait]
impl SlamBridge for UdpBridge {
    async fn next_packet(&mut self) -> anyhow::Result<SensorPacket> {
        let (len, _addr) = self.socket.recv_from(&mut self.buffer).await?;
        let packet = SensorPacket::decode(&self.buffer[..len])?;
        Ok(packet)
    }
}