//! This module contains the top level Node and NodeHandle classes.
//! These wrap the lower level management of a ROS Node connection into a higher level and thread safe API.

use super::{names::InvalidNameError, RosMasterError};
use std::{
    io,
    net::{IpAddr, Ipv4Addr},
};

mod actor;
mod handle;
mod xmlrpc;
use actor::*;
pub use handle::NodeHandle;
use tokio::sync::{mpsc, oneshot};
use xmlrpc::*;

#[derive(Debug)]
pub struct ProtocolParams {
    pub hostname: String,
    pub protocol: String,
    pub port: u16,
}

// TODO at the end of the day I'd like to offer a builder pattern for configuration that allow manual setting of this or "ros idiomatic" behavior - Carter
/// Following ROS's idiomatic address rules uses ROS_HOSTNAME and ROS_IP to determine the address that server should be hosted at.
/// Returns both the resolved IpAddress of the host (used for actually opening the socket), and the String "hostname" which should
/// be used in the URI.
async fn determine_addr() -> Result<(Ipv4Addr, String), RosMasterError> {
    // If ROS_IP is set that trumps anything else
    if let Ok(ip_str) = std::env::var("ROS_IP") {
        let ip = ip_str.parse().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!(
                "ROS_IP environment variable did not parse to a valid IpAddr::V4: {e:?}"
            ))
        })?;
        return Ok((ip, ip_str));
    }
    // If ROS_HOSTNAME is set that is next highest precedent
    if let Ok(name) = std::env::var("ROS_HOSTNAME") {
        let ip = hostname_to_ipv4(&name).await?;
        return Ok((ip, name));
    }
    // If neither env var is set, use the computers "hostname"
    let name = gethostname::gethostname();
    let name = name.into_string().map_err(|e| {
            RosMasterError::HostIpResolutionFailure(format!("This host's hostname is a string that cannot be validly converted into a Rust type, and therefore we cannot convert it into an IpAddrv4: {e:?}"))
        })?;
    let ip = hostname_to_ipv4(&name).await?;
    return Ok((ip, name));
}

/// Given a the name of a host use's std::net::ToSocketAddrs to perform a DNS lookup and return the resulting IP address.
/// This function is intended to be used to determine the correct IP host the socket for the xmlrpc server on.
async fn hostname_to_ipv4(name: &str) -> Result<Ipv4Addr, RosMasterError> {
    let name_with_port = &format!("{name}:0");
    let mut i = tokio::net::lookup_host(name_with_port).await.map_err(|e| {
        RosMasterError::HostIpResolutionFailure(format!(
            "Failure while attempting to lookup ROS_HOSTNAME: {e:?}"
        ))
    })?;
    if let Some(addr) = i.next() {
        match addr.ip() {
                IpAddr::V4(ip) => Ok(ip),
                IpAddr::V6(ip) => {
                    Err(RosMasterError::HostIpResolutionFailure(format!("ROS_HOSTNAME resolved to an IPv6 address which is not support by ROS/roslibrust: {ip:?}")))
                }
            }
    } else {
        Err(RosMasterError::HostIpResolutionFailure(format!(
            "ROS_HOSTNAME did not resolve any address: {name:?}"
        )))
    }
}

#[derive(thiserror::Error, Debug)]
pub enum NodeError {
    #[error(transparent)]
    RosMasterError(#[from] RosMasterError),
    #[error("connection closed")]
    ChannelClosedError,
    #[error(transparent)]
    InvalidName(#[from] InvalidNameError),
    #[error(transparent)]
    XmlRpcError(#[from] XmlRpcError),
    #[error(transparent)]
    IoError(#[from] io::Error),
}

impl From<oneshot::error::RecvError> for NodeError {
    fn from(_value: oneshot::error::RecvError) -> Self {
        NodeError::ChannelClosedError
    }
}

impl<T> From<mpsc::error::SendError<T>> for NodeError {
    fn from(_value: mpsc::error::SendError<T>) -> Self {
        Self::ChannelClosedError
    }
}
