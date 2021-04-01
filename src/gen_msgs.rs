use serde::{Deserialize,Serialize};

#[derive(Deserialize, Serialize, Debug)]
pub struct Float64Stamped {
    header: Header,
    value: f64,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct Header {
    seq: u32,
    stamp: TimeI,
    frame_id: String,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct LoggerLevel {
    level: String,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct Metric {
    name: String,
    time: f64,
    data: Vec<MetricPair>,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct MetricPair {
    key: String,
    value: f64,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct NodeInfo {
    node_name: String,
    pid: i64,
    status: u8,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct TimeI {
    sec: u32,
    nsec: u32,
}