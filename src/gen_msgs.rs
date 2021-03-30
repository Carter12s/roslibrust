use serde::{Deserialize,Serialize};

#[derive(Deserialize, Serialize, Debug)]
pub struct NodeInfo {
    node_name: String,
    pid: i64,
    status: u8,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct Float64Stamped {
//    header: Header,
    value: f64,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct LoggerLevel {
    level: String,
}
