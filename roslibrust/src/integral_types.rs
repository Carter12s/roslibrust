/// Matches the integral ros1 type time, with extensions for ease of use
/// NOTE: Is not a message in and of itself use std_msgs/Time for that
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Time {
    pub secs: u32,
    pub nsecs: u32,
}

/// Matches the integral ros1 duration type, with extensions for ease of use
// NOTE: Is not a message in and of itself use std_msgs/Duration for that
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Duration {
    pub sec: i32,
    pub nsec: i32,
}
