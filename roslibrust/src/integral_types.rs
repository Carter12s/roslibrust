/// Matches the integral ros1 type time, with extensions for ease of use
/// NOTE: Is not a message in and of itself use std_msgs/Time for that
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Time {
    pub secs: u32,
    pub nsecs: u32,
}

impl Into<Time> for std::time::SystemTime {
    fn into(self) -> Time {
        let delta = self
            .duration_since(std::time::UNIX_EPOCH)
            .expect("Failed to convert system time into unix epoch");
        let downcast_secs = u32::try_from(delta.as_secs()).expect("Failed to convert system time to ROS representation, seconds term overflows u32 likely");
        Time {
            secs: downcast_secs,
            nsecs: delta.subsec_nanos(),
        }
    }
}

// TODO provide chrono conversions here behind a cfg flag

/// Matches the integral ros1 duration type, with extensions for ease of use
/// NOTE: Is not a message in and of itself use std_msgs/Duration for that
#[derive(:: serde :: Deserialize, :: serde :: Serialize, Debug, Default, Clone, PartialEq)]
pub struct Duration {
    pub sec: i32,
    pub nsec: i32,
}

/// Note this provides both tokio::time::Duration and std::time::Duration
/// Note: client is the flag which tokio is behind
#[cfg(feature = "client")]
impl Into<Duration> for tokio::time::Duration {
    fn into(self) -> Duration {
        let downcast_sec = i32::try_from(self.as_secs())
            .expect("Failed to cast tokio duration to ROS duration, secs could not fit in i32");
        let downcast_nsec = i32::try_from(self.subsec_nanos())
            .expect("Failed to cast tokio duration ROS duration, nsecs could not fit in i32");
        Duration {
            sec: downcast_sec,
            nsec: downcast_nsec,
        }
    }
}

// TODO: provide chrono conversions here behind a cfg flag
