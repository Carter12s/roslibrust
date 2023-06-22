use std::collections::HashMap;

pub const MESSAGE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/msg.h.j2"));
pub const SERVICE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/srv.h.j2"));

lazy_static::lazy_static! {
    pub static ref ROS_TYPE_TO_CPP_TYPE_MAP: HashMap<String, String> = vec![
        ("bool", "uint8_t"),
        ("int8", "int8_t"),
        ("uint8", "uint8_t"),
        ("byte", "uint8_t"),
        ("char", "uint8_t"),
        ("int16", "int16_t"),
        ("uint16", "uint16_t"),
        ("int32", "int32_t"),
        ("uint32", "uint32_t"),
        ("int64", "int64_t"),
        ("uint64", "uint64_t"),
        ("float32", "float"),
        ("float64", "double"),
        ("string", "std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>"),
        ("time", "::ros::Time"),
        ("duration", "::ros::Duration"),
    ].into_iter().map(|(k, v)| (k.to_owned(), v.to_owned())).collect();
}
