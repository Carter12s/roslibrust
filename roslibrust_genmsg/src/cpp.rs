use crate::{
    generate_messages_with_templates, generate_services_with_templates, MessageGenOpts,
    ServiceGenOutput,
};
use std::{collections::HashMap, path::Path};

const MESSAGE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/msg.h.j2"));
const SERVICE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/srv.h.j2"));

lazy_static::lazy_static! {
    pub static ref ROS_TYPE_TO_CPP_TYPE_MAP: HashMap<&'static str, &'static str> = vec![
        ("bool", "bool"),
        ("int8", "int8_t"),
        ("uint8", "uint8_t"),
        ("byte", "uint8_t"),
        ("char", "char"),
        ("int16", "int16_t"),
        ("uint16", "uint16_t"),
        ("int32", "int32_t"),
        ("uint32", "uint32_t"),
        ("int64", "int64_t"),
        ("uint64", "uint64_t"),
        ("float32", "float"),
        ("float64", "double"),
        ("string", "::std::string"),
        ("time", "::ros::Time"),
        ("duration", "::ros::Duration"),
    ].into_iter().collect();
}

pub fn generate_cpp_messages(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
) -> Result<Vec<String>, minijinja::Error> {
    let typename_conversion_mapping = ROS_TYPE_TO_CPP_TYPE_MAP
        .clone()
        .into_iter()
        .map(|(k, v)| (k.to_owned(), v.to_owned()))
        .collect();
    generate_messages_with_templates(
        msg_paths,
        opts,
        typename_conversion_mapping,
        MESSAGE_HEADER_TMPL,
    )
}

pub fn generate_cpp_services(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
) -> Result<Vec<ServiceGenOutput>, minijinja::Error> {
    let typename_conversion_mapping = ROS_TYPE_TO_CPP_TYPE_MAP
        .clone()
        .into_iter()
        .map(|(k, v)| (k.to_owned(), v.to_owned()))
        .collect();
    generate_services_with_templates(
        msg_paths,
        opts,
        typename_conversion_mapping,
        MESSAGE_HEADER_TMPL,
        SERVICE_HEADER_TMPL,
    )
}
