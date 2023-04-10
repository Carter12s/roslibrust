use crate::{
    generate_actions_with_templates, generate_all_messages_with_templates,
    generate_messages_with_templates, generate_services_with_templates, ActionGenOutput,
    GeneratedMessage, MessageGenOpts, MessageGenOutput, ServiceGenOutput,
};
use std::{collections::HashMap, path::Path};

const MESSAGE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/msg.h.j2"));
const SERVICE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/srv.h.j2"));

lazy_static::lazy_static! {
    pub static ref ROS_TYPE_TO_CPP_TYPE_MAP: HashMap<String, String> = vec![
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
    ].into_iter().map(|(k, v)| (k.to_owned(), v.to_owned())).collect();
}

pub fn generate_cpp_messages(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
) -> Result<Vec<MessageGenOutput>, minijinja::Error> {
    generate_messages_with_templates(
        msg_paths,
        opts,
        ROS_TYPE_TO_CPP_TYPE_MAP.clone(),
        MESSAGE_HEADER_TMPL,
    )
}

pub fn generate_cpp_services(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
) -> Result<Vec<ServiceGenOutput>, minijinja::Error> {
    generate_services_with_templates(
        msg_paths,
        opts,
        ROS_TYPE_TO_CPP_TYPE_MAP.clone(),
        MESSAGE_HEADER_TMPL,
        SERVICE_HEADER_TMPL,
    )
}

pub fn generate_cpp_actions(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
) -> Result<Vec<ActionGenOutput>, minijinja::Error> {
    generate_actions_with_templates(
        msg_paths,
        opts,
        ROS_TYPE_TO_CPP_TYPE_MAP.clone(),
        MESSAGE_HEADER_TMPL,
    )
}

pub fn generate_all_cpp_messages<P: AsRef<Path>>(
    workspace_paths: &[P],
) -> Result<Vec<GeneratedMessage>, minijinja::Error> {
    generate_all_messages_with_templates(
        workspace_paths,
        ROS_TYPE_TO_CPP_TYPE_MAP.clone(),
        MESSAGE_HEADER_TMPL,
        SERVICE_HEADER_TMPL,
    )
}
