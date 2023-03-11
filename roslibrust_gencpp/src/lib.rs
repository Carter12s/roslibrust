use minijinja::{context, Environment, Template};
use roslibrust_codegen::{MessageFile, ServiceFile};
use std::path::{Path, PathBuf};

mod helpers;
mod spec;

use spec::MessageSpecification;

use crate::spec::ServiceSpecification;

const MESSAGE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/msg.h.j2"));
const SERVICE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/srv.h.j2"));

#[derive(Clone, Debug)]
pub struct IncludedNamespace {
    /// The package namespace being included
    pub package: String,
    /// The path to the package
    pub path: PathBuf,
}

pub struct MessageGenOpts {
    /// The package namespace for the generated message
    pub package: String,
    /// Include packages for resolving message dependencies
    pub includes: Vec<IncludedNamespace>,
}

impl MessageGenOpts {
    fn get_search_paths(&self) -> Vec<PathBuf> {
        self.includes.iter().map(|inc| inc.path.clone()).collect()
    }
}

pub struct ServiceGenOutput {
    pub request_msg_header: String,
    pub response_msg_header: String,
    pub srv_header: String,
}

pub fn generate_message(
    msg_path: &Path,
    opts: &MessageGenOpts,
) -> Result<String, minijinja::Error> {
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .unwrap_or_else(|_| panic!("Unable to find ROS messages"));
    let (messages, _) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = prepare_environment();
    let message_name = msg_path.file_stem().unwrap().to_str().unwrap().to_owned();
    match messages
        .iter()
        .find(|msg| msg.get_short_name() == message_name && msg.get_package_name() == opts.package)
    {
        Some(msg) => fill_message_template(&env.get_template("msg.h").unwrap(), msg),
        None => Err(minijinja::Error::new(
            minijinja::ErrorKind::UndefinedError,
            "Message not found in search paths",
        )),
    }
}

pub fn generate_service(
    msg_path: &Path,
    opts: &MessageGenOpts,
) -> Result<ServiceGenOutput, minijinja::Error> {
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .expect("Unable to find ROS messages");
    let (_, services) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = prepare_environment();
    let service_name = msg_path.file_stem().unwrap().to_str().unwrap().to_owned();
    match services
        .iter()
        .find(|srv| srv.get_short_name() == service_name && srv.get_package_name() == opts.package)
    {
        Some(srv) => {
            let request_msg_header =
                fill_message_template(&env.get_template("msg.h").unwrap(), srv.request())?;
            let response_msg_header =
                fill_message_template(&env.get_template("msg.h").unwrap(), srv.response())?;
            let srv_header = fill_service_template(&env.get_template("srv.h").unwrap(), srv)?;
            Ok(ServiceGenOutput {
                request_msg_header,
                response_msg_header,
                srv_header,
            })
        }
        None => Err(minijinja::Error::new(
            minijinja::ErrorKind::UndefinedError,
            "Service not found in search paths",
        )),
    }
}

fn fill_message_template(
    template: &Template,
    msg_data: &MessageFile,
) -> Result<String, minijinja::Error> {
    let context = context! {
        spec => MessageSpecification::from(msg_data),
    };
    template.render(&context)
}

fn fill_service_template(
    template: &Template,
    srv_data: &ServiceFile,
) -> Result<String, minijinja::Error> {
    let context = context! {
        spec => ServiceSpecification::from(srv_data),
    };
    template.render(&context)
}

fn prepare_environment() -> Environment<'static> {
    let mut env = Environment::new();
    env.add_function("has_header", helpers::has_header);
    env.add_function("is_fixed_length", helpers::is_fixed_length);
    env.add_function("is_intrinsic_type", helpers::is_intrinsic_type);
    env.add_filter("cpp_type", helpers::cpp_type);
    env.add_filter("cpp_literal", helpers::cpp_literal);
    env.add_template("msg.h", MESSAGE_HEADER_TMPL).unwrap();
    env.add_template("srv.h", SERVICE_HEADER_TMPL).unwrap();
    env
}
