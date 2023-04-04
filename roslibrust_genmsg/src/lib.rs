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

pub fn generate_cpp_messages(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
) -> Result<Vec<String>, minijinja::Error> {
    generate_messages_with_templates(msg_paths, opts, MESSAGE_HEADER_TMPL)
}

pub fn generate_messages_with_templates(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
    msg_template: &str,
) -> Result<Vec<String>, minijinja::Error> 
{
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .unwrap_or_else(|_| panic!("Unable to find ROS messages"));
    let (messages, _) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = prepare_environment(msg_template, "");

    let message_names: Vec<_> = msg_paths
        .iter()
        .map(|path| path.file_stem().unwrap().to_str().unwrap().to_owned())
        .collect();

    message_names
        .iter()
        .map(|message_name| {
            match messages.iter().find(|msg| {
                msg.get_short_name() == *message_name && msg.get_package_name() == opts.package
            }) {
                Some(msg) => fill_message_template(&env.get_template("message").unwrap(), msg),
                None => Err(minijinja::Error::new(
                    minijinja::ErrorKind::UndefinedError,
                    "Message not found in search paths",
                )),
            }
        })
        .collect()
}

pub fn generate_cpp_services(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
) -> Result<Vec<ServiceGenOutput>, minijinja::Error> {
    generate_services_with_templates(msg_paths, opts, MESSAGE_HEADER_TMPL, SERVICE_HEADER_TMPL)
}

pub fn generate_services_with_templates(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
    msg_template: &str,
    srv_template: &str,
) -> Result<Vec<ServiceGenOutput>, minijinja::Error> {
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .expect("Unable to find ROS messages");
    let (_, services) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = prepare_environment(msg_template, srv_template);

    msg_paths
        .iter()
        .map(|path| {
            let service_name = path.file_stem().unwrap().to_str().unwrap().to_owned();
            match services.iter().find(|srv| {
                srv.get_short_name() == service_name && srv.get_package_name() == opts.package
            }) {
                Some(srv) => {
                    let request_msg_header = fill_message_template(
                        &env.get_template("message").unwrap(),
                        srv.request(),
                    )?;
                    let response_msg_header = fill_message_template(
                        &env.get_template("message").unwrap(),
                        srv.response(),
                    )?;
                    let srv_header =
                        fill_service_template(&env.get_template("service").unwrap(), srv)?;
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
        })
        .collect()
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

fn prepare_environment<'a>(
    message_template: &'a str,
    service_template: &'a str,
) -> Environment<'a> {
    let mut env = Environment::new();
    env.add_function("has_header", helpers::has_header);
    env.add_function("is_fixed_length", helpers::is_fixed_length);
    env.add_function("is_intrinsic_type", helpers::is_intrinsic_type);
    env.add_filter("cpp_type", helpers::cpp_type);
    env.add_filter("cpp_literal", helpers::cpp_literal);
    env.add_template("message", message_template).unwrap();
    env.add_template("service", service_template).unwrap();
    env
}
