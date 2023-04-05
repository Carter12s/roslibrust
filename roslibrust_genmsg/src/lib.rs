use minijinja::{context, Template};
use roslibrust_codegen::{MessageFile, ServiceFile};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

mod cpp;
mod helpers;
mod spec;

pub use cpp::{generate_cpp_messages, generate_cpp_services};
use spec::{MessageSpecification, ServiceSpecification};

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

pub fn generate_messages_with_templates(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
    typename_conversion_mapping: HashMap<String, String>,
    msg_template: &str,
) -> Result<Vec<String>, minijinja::Error> {
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .expect("Unable to find ROS messages");
    let (messages, _) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = helpers::prepare_environment(msg_template, "", typename_conversion_mapping);

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

pub fn generate_services_with_templates(
    msg_paths: &[&Path],
    opts: &MessageGenOpts,
    typename_conversion_mapping: HashMap<String, String>,
    msg_template: &str,
    srv_template: &str,
) -> Result<Vec<ServiceGenOutput>, minijinja::Error> {
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .expect("Unable to find ROS messages");
    let (_, services) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = helpers::prepare_environment(msg_template, srv_template, typename_conversion_mapping);

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
