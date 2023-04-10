use itertools::Itertools;
use minijinja::{context, Template};
use roslibrust_codegen::{MessageFile, ServiceFile};
use std::collections::{HashMap, VecDeque};
use std::path::{Path, PathBuf};

use roslibrust_codegen::utils::{crawl, deduplicate_packages, get_message_files, Package};

mod cpp;
mod helpers;
mod spec;

pub use cpp::{
    generate_all_cpp_messages, generate_cpp_actions, generate_cpp_messages, generate_cpp_services,
};
use spec::{MessageSpecification, ServiceSpecification};

#[derive(Clone, Debug)]
pub struct IncludedNamespace {
    /// The package namespace being included
    pub package: String,
    /// The path to the package
    pub path: PathBuf,
}

impl From<Package> for IncludedNamespace {
    fn from(value: Package) -> Self {
        IncludedNamespace {
            package: value.name,
            path: value.path,
        }
    }
}

impl From<&Package> for IncludedNamespace {
    fn from(value: &Package) -> Self {
        IncludedNamespace {
            package: value.name.clone(),
            path: value.path.clone(),
        }
    }
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

pub enum GeneratedMessage {
    Msg(MessageGenOutput),
    Srv(ServiceGenOutput),
    Action(ActionGenOutput),
}

pub struct MessageGenOutput {
    pub message_name: String,
    pub package_name: String,
    pub message_source: String,
}

pub struct ServiceGenOutput {
    pub service_name: String,
    pub package_name: String,
    pub request_msg_header: String,
    pub response_msg_header: String,
    pub srv_header: String,
}

pub struct ActionGenOutput {
    pub action_name: String,
    pub package_name: String,
    pub action_source: String,
    pub goal_source: String,
    pub result_source: String,
    pub feedback_source: String,
    pub action_goal_source: String,
    pub action_result_source: String,
    pub action_feedback_source: String,
}

pub fn generate_messages_with_templates<P: AsRef<Path>>(
    msg_paths: &[P],
    opts: &MessageGenOpts,
    typename_conversion_mapping: HashMap<String, String>,
    msg_template: &str,
) -> Result<Vec<MessageGenOutput>, minijinja::Error> {
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .expect("Unable to find ROS messages");
    let (messages, _) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = helpers::prepare_environment(msg_template, "", typename_conversion_mapping);

    let message_names: Vec<_> = msg_paths.iter().map(message_name_from_path).collect();

    message_names
        .into_iter()
        .map(|message_name| {
            match messages.iter().find(|msg| {
                msg.get_short_name() == message_name && msg.get_package_name() == opts.package
            }) {
                Some(msg) => {
                    let message_source =
                        fill_message_template(&env.get_template("message").unwrap(), msg)?;
                    Ok(MessageGenOutput {
                        message_name,
                        package_name: opts.package.clone(),
                        message_source,
                    })
                }
                None => Err(minijinja::Error::new(
                    minijinja::ErrorKind::UndefinedError,
                    "Message not found in search paths",
                )),
            }
        })
        .collect()
}

pub fn generate_services_with_templates<P: AsRef<Path>>(
    msg_paths: &[P],
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
            let service_name = message_name_from_path(path);
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
                        service_name,
                        package_name: opts.package.clone(),
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

pub fn generate_actions_with_templates<P: AsRef<Path>>(
    msg_paths: &[P],
    opts: &MessageGenOpts,
    typename_conversion_mapping: HashMap<String, String>,
    msg_template: &str,
) -> Result<Vec<ActionGenOutput>, minijinja::Error> {
    let (messages, services) =
        roslibrust_codegen::find_and_parse_ros_messages(opts.get_search_paths())
            .expect("Unable to find ROS messages");
    let (messages, _) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let env = helpers::prepare_environment(msg_template, "", typename_conversion_mapping);

    let message_names: Vec<Vec<_>> = msg_paths
        .iter()
        .map(message_name_from_path)
        .map(|name| {
            vec![
                format!("{name}"),
                format!("{name}Goal"),
                format!("{name}Result"),
                format!("{name}Feedback"),
                format!("{name}ActionGoal"),
                format!("{name}ActionResult"),
                format!("{name}ActionFeedback"),
            ]
        })
        .collect();

    Ok(message_names
        .iter()
        .map(|action_names| {
            let mut action_sources = action_names
                .iter()
                .map(|message_name| {
                    match messages.iter().find(|msg| {
                        msg.get_short_name() == *message_name
                            && msg.get_package_name() == opts.package
                    }) {
                        Some(msg) => {
                            let source =
                                fill_message_template(&env.get_template("message").unwrap(), msg)?;
                            Ok(source)
                        }
                        None => Err(minijinja::Error::new(
                            minijinja::ErrorKind::UndefinedError,
                            "Message not found in search paths",
                        )),
                    }
                })
                .collect::<Result<VecDeque<_>, _>>()?;
            // Warning: this is of course very dependent on the order and number of messages assigned to each action above
            Ok(ActionGenOutput {
                action_name: action_sources.front().unwrap().clone(),
                package_name: opts.package.clone(),
                action_source: action_sources.pop_front().unwrap(),
                goal_source: action_sources.pop_front().unwrap(),
                result_source: action_sources.pop_front().unwrap(),
                feedback_source: action_sources.pop_front().unwrap(),
                action_goal_source: action_sources.pop_front().unwrap(),
                action_result_source: action_sources.pop_front().unwrap(),
                action_feedback_source: action_sources.pop_front().unwrap(),
            })
        })
        .collect::<Result<Vec<_>, minijinja::Error>>()
        .into_iter()
        .flatten()
        .collect::<Vec<_>>())
}

pub fn generate_all_messages_with_templates<P: AsRef<Path>>(
    workspace_paths: &[P],
    typename_conversion_mapping: HashMap<String, String>,
    msg_template: &str,
    srv_template: &str,
) -> Result<Vec<GeneratedMessage>, minijinja::Error> {
    let packages = deduplicate_packages(crawl(workspace_paths));
    let includes = packages
        .clone()
        .into_iter()
        .map(IncludedNamespace::from)
        .collect::<Vec<_>>();
    Ok(packages
        .iter()
        .map(|pkg| {
            generate_all_messages_for_package(
                pkg,
                &includes,
                typename_conversion_mapping.clone(),
                msg_template,
                srv_template,
            )
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .flatten()
        .collect())
}

fn generate_all_messages_for_package(
    pkg: &Package,
    includes: &[IncludedNamespace],
    typename_conversion_mapping: HashMap<String, String>,
    msg_template: &str,
    srv_template: &str,
) -> Result<Vec<GeneratedMessage>, minijinja::Error> {
    let opts = MessageGenOpts {
        package: pkg.name.clone(),
        includes: includes.to_owned(),
    };
    let message_map = get_message_files(pkg)
        .unwrap()
        .into_iter()
        .into_group_map_by(|msg_path| msg_path.extension().unwrap().to_str().unwrap().to_owned());
    Ok(generate_messages_with_templates(
        message_map.get("msg").unwrap_or(&vec![]),
        &opts,
        typename_conversion_mapping.clone(),
        msg_template,
    )?
    .into_iter()
    .map(GeneratedMessage::Msg)
    .chain(
        generate_services_with_templates(
            message_map.get("srv").unwrap_or(&vec![]),
            &opts,
            typename_conversion_mapping.clone(),
            msg_template,
            srv_template,
        )?
        .into_iter()
        .map(GeneratedMessage::Srv),
    )
    .chain(
        generate_actions_with_templates(
            message_map.get("action").unwrap_or(&vec![]),
            &opts,
            typename_conversion_mapping,
            msg_template,
        )?
        .into_iter()
        .map(GeneratedMessage::Action),
    )
    .collect())
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

fn message_name_from_path<P: AsRef<Path>>(path: &P) -> String {
    path.as_ref()
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap()
        .to_owned()
}
