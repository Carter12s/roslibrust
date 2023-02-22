use log::*;
use proc_macro2::TokenStream;
use quote::quote;
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::collections::{BTreeMap, VecDeque};
use std::fmt::Debug;
use std::path::PathBuf;
use utils::Package;

mod gen;
use gen::*;
mod parse;
use parse::*;
pub mod utils;
use utils::RosVersion;

pub mod integral_types;
pub use integral_types::*;

/// Fundamental traits for message types this crate works with
/// This trait will be satisfied for any types generated with this crate's message_gen functionality
pub trait RosMessageType:
    'static + DeserializeOwned + Send + Serialize + Sync + Clone + Debug
{
    /// Expected to be the combination pkg_name/type_name string describing the type to ros
    /// Example: std_msgs/Header
    const ROS_TYPE_NAME: &'static str;

    /// The computed md5sum of the message file and its dependencies
    const MD5SUM: &'static str;
}

// This special impl allows for services with no args / returns
impl RosMessageType for () {
    const ROS_TYPE_NAME: &'static str = "";
    const MD5SUM: &'static str = "";
}

/// Fundamental traits for service types this crate works with
/// This trait will be satisfied for any services definitions generated with this crate's message_gen functionality
pub trait RosServiceType {
    /// Name of the ros service e.g. `rospy_tutorials/AddTwoInts`
    const ROS_SERVICE_NAME: &'static str;
    /// The computed md5sum of the message file and its dependencies
    const MD5SUM: &'static str;
    /// The type of data being sent in the request
    type Request: RosMessageType;
    /// The type of the data
    type Response: RosMessageType;
}

#[derive(Clone, Debug)]
pub struct MessageFile {
    pub(crate) parsed: ParsedMessageFile,
    pub(crate) md5sum: String,
}

impl MessageFile {
    fn resolve(parsed: ParsedMessageFile, graph: &BTreeMap<String, MessageFile>) -> Option<Self> {
        let md5sum = Self::compute_md5sum(&parsed, graph)?;
        Some(MessageFile { parsed, md5sum })
    }

    pub fn get_full_name(&self) -> String {
        format!("{}/{}", self.parsed.package, self.parsed.name)
    }

    pub fn get_md5sum(&self) -> &str {
        self.md5sum.as_str()
    }

    fn compute_md5sum(
        parsed: &ParsedMessageFile,
        graph: &BTreeMap<String, MessageFile>,
    ) -> Option<String> {
        let md5sum_content = Self::_compute_md5sum(parsed, graph)?;
        // Subtract the trailing newline
        let md5sum = md5::compute(md5sum_content.trim_end().as_bytes());
        log::trace!(
            "Message type: {} calculated with md5sum: {md5sum:x}",
            parsed.get_full_name()
        );
        Some(format!("{md5sum:x}"))
    }

    fn _compute_md5sum(
        parsed: &ParsedMessageFile,
        graph: &BTreeMap<String, MessageFile>,
    ) -> Option<String> {
        let mut md5sum_content = String::new();
        for constant in &parsed.constants {
            md5sum_content.push_str(&format!(
                "{} {}={}\n",
                constant.constant_type, constant.constant_name, constant.constant_value
            ));
        }
        for field in &parsed.fields {
            let field_type = field.field_type.field_type.as_str();
            if is_intrinsic_type(parsed.version.unwrap_or(RosVersion::ROS1), field_type) {
                md5sum_content.push_str(&format!("{} {}\n", field.field_type, field.field_name));
            } else {
                let field_full_name = format!(
                    "{}/{}",
                    field
                        .field_type
                        .package_name
                        .as_ref()
                        .unwrap_or_else(|| panic!("Expected package name for field {field:#?}")),
                    field_type
                );
                let sub_message = graph.get(field_full_name.as_str())?;
                let sub_md5sum = Self::compute_md5sum(&sub_message.parsed, graph)?;
                md5sum_content.push_str(&format!("{} {}\n", sub_md5sum, field.field_name));
            }
        }

        Some(md5sum_content)
    }
}

#[derive(Clone, Debug)]
pub struct ServiceFile {
    pub(crate) parsed: ParsedServiceFile,
    pub(crate) request: MessageFile,
    pub(crate) response: MessageFile,
    pub(crate) md5sum: String,
}

impl ServiceFile {
    fn resolve(parsed: ParsedServiceFile, graph: &BTreeMap<String, MessageFile>) -> Option<Self> {
        if let (Some(request), Some(response)) = (
            MessageFile::resolve(parsed.request_type.clone(), &graph),
            MessageFile::resolve(parsed.response_type.clone(), &graph),
        ) {
            let md5sum = Self::compute_md5sum(&parsed, graph)?;
            Some(ServiceFile {
                parsed: parsed,
                request: request.clone(),
                response: response.clone(),
                md5sum,
            })
        } else {
            log::error!("Unable to resolve dependencies in service: {parsed:#?}");
            None
        }
    }

    pub fn get_full_name(&self) -> String {
        format!("{}/{}", self.parsed.package, self.parsed.name)
    }

    fn compute_md5sum(
        parsed: &ParsedServiceFile,
        graph: &BTreeMap<String, MessageFile>,
    ) -> Option<String> {
        let request_content = MessageFile::_compute_md5sum(&parsed.request_type, graph)?;
        let response_content = MessageFile::_compute_md5sum(&parsed.response_type, graph)?;
        let mut md5sum_context = md5::Context::new();
        md5sum_context.consume(request_content.trim_end().as_bytes());
        md5sum_context.consume(response_content.trim_end().as_bytes());

        let md5sum = md5sum_context.compute();
        log::trace!(
            "Message type: {} calculated with md5sum: {md5sum:x}",
            parsed.get_full_name()
        );
        Some(format!("{md5sum:x}"))
    }
}

/// Searches a list of paths for ROS packages and generates struct definitions
/// and implementations for message files and service files in packages it finds.
///
/// * `additional_search_paths` - A list of additional paths to search beyond those
/// found in ROS_PACKAGE_PATH environment variable.
pub fn find_and_generate_ros_messages(additional_search_paths: Vec<PathBuf>) -> TokenStream {
    let mut ros_package_paths = utils::get_search_paths();
    ros_package_paths.extend(additional_search_paths);
    find_and_generate_ros_messages_without_ros_package_path(ros_package_paths)
}

/// Searches a list of paths for ROS packages and generates struct definitions
/// and implementations for message files and service files in packages it finds.
///
/// * `search_paths` - A list of paths to search for ROS packages.
pub fn find_and_generate_ros_messages_without_ros_package_path(
    search_paths: Vec<PathBuf>,
) -> TokenStream {
    let (messages, services) = find_and_parse_ros_messages(search_paths).unwrap();
    if let Some((messages, services)) = resolve_dependency_graph(messages, services) {
        generate_rust_ros_message_definitions(messages, services)
    } else {
        TokenStream::default()
    }
}

/// Searches a list of paths for ROS packages to find their associated message
/// and service files, parsing and performing dependency resolution on those
/// it finds. Returns a map of PACKAGE_NAME/MESSAGE_NAME strings to message file
/// data and vector of service file data.
///
/// * `search_paths` - A list of paths to search.
///
pub fn find_and_parse_ros_messages(
    search_paths: Vec<PathBuf>,
) -> std::io::Result<(Vec<ParsedMessageFile>, Vec<ParsedServiceFile>)> {
    let search_paths = search_paths
        .into_iter()
        .map(|path| {
            if path.exists() {
                path.canonicalize()
                    .unwrap_or_else(|_| panic!("Unable to canonicalize path: {}", path.display()))
            } else {
                log::error!("{} does not exist", path.display());
                path
            }
        })
        .collect::<Vec<_>>();
    debug!(
        "Codegen is looking in following paths for files: {:?}",
        &search_paths
    );
    let packages = utils::crawl(search_paths.clone());
    // Check for duplicate package names
    let packages = utils::deduplicate_packages(packages);
    if packages.is_empty() {
        log::warn!(
            "No packages found while searching in: {search_paths:?}, relative to {:?}",
            std::env::current_dir().unwrap()
        );
    }

    let mut message_files = packages
        .iter()
        .flat_map(|pkg| {
            utils::get_message_files(pkg)
                .unwrap_or_else(|err| {
                    log::error!(
                        "Unable to get paths to message files for {}: {}",
                        pkg.name,
                        err
                    );
                    // Return an empty vec so that one package doesn't necessarily fail the process
                    vec![]
                })
                .into_iter()
                .map(|path| (pkg.clone(), path))
        })
        .collect::<Vec<_>>();
    let service_files = packages
        .iter()
        .flat_map(|pkg| {
            utils::get_service_files(pkg)
                .unwrap_or_else(|err| {
                    log::error!(
                        "Unable to get paths to service files for {}: {}",
                        pkg.name,
                        err
                    );
                    // Return an empty vec so that one package doesn't necessarily fail the process
                    vec![]
                })
                .into_iter()
                .map(|path| (pkg.clone(), path))
        })
        .collect::<Vec<_>>();

    message_files.extend_from_slice(&service_files[..]);
    parse_ros_files(message_files)
}

/// Takes in collections of ROS message and ROS service data and generates Rust
/// source code corresponding to the definitions.
///
/// This function assumes that the provided messages make up a completely resolved
/// tree of dependent messages.
///
/// * `messages` - Collection of ROS message definition data.
/// * `services` - Collection of ROS service definition data.
pub fn generate_rust_ros_message_definitions(
    messages: Vec<MessageFile>,
    services: Vec<ServiceFile>,
) -> TokenStream {
    let mut modules_to_struct_definitions: BTreeMap<String, Vec<TokenStream>> = BTreeMap::new();

    // Convert messages files into rust token streams and insert them into BTree organized by package
    messages.into_iter().for_each(|message| {
        let pkg_name = message.parsed.package.clone();
        let definition = generate_struct(message);
        if let Some(entry) = modules_to_struct_definitions.get_mut(&pkg_name) {
            entry.push(definition);
        } else {
            modules_to_struct_definitions.insert(pkg_name, vec![definition]);
        }
    });
    // Do the same for services
    services.into_iter().for_each(|service| {
        let pkg_name = service.parsed.package.clone();
        let definition = generate_service(service);
        if let Some(entry) = modules_to_struct_definitions.get_mut(&pkg_name) {
            entry.push(definition);
        } else {
            modules_to_struct_definitions.insert(pkg_name, vec![definition]);
        }
    });
    // Now generate modules to wrap all of the TokenStreams in a module for each package
    let all_pkgs = modules_to_struct_definitions
        .keys()
        .cloned()
        .collect::<Vec<String>>();
    let module_definitions = modules_to_struct_definitions
        .into_iter()
        .map(|(pkg, struct_defs)| generate_mod(pkg, struct_defs, &all_pkgs[..]))
        .collect::<Vec<_>>();

    quote! {
        #(#module_definitions)*

    }
}

struct MessageMetadata {
    msg: ParsedMessageFile,
    seen_count: u32,
}

pub fn resolve_dependency_graph(
    messages: Vec<ParsedMessageFile>,
    services: Vec<ParsedServiceFile>,
) -> Option<(Vec<MessageFile>, Vec<ServiceFile>)> {
    const MAX_PARSE_ITER_LIMIT: u32 = 2048;
    let mut unresolved_messages = messages
        .into_iter()
        .map(|msg| MessageMetadata { msg, seen_count: 0 })
        .collect::<VecDeque<_>>();

    let mut resolved_messages = BTreeMap::new();
    // First resolve the message dependencies
    while let Some(MessageMetadata { msg, seen_count }) = unresolved_messages.pop_front() {
        if seen_count > MAX_PARSE_ITER_LIMIT {
            log::error!("Unable to resolve dependencies after reaching iteration limit ({MAX_PARSE_ITER_LIMIT}).\n\
                    Message: {msg:#?}");
            return None;
        }

        // Check our resolved messages for each of the fields
        let fully_resolved = msg.fields.iter().all(|field| {
            let is_ros1_primitive =
                ROS_TYPE_TO_RUST_TYPE_MAP.contains_key(field.field_type.field_type.as_str());
            let is_ros2_primitive =
                ROS_2_TYPE_TO_RUST_TYPE_MAP.contains_key(field.field_type.field_type.as_str());
            let is_primitive = is_ros1_primitive || is_ros2_primitive;
            if !is_primitive {
                let is_resolved =
                    resolved_messages.contains_key(field.get_full_type_name().as_str());
                is_resolved
            } else {
                true
            }
        });

        if fully_resolved {
            let msg_file = MessageFile::resolve(msg, &resolved_messages).unwrap();
            resolved_messages.insert(msg_file.get_full_name(), msg_file);
        } else {
            unresolved_messages.push_back(MessageMetadata {
                seen_count: seen_count + 1,
                msg,
            });
        }
    }

    // Now that all messages are parsed, we can parse and resolve services
    let mut resolved_services: Vec<_> = services
        .into_iter()
        .map(|srv| ServiceFile::resolve(srv, &resolved_messages))
        .flatten()
        .collect();
    resolved_services.sort_by(|a, b| a.parsed.name.cmp(&b.parsed.name));

    Some((resolved_messages.into_values().collect(), resolved_services))
}

/// Parses all ROS file types and returns a final expanded set
/// Currently supports service files and message files, no planned support for actions
/// The returned collection will contain all messages files including those buried within the service definitions
/// and will have fully expanded and resolved referenced types in other packages.
/// * `msg_paths` -- List of tuple (Package, Path to File) for each file to parse
fn parse_ros_files(
    msg_paths: Vec<(Package, PathBuf)>,
) -> std::io::Result<(Vec<ParsedMessageFile>, Vec<ParsedServiceFile>)> {
    let mut parsed_messages = Vec::new();
    let mut parsed_services = Vec::new();
    for (pkg, path) in msg_paths {
        let contents = std::fs::read_to_string(&path)?;
        let name = path.file_stem().unwrap().to_str().unwrap();
        match path.extension().unwrap().to_str().unwrap() {
            "srv" => {
                let srv_file = parse_ros_service_file(&contents, name, &pkg, &path);
                parsed_services.push(srv_file);
            }
            "msg" => {
                let msg = parse_ros_message_file(&contents, name, &pkg, &path);
                parsed_messages.push(msg);
            }
            _ => {
                log::error!("File extension not recognized as a ROS file: {path:?}");
            }
        }
    }
    Ok((parsed_messages, parsed_services))
}

#[cfg(test)]
mod test {
    use crate::find_and_generate_ros_messages;

    /// Confirms we don't panic on ros1 parsing
    #[test]
    fn generate_ok_on_ros1() {
        let assets_path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../assets/ros1_common_interfaces"
        );

        let gen = find_and_generate_ros_messages(vec![assets_path.into()]);
        // Make sure something actually got generated
        assert!(!gen.is_empty())
    }

    /// Confirms we don't panic on ros2 parsing
    #[test]
    fn generate_ok_on_ros2() {
        let assets_path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../assets/ros2_common_interfaces"
        );

        let gen = find_and_generate_ros_messages(vec![assets_path.into()]);
        // Make sure something actually got generated
        assert!(!gen.is_empty())
    }

    /// Confirms we don't panic on ros1_test_msgs parsing
    #[test]
    #[cfg_attr(not(feature = "ros1_test"), ignore)]
    fn generate_ok_on_ros1_test_msgs() {
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/ros1_test_msgs");

        let gen = find_and_generate_ros_messages(vec![assets_path.into()]);
        assert!(!gen.is_empty());
    }

    /// Confirms we don't panic on ros2_test_msgs parsing
    #[test]
    #[cfg_attr(not(feature = "ros2_test"), ignore)]
    fn generate_ok_on_ros2_test_msgs() {
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/ros2_test_msgs");

        let gen = find_and_generate_ros_messages(vec![assets_path.into()]);
        assert!(!gen.is_empty());
    }
}
