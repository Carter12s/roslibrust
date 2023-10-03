use log::*;
use proc_macro2::TokenStream;
use quote::quote;
use serde::de::DeserializeOwned;
use serde::Serialize;
use simple_error::{bail, SimpleError as Error};
use std::collections::{BTreeMap, VecDeque};
use std::fmt::{Debug, Display};
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
    /// This field is optional, and only needed when using ros1 native communication
    const MD5SUM: &'static str = "";

    /// The definition from the msg, srv, or action file
    /// This field is optional, and only needed when using ros1 native communication
    const DEFINITION: &'static str = "";
}

// This special impl allows for services with no args / returns
impl RosMessageType for () {
    const ROS_TYPE_NAME: &'static str = "";
    const MD5SUM: &'static str = "";
    const DEFINITION: &'static str = "";
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
    pub(crate) is_fixed_length: bool,
}

impl MessageFile {
    fn resolve(parsed: ParsedMessageFile, graph: &BTreeMap<String, MessageFile>) -> Option<Self> {
        let md5sum = Self::compute_md5sum(&parsed, graph)?;
        let is_fixed_length = Self::determine_if_fixed_length(&parsed, graph)?;
        Some(MessageFile {
            parsed,
            md5sum,
            is_fixed_length,
        })
    }

    pub fn get_package_name(&self) -> String {
        self.parsed.package.clone()
    }

    pub fn get_short_name(&self) -> String {
        self.parsed.name.clone()
    }

    pub fn get_full_name(&self) -> String {
        format!("{}/{}", self.parsed.package, self.parsed.name)
    }

    pub fn get_md5sum(&self) -> &str {
        self.md5sum.as_str()
    }

    pub fn get_fields(&self) -> &[FieldInfo] {
        &self.parsed.fields
    }

    pub fn get_constants(&self) -> &[ConstantInfo] {
        &self.parsed.constants
    }

    pub fn is_fixed_length(&self) -> bool {
        self.is_fixed_length
    }

    pub fn get_definition(&self) -> &str {
        &self.parsed.source
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
                let field_package = field
                    .field_type
                    .package_name
                    .as_ref()
                    .expect(&format!("Expected package name for field {field:#?}"));
                let field_full_name = format!("{field_package}/{field_type}");
                let sub_message = graph.get(field_full_name.as_str())?;
                let sub_md5sum = Self::compute_md5sum(&sub_message.parsed, graph)?;
                md5sum_content.push_str(&format!("{} {}\n", sub_md5sum, field.field_name));
            }
        }

        Some(md5sum_content)
    }

    fn determine_if_fixed_length(
        parsed: &ParsedMessageFile,
        graph: &BTreeMap<String, MessageFile>,
    ) -> Option<bool> {
        for field in &parsed.fields {
            if matches!(field.field_type.array_info, Some(Some(_))) {
                return Some(true);
            } else if matches!(field.field_type.array_info, Some(None)) {
                return Some(false);
            }
            if field.field_type.package_name.is_none() {
                if field.field_type.field_type == "string" {
                    return Some(false);
                }
            } else {
                let field_msg = graph.get(field.get_full_name().as_str())?;
                let field_is_fixed_length =
                    Self::determine_if_fixed_length(&field_msg.parsed, graph)?;
                if !field_is_fixed_length {
                    return Some(false);
                }
            }
        }
        Some(true)
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
            MessageFile::resolve(parsed.request_type.clone(), graph),
            MessageFile::resolve(parsed.response_type.clone(), graph),
        ) {
            let md5sum = Self::compute_md5sum(&parsed, graph)?;
            Some(ServiceFile {
                parsed,
                request,
                response,
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

    pub fn get_short_name(&self) -> String {
        self.parsed.name.clone()
    }

    pub fn get_package_name(&self) -> String {
        self.parsed.package.clone()
    }

    pub fn request(&self) -> &MessageFile {
        &self.request
    }

    pub fn response(&self) -> &MessageFile {
        &self.response
    }

    pub fn get_md5sum(&self) -> String {
        self.md5sum.clone()
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

/// Stores the ROS string representation of a literal
#[derive(Clone, Debug)]
pub struct RosLiteral {
    pub inner: String,
}

impl Display for RosLiteral {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.inner, f)
    }
}

impl From<String> for RosLiteral {
    fn from(value: String) -> Self {
        Self { inner: value }
    }
}

/// Describes the type for an individual field in a message
#[derive(PartialEq, Eq, Hash, Debug, Clone)]
pub struct FieldType {
    // Present when an externally referenced package is used
    // Note: support for messages within same package is spotty...
    pub package_name: Option<String>,
    // Explicit text of type without array specifier
    pub field_type: String,
    // Metadata indicating whether the field is a collection.
    // Is Some(None) if it's an array type of variable size or Some(Some(N))
    // if it's an array type of fixed size.
    pub array_info: Option<Option<usize>>,
}

impl std::fmt::Display for FieldType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.array_info {
            Some(Some(n)) => f.write_fmt(format_args!("{}[{}]", self.field_type, n)),
            Some(None) => f.write_fmt(format_args!("{}[]", self.field_type)),
            None => f.write_fmt(format_args!("{}", self.field_type)),
        }
    }
}

/// Describes all information for an individual field
#[derive(Clone, Debug)]
pub struct FieldInfo {
    pub field_type: FieldType,
    pub field_name: String,
    // Exists if this is a ros2 message field with a default value
    pub default: Option<RosLiteral>,
}

// Because TokenStream doesn't impl PartialEq we have to do it manually for FieldInfo
impl PartialEq for FieldInfo {
    fn eq(&self, other: &Self) -> bool {
        self.field_type == other.field_type && self.field_name == other.field_name
        // && self.default == other.default
    }
}

impl FieldInfo {
    pub fn get_full_name(&self) -> String {
        let field_package = self
            .field_type
            .package_name
            .as_ref()
            .expect(&format!("Expected package name for field {self:#?}"));
        format!("{field_package}/{}", self.field_type.field_type)
    }
}

/// Describes all information for a constant within a message
/// Note: Constants are not fully supported yet (waiting on codegen support)
#[derive(Clone, Debug)]
pub struct ConstantInfo {
    pub constant_type: String,
    pub constant_name: String,
    pub constant_value: RosLiteral,
}

// Because TokenStream doesn't impl PartialEq we have to do it manually for ConstantInfo
impl PartialEq for ConstantInfo {
    fn eq(&self, other: &Self) -> bool {
        self.constant_type == other.constant_type && self.constant_name == other.constant_name
        // && self.constant_value == other.constant_value
    }
}

/// Searches a list of paths for ROS packages and generates struct definitions
/// and implementations for message files and service files in packages it finds.
///
/// * `additional_search_paths` - A list of additional paths to search beyond those
/// found in ROS_PACKAGE_PATH environment variable.
pub fn find_and_generate_ros_messages(
    additional_search_paths: Vec<PathBuf>,
) -> Result<TokenStream, Error> {
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
) -> Result<TokenStream, Error> {
    let (messages, services) = find_and_parse_ros_messages(&search_paths)?;

    if messages.is_empty() && services.is_empty() {
        // I'm considering this an error for now, but I could see this one being debateable
        // As it stands there is not good way for us to manually produce a warning, so I'd rather fail loud
        bail!("Failed to find any services or messages while generating ROS message definitions, paths searched: {search_paths:?}");
    }
    let (messages, services) = resolve_dependency_graph(messages, services)?;
    generate_rust_ros_message_definitions(messages, services)
}

/// Searches a list of paths for ROS packages to find their associated message
/// and service files, parsing and performing dependency resolution on those
/// it finds. Returns a map of PACKAGE_NAME/MESSAGE_NAME strings to message file
/// data and vector of service file data.
///
/// * `search_paths` - A list of paths to search.
///
pub fn find_and_parse_ros_messages(
    search_paths: &Vec<PathBuf>,
) -> Result<(Vec<ParsedMessageFile>, Vec<ParsedServiceFile>), Error> {
    let search_paths  = search_paths
        .into_iter()
        .map(|path| {
            path.canonicalize().map_err(
            |e| {
                    Error::with(format!("Codegen was instructed to search a path that could not be canonicalized relative to {:?}: {path:?}", std::env::current_dir().unwrap()).as_str(), e)
        })
        })
        .collect::<Result<Vec<_>, Error>>()?;
    debug!(
        "Codegen is looking in following paths for files: {:?}",
        &search_paths
    );
    let packages = utils::crawl(&search_paths);
    // Check for duplicate package names
    let packages = utils::deduplicate_packages(packages);
    if packages.is_empty() {
        bail!(
            "No ROS packages found while searching in: {search_paths:?}, relative to {:?}",
            std::env::current_dir().unwrap()
        );
    }

    let message_files = packages
        .iter()
        .flat_map(|pkg| {
            let files = utils::get_message_files(pkg).map_err(|err| {
                Error::with(
                    format!("Unable to get paths to message files for {pkg:?}:").as_str(),
                    err,
                )
            });
            // See https://stackoverflow.com/questions/59852161/how-to-handle-result-in-flat-map
            match files {
                Ok(files) => files
                    .into_iter()
                    .map(|path| Ok((pkg.clone(), path)))
                    .collect(),
                Err(e) => vec![Err(e)],
            }
        })
        .collect::<Result<Vec<(Package, PathBuf)>, Error>>()?;

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
) -> Result<TokenStream, Error> {
    let mut modules_to_struct_definitions: BTreeMap<String, Vec<TokenStream>> = BTreeMap::new();

    // Convert messages files into rust token streams and insert them into BTree organized by package
    messages
        .into_iter()
        .map(|message| {
            let pkg_name = message.parsed.package.clone();
            let definition = generate_struct(message)?;
            if let Some(entry) = modules_to_struct_definitions.get_mut(&pkg_name) {
                entry.push(definition);
            } else {
                modules_to_struct_definitions.insert(pkg_name, vec![definition]);
            }
            Ok(())
        })
        .collect::<Result<_, Error>>()?;
    // Do the same for services
    services
        .into_iter()
        .map(|service| {
            let pkg_name = service.parsed.package.clone();
            let definition = generate_service(service)?;
            if let Some(entry) = modules_to_struct_definitions.get_mut(&pkg_name) {
                entry.push(definition);
            } else {
                modules_to_struct_definitions.insert(pkg_name, vec![definition]);
            }
            Ok(())
        })
        .collect::<Result<_, Error>>()?;
    // Now generate modules to wrap all of the TokenStreams in a module for each package
    let all_pkgs = modules_to_struct_definitions
        .keys()
        .cloned()
        .collect::<Vec<String>>();
    let module_definitions = modules_to_struct_definitions
        .into_iter()
        .map(|(pkg, struct_defs)| generate_mod(pkg, struct_defs, &all_pkgs[..]))
        .collect::<Vec<_>>();

    Ok(quote! {
        #(#module_definitions)*

    })
}

struct MessageMetadata {
    msg: ParsedMessageFile,
    seen_count: u32,
}

pub fn resolve_dependency_graph(
    messages: Vec<ParsedMessageFile>,
    services: Vec<ParsedServiceFile>,
) -> Result<(Vec<MessageFile>, Vec<ServiceFile>), Error> {
    const MAX_PARSE_ITER_LIMIT: u32 = 2048;
    let mut unresolved_messages = messages
        .into_iter()
        .map(|msg| MessageMetadata { msg, seen_count: 0 })
        .collect::<VecDeque<_>>();

    let mut resolved_messages = BTreeMap::new();
    // First resolve the message dependencies
    while let Some(MessageMetadata { msg, seen_count }) = unresolved_messages.pop_front() {
        // Check our resolved messages for each of the fields
        let fully_resolved = msg.fields.iter().all(|field| {
            let is_ros1_primitive =
                ROS_TYPE_TO_RUST_TYPE_MAP.contains_key(field.field_type.field_type.as_str());
            let is_ros2_primitive =
                ROS_2_TYPE_TO_RUST_TYPE_MAP.contains_key(field.field_type.field_type.as_str());
            let is_primitive = is_ros1_primitive || is_ros2_primitive;
            if !is_primitive {
                let is_resolved = resolved_messages.contains_key(field.get_full_name().as_str());
                is_resolved
            } else {
                true
            }
        });

        if fully_resolved {
            let debug_name = msg.get_full_name();
            let msg_file = MessageFile::resolve(msg, &resolved_messages).ok_or(
                Error::new(format!("Failed to correctly resolve message {debug_name:?}, either md5sum could not be calculated, or fixed length was indeterminate"))
            )?;
            resolved_messages.insert(msg_file.get_full_name(), msg_file);
        } else {
            unresolved_messages.push_back(MessageMetadata {
                seen_count: seen_count + 1,
                msg,
            });
        }

        if seen_count > MAX_PARSE_ITER_LIMIT {
            let msg_names = unresolved_messages
                .iter()
                .map(|item| format!("{}/{}", item.msg.package, item.msg.name))
                .collect::<Vec<_>>();
            bail!("Unable to resolve dependencies after reaching search limit.\n\
                   The following messages have unresolved dependencies: {msg_names:?}\n\
                   These messages likely depend on packages not found in the provided search paths.");
        }
    }

    // Now that all messages are parsed, we can parse and resolve services
    let mut resolved_services: Vec<_> = services
        .into_iter()
        .filter_map(|srv| ServiceFile::resolve(srv, &resolved_messages))
        .collect();
    resolved_services.sort_by(|a, b| a.parsed.name.cmp(&b.parsed.name));

    Ok((resolved_messages.into_values().collect(), resolved_services))
}

/// Parses all ROS file types and returns a final expanded set
/// Currently supports service files, message files, and action files
/// The returned collection will contain all messages files including those buried with the
/// service or action files, and will have fully expanded and resolved referenced types in other packages.
/// * `msg_paths` -- List of tuple (Package, Path to File) for each file to parse
fn parse_ros_files(
    msg_paths: Vec<(Package, PathBuf)>,
) -> Result<(Vec<ParsedMessageFile>, Vec<ParsedServiceFile>), Error> {
    let mut parsed_messages = Vec::new();
    let mut parsed_services = Vec::new();
    for (pkg, path) in msg_paths {
        let contents = std::fs::read_to_string(&path).map_err(|e| {
            Error::with(
                format!("Codgen failed while attempting to read file {path:?} from disk:").as_str(),
                e,
            )
        })?;
        // Probably being overly aggressive with error shit here, but I'm on a kick
        let name = path
            .file_stem()
            .ok_or(Error::new(format!(
                "Failed to extract valid file stem for file at {path:?}"
            )))?
            .to_str()
            .ok_or(Error::new(format!(
                "File stem for file at path {path:?} was not valid unicode?"
            )))?;
        match path.extension().unwrap().to_str().unwrap() {
            "srv" => {
                let srv_file = parse_ros_service_file(&contents, name, &pkg, &path)?;
                parsed_services.push(srv_file);
            }
            "msg" => {
                let msg = parse_ros_message_file(&contents, name, &pkg, &path)?;
                parsed_messages.push(msg);
            }
            "action" => {
                let action = parse_ros_action_file(&contents, name, &pkg, &path)?;
                parsed_messages.push(action.action_type);
                parsed_messages.push(action.action_goal_type);
                parsed_messages.push(action.goal_type);
                parsed_messages.push(action.action_result_type);
                parsed_messages.push(action.result_type);
                parsed_messages.push(action.action_feedback_type);
                parsed_messages.push(action.feedback_type);
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
    #[test_log::test]
    fn generate_ok_on_ros1() {
        let assets_path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../assets/ros1_common_interfaces"
        );

        let gen = find_and_generate_ros_messages(vec![assets_path.into()]);
        // Make sure something actually got generated
        assert!(!gen.unwrap().is_empty())
    }

    /// Confirms we don't panic on ros2 parsing
    #[test_log::test]
    fn generate_ok_on_ros2() {
        let assets_path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../assets/ros2_common_interfaces"
        );

        let gen = find_and_generate_ros_messages(vec![assets_path.into()]);
        // Make sure something actually got generated
        assert!(!gen.unwrap().is_empty())
    }

    /// Confirms we don't panic on ros1_test_msgs parsing
    #[test_log::test]
    #[cfg_attr(not(feature = "ros1_test"), ignore)]
    fn generate_ok_on_ros1_test_msgs() {
        // Note: because our test msgs depend on std_message this test will fail unless ROS_PACKAGE_PATH includes std_msgs
        // To avoid that we add std_messsages to the extra paths.
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/ros1_test_msgs");
        let std_msgs = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../assets/ros1_common_interfaces/std_msgs"
        );
        let gen = find_and_generate_ros_messages(vec![assets_path.into(), std_msgs.into()]);
        assert!(!gen.unwrap().is_empty());
    }

    /// Confirms we don't panic on ros2_test_msgs parsing
    #[test_log::test]
    #[cfg_attr(not(feature = "ros2_test"), ignore)]
    fn generate_ok_on_ros2_test_msgs() {
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/ros2_test_msgs");

        let gen = find_and_generate_ros_messages(vec![assets_path.into()]);
        assert!(!gen.unwrap().is_empty());
    }
}
