use log::*;
use proc_macro2::TokenStream;
use quote::quote;
use simple_error::{bail, SimpleError as Error};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
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

// These pub use statements are here to be able to export the dependencies of the generated code
// so that crates using this crate don't need to add these dependencies themselves.
// Our generated code should find these exports.
// Modeled from: https://users.rust-lang.org/t/proc-macros-using-third-party-crate/42465/4
pub use ::serde;
pub use serde::{de::DeserializeOwned, Deserialize, Serialize};
pub use serde_big_array::BigArray; // Used in generated code for large fixed sized arrays
pub use serde_bytes;
pub use smart_default::SmartDefault; // Used in generated code for default values // Used in generated code for faster Vec<u8> serialization

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
pub trait RosServiceType: 'static + Send + Sync {
    /// Name of the ros service e.g. `rospy_tutorials/AddTwoInts`
    const ROS_SERVICE_NAME: &'static str;
    /// The computed md5sum of the message file and its dependencies
    const MD5SUM: &'static str;
    /// The type of data being sent in the request
    type Request: RosMessageType;
    /// The type of the data
    type Response: RosMessageType;
}

/// Taking in a message definition
/// reformat it according to the md5sum algorithm: <https://wiki.ros.org/ROS/Technical%20Overview#Message_serialization_and_msg_MD5_sums>
/// - Comments removed
/// - Extra whitespace removed
/// - package names of dependencies removed
/// - constants reordered to be at the front
fn clean_msg(msg: &str) -> String {
    let mut result_params = vec![];
    let mut result_constants = vec![];
    for line in msg.lines() {
        let line = line.trim();
        // Skip comment lines
        if line.starts_with('#') {
            continue;
        }
        // Strip comment from the end of the line (if present)
        let line = line.split('#').collect::<Vec<&str>>()[0].trim();
        // Remove any extra whitespace from inside the line
        let line = line.split_whitespace().collect::<Vec<&str>>().join(" ");
        // Remove any whitespace on either side of the "=" for constants
        let line = line.replace(" = ", "=");
        // Skip any empty lines
        if line.is_empty() {
            continue;
        }
        // Determine if constant or not
        if line.contains('=') {
            result_constants.push(line);
        } else {
            result_params.push(line);
        }
    }
    format!(
        "{}\n{}",
        result_constants.join("\n"),
        result_params.join("\n")
    )
    .trim()
    .to_string() // Last trim here is lazy, but gets job done
}

// TODO(lucasw) this deserves a lot of str vs String cleanup
// TODO(lucasw) the msg_type isn't actually needed - Carter (it actually is, or we need to know the package name at least)
/// This function will calculate the md5sum of an expanded message definition.
/// The expanded message definition is the output of `gendeps --cat` see: <https://wiki.ros.org/roslib/gentools>
/// This definition is typically sent in the connection header of a ros1 topic and is also stored in bag files.
/// This can be used to calculate the md5sum when message definitions aren't available at compile time.
pub fn message_definition_to_md5sum(msg_name: &str, full_def: &str) -> Result<String, Error> {
    if full_def.is_empty() {
        return Err("empty input definition".into());
    }

    // Split the full definition into sections per message
    let sep: &str =
        "================================================================================\n";
    let sections = full_def.split(sep).collect::<Vec<&str>>();
    if sections.is_empty() {
        // Carter: this error is impossible, split only gives empty iterator when input string is empty
        // which we've already checked for above
        return Err("empty sections".into());
    }

    // Split the overall definition into seperate sub-messages sorted by message type (incluidng package name)
    let mut sub_messages: HashMap<&str, String> = HashMap::new();
    // Note: the first section doesn't contain the "MSG: <type>" line so we don't need to strip it here
    let clean_root = clean_msg(sections[0]);
    if clean_root.is_empty() {
        return Err("empty cleaned root definition".into());
    }
    sub_messages.insert(msg_name, clean_root);

    for section in &sections[1..] {
        let line0 = section.lines().next().ok_or("empty section")?;
        if !line0.starts_with("MSG: ") {
            return Err("bad section {section} -> {line0} doesn't start with 'MSG: '".into());
        }
        // TODO(lucasw) the full text definition doesn't always have the full message types with
        // the package name,
        // but I think this is only when the message type is Header or the package of the message
        // being define is the same as the message in the field
        // Carter: I agree with this, we found the same when dealing with this previously
        let section_type = line0.split_whitespace().collect::<Vec<&str>>()[1];
        let end_of_first_line = section.find('\n').ok_or("No body found in section")?;
        let body = clean_msg(&section[end_of_first_line + 1..]);
        sub_messages.insert(section_type, body);
    }

    // TODO MAJOR(carter): I'd like to convert this loop to a recursive function where we pass in the map of hashes
    // and update them as we go, this tripple loop is stinky to my eye.
    // TODO(carter) we should be able to do this in close to one pass if we iterate the full_def backwards
    let mut hashed = HashMap::new();
    let hash = message_definition_to_md5sum_recursive(msg_name, &sub_messages, &mut hashed)?;

    Ok(hash)
}

/// Calculates the hash of the specified message type by recursively calling itself on all dependencies
/// Uses defs as the list of message definitions available for it (expects them to already be cleaned)
/// Uses hashes as the cache of already calculated hashes so we don't redo work
fn message_definition_to_md5sum_recursive(
    msg_type: &str,
    defs: &HashMap<&str, String>,
    hashes: &mut HashMap<String, String>,
) -> Result<String, Error> {
    let base_types: HashSet<String> = HashSet::from_iter(
        [
            "bool", "byte", "int8", "int16", "int32", "int64", "uint8", "uint16", "uint32",
            "uint64", "float32", "float64", "time", "duration", "string",
        ]
        .map(|name| name.to_string()),
    );
    let def = defs.get(msg_type).ok_or(simple_error::simple_error!(
        "Couldn't find message type: {msg_type}"
    ))?;
    let pkg_name = msg_type.split('/').collect::<Vec<&str>>()[0];
    // We'll store the expanded hash definition in this string as we go
    let mut field_def = "".to_string();
    for line_raw in def.lines() {
        let line_split = line_raw.split_whitespace().collect::<Vec<&str>>();
        if line_split.len() < 2 {
            log::error!("bad line to split '{line_raw}'");
            // TODO(lucasw) or error out
            continue;
        }
        let (raw_field_type, _field_name) = (line_split[0], line_split[1]);
        // leave array characters alone, could be [] [C] where C is a constant
        let field_type = raw_field_type.split('[').collect::<Vec<&str>>()[0].to_string();

        let full_field_type;
        let line;
        if base_types.contains(&field_type) {
            line = line_raw.to_string();
        } else {
            // TODO(lucasw) are there other special message types besides header- or is it anything in std_msgs?
            if field_type == "Header" {
                full_field_type = "std_msgs/Header".to_string();
            } else if !field_type.contains('/') {
                full_field_type = format!("{pkg_name}/{field_type}");
            } else {
                full_field_type = field_type;
            }

            match hashes.get(&full_field_type) {
                Some(hash_value) => {
                    // Hash already exists in cache so we can use it
                    line = line_raw.replace(raw_field_type, hash_value).to_string();
                }
                None => {
                    // Recurse! To calculate hash of this field type
                    let hash =
                        message_definition_to_md5sum_recursive(&full_field_type, defs, hashes)?;
                    line = line_raw.replace(raw_field_type, &hash).to_string();
                }
            }
        }
        field_def += &format!("{line}\n");
    }
    field_def = field_def.trim().to_string();
    let md5sum = md5::compute(field_def.trim_end().as_bytes());
    let md5sum_text = format!("{md5sum:x}");
    // Insert our hash into the cache before we return
    hashes.insert(msg_type.to_string(), md5sum_text.clone());

    Ok(md5sum_text)
}

#[derive(Clone, Debug)]
pub struct MessageFile {
    pub(crate) parsed: ParsedMessageFile,
    pub(crate) md5sum: String,
    // This is the expanded definition of the message for use in message_definition field of
    // a connection header.
    // See how https://wiki.ros.org/ROS/TCPROS references gendeps --cat
    // See https://wiki.ros.org/roslib/gentools for an example of the output
    pub(crate) definition: String,
    pub(crate) is_fixed_length: bool,
}

impl MessageFile {
    fn resolve(parsed: ParsedMessageFile, graph: &BTreeMap<String, MessageFile>) -> Option<Self> {
        let md5sum = Self::compute_md5sum(&parsed, graph)?;
        let definition = Self::compute_full_definition(&parsed, graph)?;
        let is_fixed_length = Self::determine_if_fixed_length(&parsed, graph)?;
        Some(MessageFile {
            parsed,
            md5sum,
            definition,
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
        &self.definition
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

    /// Returns the set of all referenced non-intrinsic field types in this type or any of its dependencies
    fn get_unique_field_types(
        parsed: &ParsedMessageFile,
        graph: &BTreeMap<String, MessageFile>,
    ) -> Option<BTreeSet<String>> {
        let mut unique_field_types = BTreeSet::new();
        for field in &parsed.fields {
            let field_type = field.field_type.field_type.as_str();
            if is_intrinsic_type(parsed.version.unwrap_or(RosVersion::ROS1), field_type) {
                continue;
            }
            let sub_message = graph.get(field.get_full_name().as_str())?;
            // Note: need to add both the field that is referenced AND its sub-dependencies
            unique_field_types.insert(field.get_full_name());
            let mut sub_deps = Self::get_unique_field_types(&sub_message.parsed, graph)?;
            unique_field_types.append(&mut sub_deps);
        }
        Some(unique_field_types)
    }

    /// Computes the full definition of the message, including all referenced custom types
    /// For reference see: https://wiki.ros.org/roslib/gentools
    /// Implementation in gentools: https://github.com/strawlab/ros/blob/c3a8785f9d9551cc05cd74000c6536e2244bb1b1/core/roslib/src/roslib/gentools.py#L245
    fn compute_full_definition(
        parsed: &ParsedMessageFile,
        graph: &BTreeMap<String, MessageFile>,
    ) -> Option<String> {
        let mut definition_content = String::new();
        definition_content.push_str(&format!("{}\n", parsed.source.trim()));
        let sep: &str =
            "================================================================================\n";
        for field in Self::get_unique_field_types(parsed, graph)? {
            let Some(sub_message) = graph.get(&field) else {
                log::error!(
                    "Unable to find message type: {field:?}, while computing full definition of {}",
                    parsed.get_full_name()
                );
                return None;
            };
            definition_content.push_str(sep);
            definition_content.push_str(&format!("MSG: {}\n", sub_message.get_full_name()));
            definition_content.push_str(&format!("{}\n", sub_message.get_definition().trim()));
        }
        // Remove trailing \n added by concatenation logic
        definition_content.pop();
        Some(definition_content)
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
    pub package_name: Option<String>,
    // Redundantly store the name of the package the field is in
    // This is so that when an external package_name is not present
    // we can still construct the full name of the field "package/field_type"
    pub source_package: String,
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
            .unwrap_or(&self.field_type.source_package);
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
/// Returns a tuple of the generated source code and list of file system paths that if
/// modified would trigger re-generation of the source. This function is designed to
/// be used either in a build.rs file or via the roslibrust_codegen_macro crate.
/// * `additional_search_paths` - A list of additional paths to search beyond those
/// found in ROS_PACKAGE_PATH environment variable.
pub fn find_and_generate_ros_messages(
    additional_search_paths: Vec<PathBuf>,
) -> Result<(TokenStream, Vec<PathBuf>), Error> {
    let mut ros_package_paths = utils::get_search_paths();
    ros_package_paths.extend(additional_search_paths);
    find_and_generate_ros_messages_without_ros_package_path(ros_package_paths)
}

/// Searches a list of paths for ROS packages and generates struct definitions
/// and implementations for message files and service files in packages it finds.
/// Returns a tuple of the generated source code and list of file system paths that if
/// modified would trigger re-generation of the source. This function is designed to
/// be used either in a build.rs file or via the roslibrust_codegen_macro crate.
///
/// * `search_paths` - A list of paths to search for ROS packages.
pub fn find_and_generate_ros_messages_without_ros_package_path(
    search_paths: Vec<PathBuf>,
) -> Result<(TokenStream, Vec<PathBuf>), Error> {
    let (messages, services, actions) = find_and_parse_ros_messages(&search_paths)?;
    if messages.is_empty() && services.is_empty() {
        // I'm considering this an error for now, but I could see this one being debateable
        // As it stands there is not good way for us to manually produce a warning, so I'd rather fail loud
        bail!("Failed to find any services or messages while generating ROS message definitions, paths searched: {search_paths:?}");
    }
    tokenize_messages_and_services(messages, services, actions)
}

/// Generates source code and list of depnendent file system paths
fn tokenize_messages_and_services(
    messages: Vec<ParsedMessageFile>,
    services: Vec<ParsedServiceFile>,
    actions: Vec<ParsedActionFile>,
) -> Result<(TokenStream, Vec<PathBuf>), Error> {
    let (messages, services) = resolve_dependency_graph(messages, services)?;
    let msg_iter = messages.iter().map(|m| m.parsed.path.clone());
    let srv_iter = services.iter().map(|s| s.parsed.path.clone());
    let action_iter = actions.iter().map(|a| a.path.clone());
    let dependent_paths = msg_iter.chain(srv_iter).chain(action_iter).collect();
    let source = generate_rust_ros_message_definitions(messages, services)?;
    Ok((source, dependent_paths))
}

/// Generates struct definitions and implementations for message and service files
/// in the given packages.
pub fn generate_ros_messages_for_packages(
    packages: Vec<Package>,
) -> Result<(TokenStream, Vec<PathBuf>), Error> {
    let msg_paths = packages
        .iter()
        .flat_map(|package| {
            utils::get_message_files(&package).map(|msgs| {
                msgs.into_iter()
                    .map(|msg| (package.clone(), msg))
                    .collect::<Vec<_>>()
            })
        })
        .flatten()
        .collect();
    let (messages, services, actions) = parse_ros_files(msg_paths)?;
    if messages.is_empty() && services.is_empty() {
        bail!("Failed to find any services or messages while generating ROS message definitions, packages searched: {packages:?}")
    }
    tokenize_messages_and_services(messages, services, actions)
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
) -> Result<
    (
        Vec<ParsedMessageFile>,
        Vec<ParsedServiceFile>,
        Vec<ParsedActionFile>,
    ),
    Error,
> {
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
        .collect::<Result<(), Error>>()?;
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
        .collect::<Result<(), Error>>()?;
    // Now generate modules to wrap all of the TokenStreams in a module for each package
    let all_pkgs = modules_to_struct_definitions
        .keys()
        .cloned()
        .collect::<Vec<String>>();
    let module_definitions = modules_to_struct_definitions
        .into_iter()
        .map(|(pkg, struct_defs)| generate_mod(pkg, struct_defs, &all_pkgs[..]))
        .collect::<Vec<TokenStream>>();

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
        .map(|srv| {
            let name = srv.path.clone();
            ServiceFile::resolve(srv, &resolved_messages).ok_or(Error::new(format!(
                "Failed to correctly resolve service: {:?}",
                &name
            )))
        })
        .collect::<Result<Vec<_>, Error>>()?;
    resolved_services.sort_by(|a: &ServiceFile, b: &ServiceFile| a.parsed.name.cmp(&b.parsed.name));

    Ok((resolved_messages.into_values().collect(), resolved_services))
}

/// Parses all ROS file types and returns a final expanded set
/// Currently supports service files, message files, and action files
/// The returned collection will contain all messages files including those buried with the
/// service or action files, and will have fully expanded and resolved referenced types in other packages.
/// * `msg_paths` -- List of tuple (Package, Path to File) for each file to parse
fn parse_ros_files(
    msg_paths: Vec<(Package, PathBuf)>,
) -> Result<
    (
        Vec<ParsedMessageFile>,
        Vec<ParsedServiceFile>,
        Vec<ParsedActionFile>,
    ),
    Error,
> {
    let mut parsed_messages = Vec::new();
    let mut parsed_services = Vec::new();
    let mut parsed_actions = Vec::new();
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
                parsed_actions.push(action.clone());
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
    Ok((parsed_messages, parsed_services, parsed_actions))
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

        let (source, paths) = find_and_generate_ros_messages(vec![assets_path.into()]).unwrap();
        // Make sure something actually got generated
        assert!(!source.is_empty());
        // Make sure we have some paths
        assert!(!paths.is_empty());
    }

    /// Confirms we don't panic on ros2 parsing
    #[test_log::test]
    fn generate_ok_on_ros2() {
        let assets_path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../assets/ros2_common_interfaces"
        );

        let (source, paths) = find_and_generate_ros_messages(vec![assets_path.into()]).unwrap();
        // Make sure something actually got generated
        assert!(!source.is_empty());
        // Make sure we have some paths
        assert!(!paths.is_empty());
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
        let (source, paths) =
            find_and_generate_ros_messages(vec![assets_path.into(), std_msgs.into()]).unwrap();
        assert!(!source.is_empty());
        // Make sure we have some paths
        assert!(!paths.is_empty());
    }

    /// Confirms we don't panic on ros2_test_msgs parsing
    #[test_log::test]
    #[cfg_attr(not(feature = "ros2_test"), ignore)]
    fn generate_ok_on_ros2_test_msgs() {
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/ros2_test_msgs");

        let (source, paths) = find_and_generate_ros_messages(vec![assets_path.into()]).unwrap();
        assert!(!source.is_empty());
        assert!(!paths.is_empty());
    }

    /// Confirm md5sum from the connection header message definition matches normally
    /// generated checksums
    #[test_log::test]
    fn msg_def_to_md5() {
        {
            let def = "byte DEBUG=1\nbyte INFO=2\nbyte WARN=4\nbyte ERROR=8\nbyte FATAL=16\n\
                2176decaecbce78abc3b96ef049fabed header\n\
                byte level\nstring name\nstring msg\nstring file\nstring function\nuint32 line\nstring[] topics";
            let expected = "acffd30cd6b6de30f120938c17c593fb";
            let md5sum = format!("{:x}", md5::compute(def.trim_end().as_bytes()));
            assert_eq!(md5sum, expected, "partially checksumed rosgraph_msgs/Log");
        }

        {
            let msg_type = "bad_msgs/Empty";
            let def = "";
            let _md5sum =
                crate::message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            let msg_type = "bad_msgs/CommentSpacesOnly";
            let def =
                "# message with only comments and whitespace\n# another line comment\n\n    \n";
            let _md5sum =
                crate::message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            let msg_type = "fake_msgs/MissingSectionMsg";
            let def = "string name\nstring msg\n================================================================================\n# message with only comments and whitespace\n# another line comment\n\n    \n";
            let _md5sum =
                crate::message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            let msg_type = "bad_msgs/BadLog";
            let def = "##
## Severity level constants
byte DEUG=1 #debug level
byte FATAL=16 #fatal/critical level
##
## Fields
##
Header header
byte level
string name # name of the node
uint32 line # line the message came from
string[] topics # topic names that the node publishes

================================================================================
MSG: std_msgs/badHeader
# Standard metadata for higher-level stamped data types.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
time stamp
";
            let _md5sum =
                crate::message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            // TODO(lucasw) not sure if this is an ok message, currently it passes
            let expected = "96c44a027b586ee888fe95ac325151ae";
            let msg_type = "fake_msgs/CommentSpacesOnlySection";
            let def = "string name\nstring msg\n================================================================================\nMSG: foo/bar\n# message with only comments and whitespace\n# another line comment\n\n    \n";
            let md5sum = crate::message_definition_to_md5sum(msg_type.into(), def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "fake_msgs/Garbage";
            let def = r#"
fsdajklf

==                   #fdjkl

MSG:    jklfd
# 
================================================================================
f

vjk
"#;
            let _md5sum =
                crate::message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        // TODO(lucasw) it would be nice to pull these out of the real messages, but to avoid
        // dependencies just storing expected definition and md5sum
        // from roslib.message import get_message_class
        // msg = get_message_class("std_msgs/Header")
        // md5 = msg._md5sum
        // def = msg._full_text
        //

        {
            let msg_type = "sensor_msgs/CameraInfo";
            // This definition contains double quotes, so representing it with r# and newlines which is nicer
            // for limited width text editing anyhow
            let def = r#"
# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficient.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] P # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify

"#;
            let expected = "c9a58c1b0b154e0e6da7578cb991d214";
            let md5sum = crate::message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "std_msgs/Header";
            let def = "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
            let expected = "2176decaecbce78abc3b96ef049fabed";
            let md5sum = crate::message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "rosgraph_msgs/Log";
            let def = "##\n## Severity level constants\n##\nbyte DEBUG=1 #debug level\nbyte INFO=2  #general level\nbyte WARN=4  #warning level\nbyte ERROR=8 #error level\nbyte FATAL=16 #fatal/critical level\n##\n## Fields\n##\nHeader header\nbyte level\nstring name # name of the node\nstring msg # message \nstring file # file the message came from\nstring function # function the message came from\nuint32 line # line the message came from\nstring[] topics # topic names that the node publishes\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
            let expected = "acffd30cd6b6de30f120938c17c593fb";
            let md5sum = crate::message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "nav_msgs/Odometry";
            let def = "# This represents an estimate of a position and velocity in free space.  \n# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n# The twist in this message should be specified in the coordinate frame given by the child_frame_id\nHeader header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n================================================================================\nMSG: geometry_msgs/TwistWithCovariance\n# This expresses velocity in free space with uncertainty.\n\nTwist twist\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
            let expected = "cd5e73d190d741a2f92e81eda573aca7";
            let md5sum = crate::message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected);
        }

        {
            let msg_type = "tf2_msgs/TFMessage";
            let def = r#"
geometry_msgs/TransformStamped[] transforms

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href="http://wiki.ros.org/tf">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"#;
            let expected = "94810edda583a504dfda3829e70d7eec";
            let md5sum = crate::message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected);
        }

        {
            let msg_type = "vision_msgs/Detection3DArray";
            let def = r#"
# A list of 3D detections, for a multi-object 3D detector.

Header header

# A list of the detected proposals. A multi-proposal detector might generate
#   this list with many candidate detections generated from a single input.
Detection3D[] detections

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: vision_msgs/Detection3D
# Defines a 3D detection result.
#
# This extends a basic 3D classification by including position information,
#   allowing a classification result for a specific position in an image to
#   to be located in the larger image.

Header header

# Class probabilities. Does not have to include hypotheses for all possible
#   object ids, the scores for any ids not listed are assumed to be 0.
ObjectHypothesisWithPose[] results

# 3D bounding box surrounding the object.
BoundingBox3D bbox

# The 3D data that generated these results (i.e. region proposal cropped out of
#   the image). This information is not required for all detectors, so it may
#   be empty.
sensor_msgs/PointCloud2 source_cloud

================================================================================
MSG: vision_msgs/ObjectHypothesisWithPose
# An object hypothesis that contains position information.

# The unique numeric ID of object detected. To get additional information about
#   this ID, such as its human-readable name, listeners should perform a lookup
#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
int64 id

# The probability or confidence value of the detected object. By convention,
#   this value should lie in the range [0-1].
float64 score

# The 6D pose of the object hypothesis. This pose should be
#   defined as the pose of some fixed reference point on the object, such a
#   the geometric center of the bounding box or the center of mass of the
#   object.
# Note that this pose is not stamped; frame information can be defined by
#   parent messages.
# Also note that different classes predicted for the same input data may have
#   different predicted 6D poses.
geometry_msgs/PoseWithCovariance pose
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: vision_msgs/BoundingBox3D
# A 3D bounding box that can be positioned and rotated about its center (6 DOF)
# Dimensions of this box are in meters, and as such, it may be migrated to
#   another package, such as geometry_msgs, in the future.

# The 3D position and orientation of the bounding box center
geometry_msgs/Pose center

# The size of the bounding box, in meters, surrounding the object's center
#   pose.
geometry_msgs/Vector3 size

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

"#;
            let expected = "05c51d9aea1fb4cfdc8effb94f197b6f";
            let md5sum = crate::message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }
    }

    // Basic test of clean_msg function
    #[test]
    fn clean_msg_test() {
        let test_msg = r#"
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6 # Random Comment

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field


uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

"#;
        let result = crate::clean_msg(test_msg);
        let expected = r#"uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count"#;
        assert_eq!(result, expected);
    }
}
