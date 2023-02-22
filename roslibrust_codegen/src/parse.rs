use crate::utils::{Package, RosVersion};
use crate::{ConstantInfo, FieldInfo, FieldType};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

lazy_static::lazy_static! {
    pub static ref ROS_TYPE_TO_RUST_TYPE_MAP: HashMap<&'static str, &'static str> = vec![
        ("bool", "bool"),
        ("int8", "i8"),
        ("uint8", "u8"),
        ("byte", "u8"),
        ("char", "u8"), // NOTE: a rust char != C++ char
        ("int16", "i16"),
        ("uint16", "u16"),
        ("int32", "i32"),
        ("uint32", "u32"),
        ("int64", "i64"),
        ("uint64", "u64"),
        ("float32", "f32"),
        ("float64", "f64"),
        ("string", "::std::string::String"),
        ("time", "::roslibrust_codegen::integral_types::Time"),
        ("duration", "::roslibrust_codegen::integral_types::Duration"),
    ].into_iter().collect();

    pub static ref ROS_2_TYPE_TO_RUST_TYPE_MAP: HashMap<&'static str, &'static str> = vec![
        ("bool", "bool"),
        ("int8", "i8"),
        ("uint8", "u8"),
        ("byte", "u8"),
        ("char", "u8"),
        ("int16", "i16"),
        ("uint16", "u16"),
        ("int32", "i32"),
        ("uint32", "u32"),
        ("int64", "i64"),
        ("uint64", "u64"),
        ("float32", "f32"),
        ("float64", "f64"),
        ("string", "::std::string::String"),
        ("builtin_interfaces/Time", "::roslibrust_codegen::integral_types::Time"),
        ("builtin_interfaces/Duration", "::roslibrust_codegen::integral_types::Duration"),
        // ("wstring", TODO),
    ].into_iter().collect();
}

pub fn is_intrinsic_type(version: RosVersion, ros_type: &str) -> bool {
    match version {
        RosVersion::ROS1 => ROS_TYPE_TO_RUST_TYPE_MAP.contains_key(ros_type),
        RosVersion::ROS2 => ROS_2_TYPE_TO_RUST_TYPE_MAP.contains_key(ros_type),
    }
}

pub fn convert_ros_type_to_rust_type(version: RosVersion, ros_type: &str) -> Option<&'static str> {
    match version {
        RosVersion::ROS1 => ROS_TYPE_TO_RUST_TYPE_MAP.get(ros_type).copied(),
        RosVersion::ROS2 => ROS_2_TYPE_TO_RUST_TYPE_MAP.get(ros_type).copied(),
    }
}

/// Describes all information for a single message file
#[derive(Clone, PartialEq, Debug)]
pub struct ParsedMessageFile {
    pub name: String,
    pub package: String,
    pub fields: Vec<FieldInfo>,
    pub constants: Vec<ConstantInfo>,
    pub version: Option<RosVersion>,
    /// The contents of the message file this instance was parsed from
    pub source: String,
    /// The path where the message was found
    pub path: PathBuf,
}

impl ParsedMessageFile {
    pub fn has_header(&self) -> bool {
        self.fields.iter().any(|field| {
            field.field_type.field_type.as_str() == "Header"
                && (field.field_type.package_name.is_none()
                    || field.field_type.package_name == Some(String::from("std_msgs")))
        })
    }

    pub fn get_full_name(&self) -> String {
        format!("{}/{}", self.package, self.name)
    }
}

/// Describes all information for a single service file
#[derive(Clone, Debug)]
pub struct ParsedServiceFile {
    pub name: String,
    pub package: String,
    // The names of these types will be auto generated as {name}Request and {name}Response
    pub request_type: ParsedMessageFile,
    pub request_type_raw: String, // needed elsewhere in generation, but would like to remove
    pub response_type: ParsedMessageFile,
    pub response_type_raw: String, // needed elsewhere in generation, but would like to remove
    /// The contents of the service file this instance was parsed from
    pub source: String,
    /// The path where the message was found
    pub path: PathBuf,
}

impl ParsedServiceFile {
    pub fn get_full_name(&self) -> String {
        format!("{}/{}", self.package, self.name)
    }
}

fn parse_field(line: &str, pkg: &Package, msg_name: &str) -> FieldInfo {
    let mut splitter = line.split_whitespace();
    let pkg_name = pkg.name.as_str();
    let field_type = splitter.next().unwrap_or_else(|| {
        panic!("Did not find field_type on line: {line} while parsing {pkg_name}/{msg_name}")
    });
    let field_type = parse_type(field_type, pkg);
    let field_name = splitter.next().unwrap_or_else(|| {
        panic!("Did not find field_name on line: {line} while parsing {pkg_name}/{msg_name}")
    });

    let sep = line.find(' ').unwrap();
    // Determine if there is a default value for this field
    let default = if matches!(pkg.version, Some(RosVersion::ROS2)) {
        // For ros2 packages only, check if there is a default value
        let line_after_sep = line[sep + 1..].trim();
        match line_after_sep.find(' ') {
            Some(def_start) => {
                let remainder = line_after_sep[def_start..].trim();
                if remainder.is_empty() {
                    None
                } else {
                    Some(remainder.to_owned().into())
                }
            }
            None => {
                // No extra space separator found, not default was provided
                None
            }
        }
    } else {
        None
    };

    FieldInfo {
        field_type,
        field_name: field_name.to_string(),
        default,
    }
}

fn parse_constant_field(line: &str, pkg: &Package) -> ConstantInfo {
    let sep = line.find(' ').unwrap();
    let equal_after_sep = line[sep..].find('=').unwrap();
    let mut constant_type = parse_type(line[..sep].trim(), pkg).field_type;
    let constant_name = line[sep + 1..(equal_after_sep + sep)].trim().to_string();

    // Handle the fact that string type should be different for constants than fields
    if constant_type == "String" {
        constant_type = "&'static str".to_string();
    }

    let constant_value = line[sep + equal_after_sep + 1..].trim().to_string();
    ConstantInfo {
        constant_type,
        constant_name,
        constant_value: constant_value.into(),
    }
}

/// Converts a ros message file into a struct representation
/// * `data` -- Raw contents of the file as loaded from disk
/// * `name` -- Name of the object being parsed excluding the file extension, e.g. `Header`
/// * `package` -- Name of the package the message is found in, required for relative type paths
/// * `ros2` -- True iff the package is a ros2 package and should be parsed with ros2 logic
pub fn parse_ros_message_file(
    data: &str,
    name: &str,
    package: &Package,
    path: &Path,
) -> ParsedMessageFile {
    let mut fields = vec![];
    let mut constants = vec![];

    for line in data.lines() {
        let line = strip_comments(line).trim();
        if line.is_empty() {
            // Comment only line skip
            continue;
        }
        // Determine if we're looking at a constant or a field
        let sep = line.find(' ').unwrap_or_else(|| {
            panic!("Found an invalid ros field line, no space delinting type from name: {line}")
        });
        let equal_after_sep = line[sep..].find('=');
        if equal_after_sep.is_some() {
            // Since we found an equal sign after a space, this must be a constant
            constants.push(parse_constant_field(line, package))
        } else {
            // Is regular field
            fields.push(parse_field(line, package, name));
        }
    }
    ParsedMessageFile {
        fields,
        constants,
        name: name.to_owned(),
        package: package.name.clone(),
        version: package.version,
        source: data.to_owned(),
        path: path.to_owned(),
    }
}

/// Parses the contents of a service file and returns and struct representing the found content.
/// * `data` -- Actual contents of the file
/// * `name` -- Name of the file excluding the extension, e.g. 'Header'
/// * `package` -- Name of the package the file was found within, required for understanding relative type paths
/// * `ros2` -- True iff this file is part of a ros2 package and should be parsed with ros2 logic
pub fn parse_ros_service_file(
    data: &str,
    name: &str,
    package: &Package,
    path: &Path,
) -> ParsedServiceFile {
    let mut dash_line_number = None;
    for (line_num, line) in data.lines().enumerate() {
        match (line.find("---"), line.find('#')) {
            (Some(dash_idx), Some(cmt_idx)) => {
                if dash_idx < cmt_idx {
                    // Comment appears after dash
                    dash_line_number = Some(line_num);
                    break;
                }
            }
            (Some(_), None) => {
                dash_line_number = Some(line_num);
                break;
            }
            _ => continue,
        }
    }
    let str_accumulator = |mut acc: String, line: &str| -> String {
        acc.push_str(line);
        acc.push('\n');
        acc
    };

    let dash_line_number = dash_line_number.unwrap_or_else(|| {
        panic!(
            "Failed to find delimiter line '---' in {}/{name}",
            &package.name
        )
    });
    let request_str = data
        .lines()
        .take(dash_line_number)
        .fold(String::new(), str_accumulator);
    let response_str = data
        .lines()
        .skip(dash_line_number + 1)
        .fold(String::new(), str_accumulator);

    ParsedServiceFile {
        name: name.to_owned(),
        package: package.name.clone(),
        request_type: parse_ros_message_file(
            request_str.clone().as_str(),
            format!("{name}Request").as_str(),
            package,
            path,
        ),
        request_type_raw: request_str,
        response_type: parse_ros_message_file(
            response_str.clone().as_str(),
            format!("{name}Response").as_str(),
            package,
            path,
        ),
        response_type_raw: response_str,
        source: data.to_owned(),
        path: path.to_owned(),
    }
}

/// Looks for # comment character and sub-slices for characters preceding it
fn strip_comments(line: &str) -> &str {
    if let Some(token) = line.find('#') {
        return &line[..token];
    }
    line
}

fn parse_field_type(type_str: &str, is_vec: bool, pkg: &Package) -> FieldType {
    let items = type_str.split('/').collect::<Vec<&str>>();

    if items.len() == 1 {
        // If there is only one item (no package redirect)
        let pkg_version = pkg.version.unwrap_or(RosVersion::ROS1);
        FieldType {
            package_name: if is_intrinsic_type(pkg_version, type_str) {
                // If it is a fundamental type, no package
                None
            } else {
                // Otherwise it is referencing another message in the same package
                if type_str == "Header" {
                    Some("std_msgs".to_owned())
                } else {
                    Some(pkg.name.clone())
                }
            },
            field_type: items[0].to_string(),
            is_vec,
        }
    } else {
        // If there is more than one item there is a package redirect

        // Special workaround for builtin_interfaces package
        if items[0] == "builtin_interfaces" {
            FieldType {
                package_name: None,
                field_type: type_str.to_string(),
                is_vec,
            }
        } else {
            FieldType {
                package_name: Some(items[0].to_string()),
                field_type: items[1].to_string(),
                is_vec,
            }
        }
    }
}

/// Determines the type of a field
/// `type_str` -- Expects the part of the line containing all type information (up to the first space), e.g. "int32[3>=]"
/// `pkg` -- Reference to package this type is within, used for version information and determining relative types
fn parse_type(type_str: &str, pkg: &Package) -> FieldType {
    // Handle array logic
    let open_bracket_idx = type_str.find('[');
    let close_bracket_idx = type_str.find(']');
    match (open_bracket_idx, close_bracket_idx) {
        (Some(o), Some(_c)) => {
            // After having stripped array information, parse the remainder of the type
            parse_field_type(&type_str[..o], true, pkg)
        }
        (None, None) => {
            // Not an array parse normally
            parse_field_type(type_str, false, pkg)
        }
        _ => {
            panic!("Found malformed type: {type_str}");
        }
    }
}
