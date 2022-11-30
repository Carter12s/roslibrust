use std::collections::HashMap;

use crate::utils::{Package, RosVersion};

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
        ("Header", "Header"),
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

/// Describes the type for an individual field in a message
#[derive(PartialEq, Eq, Hash, Debug, Clone)]
pub struct FieldType {
    // Present when an externally referenced package is used
    // Note: support for messages within same package is spotty...
    pub package_name: Option<String>,
    // Explicit text of type without array specifier
    pub field_type: String,
    // true iff "[]" or "[#]" are found
    // Note: no support for fixed size arrays yet
    pub is_vec: bool,
}

/// Describes all information for an individual field
#[derive(Clone, PartialEq, Debug)]
pub struct FieldInfo {
    pub field_type: FieldType,
    pub field_name: String,
}

/// Describes all information for a constant within a message
/// Note: Constants are not fully supported yet (waiting on codegen support)
#[derive(Clone, PartialEq, Debug)]
pub struct ConstantInfo {
    pub constant_type: String,
    pub constant_name: String,
    pub constant_value: String,
}

/// Describes all information for a single message file
#[derive(Clone, PartialEq, Debug)]
pub struct MessageFile {
    pub name: String,
    pub package: String,
    pub fields: Vec<FieldInfo>,
    pub constants: Vec<ConstantInfo>,
    pub version: Option<RosVersion>,
}

/// Describes all information for a single service file
pub struct ServiceFile {
    pub name: String,
    pub package: String,
    // The names of these types will be auto generated as {name}Request and {name}Response
    pub request_type: MessageFile,
    pub request_type_raw: String, // needed elsewhere in generation, but would like to remove
    pub response_type: MessageFile,
    pub response_type_raw: String, // needed elsewhere in generation, but would like to remove
}

/// Converts a ros message file into a struct representation
/// * `data` -- Raw contents of the file as loaded from disk
/// * `name` -- Name of the object being parsed excluding the file extension, e.g. `Header`
/// * `package` -- Name of the package the message is found in, required for relative type paths
/// * `ros2` -- True iff the package is a ros2 package and should be parsed with ros2 logic
pub fn parse_ros_message_file(data: String, name: String, package: &Package) -> MessageFile {
    let mut result = MessageFile {
        fields: vec![],
        constants: vec![],
        name: name.clone(),
        package: package.name.clone(),
        version: package.version,
    };

    for line in data.lines() {
        let line = strip_comments(line).trim();
        if line.len() == 0 {
            // Comment only line skip
            continue;
        }
        // Determine if we're looking at a constant or a field
        let sep = line.find(' ').expect(&format!(
            "Found an invalid ros field line, no space delinting type from name: {line}"
        ));
        let equal_after_sep = line[sep..].find("=");
        if let Some(equal_i) = equal_after_sep {
            // Since we found an equal sign after a space, this must be a constant
            let mut constant_type = parse_type(line[..sep].trim(), &package).field_type;
            let constant_name = line[sep + 1..(equal_i + sep)].trim().to_string();

            // Handle the fact that string type should be different for constants than fields
            if constant_type == "String" {
                constant_type = "&'static str".to_string();
            }

            let constant_value = line[sep + equal_i + 1..].trim().to_string();
            result.constants.push(ConstantInfo {
                // NOTE: Bug here, no idea how to support constant vectors etc...
                constant_type,
                constant_name,
                constant_value,
            })
        } else {
            // Is regular field
            let mut splitter = line.split_whitespace();
            let field_type = splitter.next().expect(&format!(
                "Did not find field_type on line: {line} while parsing {}/{name}",
                &package.name
            ));
            let field_type = parse_type(field_type, &package);
            let field_name = splitter.next().expect(&format!(
                "Did not find field_name on line: {line} while parsing {}/{name}",
                &package.name
            ));
            result.fields.push(FieldInfo {
                field_type,
                field_name: field_name.to_string(),
            });
        }
    }
    result
}

/// Parses the contents of a service file and returns and struct representing the found content.
/// * `data` -- Actual contents of the file
/// * `name` -- Name of the file excluding the extension, e.g. 'Header'
/// * `package` -- Name of the package the file was found within, required for understanding relative type paths
/// * `ros2` -- True iff this file is part of a ros2 package and should be parsed with ros2 logic
pub fn parse_ros_service_file(data: String, name: String, package: &Package) -> ServiceFile {
    let mut dash_line_number = None;
    for (line_num, line) in data.as_str().lines().enumerate() {
        match (line.find("---"), line.find("#")) {
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
        acc.push_str("\n");
        acc
    };

    let dash_line_number = dash_line_number.expect(
        format!(
            "Failed to find delimiter line '---' in {}/{name}",
            &package.name
        )
        .as_str(),
    );
    let request_str = data
        .as_str()
        .lines()
        .take(dash_line_number)
        .fold(String::new(), str_accumulator);
    let response_str = data
        .as_str()
        .lines()
        .skip(dash_line_number + 1)
        .fold(String::new(), str_accumulator);

    ServiceFile {
        name: name.clone(),
        package: package.name.clone(),
        request_type: parse_ros_message_file(
            request_str.clone(),
            format!("{name}Request"),
            package,
        ),
        request_type_raw: request_str,
        response_type: parse_ros_message_file(
            response_str.clone(),
            format!("{name}Response"),
            package,
        ),
        response_type_raw: response_str,
    }
}

pub fn replace_ros_types_with_rust_types(mut msg: MessageFile) -> MessageFile {
    const INTERNAL_STD_MSGS: [&str; 1] = ["Header"];

    // Select which type conversion map to use depending on ros version
    let prop_map: &HashMap<&'static str, &'static str> = match msg.version {
        Some(RosVersion::ROS1) => &ROS_TYPE_TO_RUST_TYPE_MAP,
        Some(RosVersion::ROS2) => &ROS_2_TYPE_TO_RUST_TYPE_MAP,
        None => {
            // If we couldn't determine the package type, assume ROS1 for now
            &ROS_TYPE_TO_RUST_TYPE_MAP
        }
    };

    msg.constants = msg
        .constants
        .into_iter()
        .map(|mut constant| {
            if prop_map.contains_key(constant.constant_type.as_str()) {
                constant.constant_type = prop_map
                    .get(constant.constant_type.as_str())
                    .unwrap()
                    .to_string();
                // We do not need to consider the package for constants as they're required
                // to be built-in types other than Time and Duration (I think Header is not
                // technically built-in)
            }
            constant
        })
        .collect();
    msg.fields = msg
        .fields
        .into_iter()
        .map(|mut field| {
            field.field_type.field_type = prop_map
                .get(field.field_type.field_type.as_str())
                .unwrap_or(&field.field_type.field_type.as_str())
                .to_string();
            for std_msg in INTERNAL_STD_MSGS {
                if field.field_type.field_type.as_str() == std_msg {
                    field.field_type.package_name = Some("std_msgs".into());
                }
            }
            field
        })
        .collect();
    msg
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

    // Select which type converison map to use depending on package version
    let prop_map: &HashMap<&'static str, &'static str> = match pkg.version {
        Some(RosVersion::ROS1) => &ROS_TYPE_TO_RUST_TYPE_MAP,
        Some(RosVersion::ROS2) => &ROS_2_TYPE_TO_RUST_TYPE_MAP,
        None => {
            // If we couldn't determine the package type, assume ROS1 for now
            &ROS_TYPE_TO_RUST_TYPE_MAP
        }
    };

    if items.len() == 1 {
        // If there is only one item (no package redirect)
        FieldType {
            package_name: if prop_map.contains_key(type_str) {
                // If it is a fundamental type, no package
                None
            } else {
                // Otherwise it is referencing another message in the same package
                Some(pkg.name.clone())
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
    let open_bracket_idx = type_str.find("[");
    let close_bracket_idx = type_str.find("]");
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
