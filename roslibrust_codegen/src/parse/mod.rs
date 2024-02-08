use crate::utils::{Package, RosVersion};
use crate::{bail, Error};
use crate::{ConstantInfo, FieldInfo, FieldType};
use std::collections::HashMap;

mod action;
pub use action::{parse_ros_action_file, ParsedActionFile};
mod msg;
pub use msg::{parse_ros_message_file, ParsedMessageFile};
mod srv;
pub use srv::{parse_ros_service_file, ParsedServiceFile};

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

fn parse_field(line: &str, pkg: &Package, msg_name: &str) -> Result<FieldInfo, Error> {
    let mut splitter = line.split_whitespace();
    let pkg_name = pkg.name.as_str();
    let field_type = splitter.next().ok_or(Error::new(format!(
        "Did not find field_type on line: {line} while parsing {pkg_name}/{msg_name}"
    )))?;
    let field_type = parse_type(field_type, pkg)?;
    let field_name = splitter.next().ok_or(Error::new(format!(
        "Did not find field_name on line: {line} while parsing {pkg_name}/{msg_name}"
    )))?;

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

    Ok(FieldInfo {
        field_type,
        field_name: field_name.to_string(),
        default,
    })
}

fn parse_constant_field(line: &str, pkg: &Package) -> Result<ConstantInfo, Error> {
    let sep = line.find(' ').ok_or(
        Error::new(format!("Failed to find white space seperator ' ' while parsing constant information one line {line} for package {pkg:?}"))
    )?;
    let equal_after_sep = line[sep..].find('=').ok_or(
        Error::new(format!("Failed to find expected '=' while parsing constant information on line {line} for package {pkg:?}"))
    )?;
    let mut constant_type = parse_type(line[..sep].trim(), pkg)?.field_type;
    let constant_name = line[sep + 1..(equal_after_sep + sep)].trim().to_string();

    // Handle the fact that string type should be different for constants than fields
    if constant_type == "String" {
        constant_type = "&'static str".to_string();
    }

    let constant_value = line[sep + equal_after_sep + 1..].trim().to_string();
    Ok(ConstantInfo {
        constant_type,
        constant_name,
        constant_value: constant_value.into(),
    })
}

/// Looks for # comment character and sub-slices for characters preceding it
fn strip_comments(line: &str) -> &str {
    if let Some(token) = line.find('#') {
        return &line[..token];
    }
    line
}

//TODO it is a little scary that this function appears infallible?
fn parse_field_type(type_str: &str, array_info: Option<Option<usize>>, pkg: &Package) -> FieldType {
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
            array_info,
        }
    } else {
        // If there is more than one item there is a package redirect

        // Special workaround for builtin_interfaces package
        if items[0] == "builtin_interfaces" {
            FieldType {
                package_name: None,
                field_type: type_str.to_string(),
                array_info,
            }
        } else {
            FieldType {
                package_name: Some(items[0].to_string()),
                field_type: items[1].to_string(),
                array_info,
            }
        }
    }
}

/// Determines the type of a field
/// `type_str` -- Expects the part of the line containing all type information (up to the first space), e.g. "int32[3>=]"
/// `pkg` -- Reference to package this type is within, used for version information and determining relative types
fn parse_type(type_str: &str, pkg: &Package) -> Result<FieldType, Error> {
    // Handle array logic
    let open_bracket_idx = type_str.find('[');
    let close_bracket_idx = type_str.find(']');
    match (open_bracket_idx, close_bracket_idx) {
        (Some(o), Some(c)) => {
            // After having stripped array information, parse the remainder of the type
            let array_size = if c - o == 1 {
                // No size specified
                None
            } else {
                let fixed_size_str = &type_str[(o + 1)..c];
                let fixed_size = fixed_size_str.parse::<usize>().map_err(|err| {
                    Error::new(format!(
                        "Unable to parse size of the array: {type_str}, defaulting to 0: {err}"
                    ))
                });
                // TODO we don't currently handle "array limits" in ROS2, so for now we're ejecting this error
                // To make this function complete we need to handle clauses like '<=3'
                // None of this really matters at current time, because we don't generate fixed size array types yet anyway
                let fixed_size = fixed_size.unwrap_or(0);
                Some(fixed_size)
            };
            Ok(parse_field_type(&type_str[..o], Some(array_size), pkg))
        }
        (None, None) => {
            // Not an array parse normally
            Ok(parse_field_type(type_str, None, pkg))
        }
        _ => {
            bail!("Found malformed type: {type_str} in package {pkg:?}. Likely file is invalid.");
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{
        parse::parse_type,
        utils::{Package, RosVersion},
    };

    // Simple test to just confirm fixed size logic is working correctly on the parse side
    #[test_log::test]
    fn parse_type_handles_fixed_size_correctly() {
        let line = "int32[9]";
        let pkg = Package {
            name: "test_pkg".to_string(),
            path: "./not_a_path".into(),
            version: Some(RosVersion::ROS1),
        };
        let parsed = parse_type(line, &pkg).unwrap();
        assert_eq!(parsed.array_info, Some(Some(9)));
    }
}
