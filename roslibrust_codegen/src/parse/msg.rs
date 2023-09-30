use crate::parse::{parse_constant_field, parse_field, strip_comments};
use crate::{bail, Error};
use crate::{ConstantInfo, FieldInfo, Package, RosVersion};
use std::path::{Path, PathBuf};

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
) -> Result<ParsedMessageFile, Error> {
    let mut fields = vec![];
    let mut constants = vec![];

    for line in data.lines() {
        let line = strip_comments(line).trim();
        if line.is_empty() {
            // Comment only line skip
            continue;
        }
        // Determine if we're looking at a constant or a field
        let sep = line.find(' ').ok_or(
            Error::new(
                format!("Found an invalid ros field line, no space delinting type from name: {line} in {}\n{data}",
                path.display())
            )
        )?;
        let equal_after_sep = line[sep..].find('=');
        if equal_after_sep.is_some() {
            // Since we found an equal sign after a space, this must be a constant
            constants.push(parse_constant_field(line, package))
        } else {
            // Is regular field
            fields.push(parse_field(line, package, name));
        }
    }
    Ok(ParsedMessageFile {
        fields,
        constants,
        name: name.to_owned(),
        package: package.name.clone(),
        version: package.version,
        source: data.to_owned(),
        path: path.to_owned(),
    })
}
