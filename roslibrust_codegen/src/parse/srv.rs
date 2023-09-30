use crate::{
    bail,
    parse::{parse_ros_message_file, ParsedMessageFile},
    Error, Package,
};
use std::path::{Path, PathBuf};

/// Describes all information for a single service file
#[derive(Clone, Debug)]
pub struct ParsedServiceFile {
    pub name: String,
    pub package: String,
    // The names of these types will be auto generated as {name}Request and {name}Response
    pub request_type: ParsedMessageFile,
    pub response_type: ParsedMessageFile,
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
) -> Result<ParsedServiceFile, Error> {
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

    let dash_line_number = dash_line_number.ok_or(Error::new(format!(
        "Failed to find delimiter line '---' in {}/{name}",
        &package.name
    )))?;
    let request_str = data
        .lines()
        .take(dash_line_number)
        .fold(String::new(), str_accumulator);
    let response_str = data
        .lines()
        .skip(dash_line_number + 1)
        .fold(String::new(), str_accumulator);

    Ok(ParsedServiceFile {
        name: name.to_owned(),
        package: package.name.clone(),
        request_type: parse_ros_message_file(
            &request_str,
            format!("{name}Request").as_str(),
            package,
            path,
        )?,
        response_type: parse_ros_message_file(
            &response_str,
            format!("{name}Response").as_str(),
            package,
            path,
        )?,
        source: data.to_owned(),
        path: path.to_owned(),
    })
}
