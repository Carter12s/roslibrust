use std::collections::{BTreeMap, VecDeque};
use std::fs;
use std::path::PathBuf;

use anyhow::bail;
use codegen::Scope;
use log::{debug, warn};

use super::util;

/*
Note: Dear god, I tried to use &str in these parse structs. It was painful...
TODO revisit, we could make message_gen more efficient with less copies if everything was references
 */

/// Specify how resolution of std_msgs should be handled during generation
pub enum MsgSource {
    /// Use std message definitions bundled with this library, this is for use in environments
    /// without ROS installed
    Internal,
    /// Try to find std_msgs using ROS_PACKAGE_PATH, this is the default method
    /// Generation will fail if messages are not found.
    Environment,
}

/// Configurable options for message generation following the builder pattern
pub struct MessageGenOpts {
    format: bool,
    targets: Vec<PathBuf>,
    msg_source: MsgSource,
    destination: Option<PathBuf>,
    default_package: Option<String>,
    local: bool,
}

impl MessageGenOpts {
    /// Creates a new message generation builder.
    ///
    /// ## Example
    /// ```
    /// use roslibrust::message_gen::{MessageGenOpts, generate_messages_str};
    ///
    /// let targets = vec!["my_file.msg"];
    /// let opts = MessageGenOpts::new(targets);
    /// let _ = generate_messages_str(&opts);
    /// ```
    pub fn new<T: Into<PathBuf>>(targets: Vec<T>) -> MessageGenOpts {
        MessageGenOpts {
            format: true,
            targets: targets.into_iter().map(|t| t.into()).collect(),
            msg_source: MsgSource::Environment,
            local: false,
            destination: None,
            default_package: None,
        }
    }

    /// Indicates whether or `rustfmt` should be invoked on the result of generation
    /// `rustfmt` must be installed and executable on the host to use this option.
    /// Ignored when generating to a string.
    pub fn format(mut self, format: bool) -> MessageGenOpts {
        self.format = format;
        self
    }

    /// Which source of std_msgs definitions should be used (useful for generation without ros installed)
    /// If set to INTERNAL, generation uses std_msgs that are shipped with this crate;
    /// otherwise, std_msgs are looked for on the host system
    ///
    /// Roadmap:
    ///   - Better documentation on what messages we ship with, it is more than std_msgs right now
    pub fn std_msgs_source(mut self, source: MsgSource) -> MessageGenOpts {
        self.msg_source = source;
        self
    }

    /// Files to run generation against, all files referenced in a single generation call will be
    /// bundled into a single output file. Multiple calls to generation can be made if separate
    /// files are desired
    pub fn targets<T: Into<PathBuf>>(mut self, targets: Vec<T>) -> MessageGenOpts {
        self.targets = targets.into_iter().map(|t| t.into()).collect();
        self
    }

    // TODO this being optional is slightly awkward... Consider different form...
    /// Destination file to generate to.
    /// Ignored if generating to file.
    pub fn destination<T: Into<PathBuf>>(mut self, destination: T) -> MessageGenOpts {
        self.destination = Some(destination.into());
        self
    }

    //TODO this is a hack and should be removed, but I need it for now
    /// Defines a default package name to use when package discovery fails
    /// When given a file we walk up the directory structure looking for package.xml
    /// However always having a package.xml can be problematic, and it is nice to be able to generate
    /// on raw message files without them being in a package, so this lets you override that.
    pub fn default_package<S: AsRef<str>>(mut self, package_name: S) -> MessageGenOpts {
        self.default_package = Some(package_name.as_ref().to_string());
        self
    }

    /// Only used for internal testing with the crate
    /// Generated file will use 'crate' instead of 'roslibrust' when referring to types defined
    /// in this package
    //TODO find a better mechanism for this
    pub fn local(mut self, local: bool) -> MessageGenOpts {
        self.local = local;
        self
    }
}

/// Describes the type for an individual field in a message
#[derive(PartialEq, Eq, Hash, Debug, Clone)]
pub(crate) struct FieldType {
    // Present when an externally referenced package is used
    // Note: support for messages within same package is spotty...
    package_name: Option<String>,
    // Explicit text of type without array specifier
    field_type: String,
    // true iff "[]" or "[#]" are found
    // Note: no support for fixed size arrays yet
    is_vec: bool,
}

/// Describes all information for an individual field
#[derive(PartialEq, Debug)]
pub(crate) struct FieldInfo {
    field_type: FieldType,
    field_name: String,
}

/// Describes all information for a constant within a message
/// Note: Constants are not fully supported yet (waiting on codegen support)
#[derive(PartialEq, Debug)]
pub(crate) struct ConstantInfo {
    constant_type: String,
    constant_name: String,
    constant_value: String,
}

/// Describes all information for a single message file
#[derive(PartialEq, Debug)]
pub(crate) struct MessageFile {
    name: String,
    package: String,
    fields: Vec<FieldInfo>,
    constants: Vec<ConstantInfo>,
}

/// Generates rust files (.rs) containing type representing the ros messages
/// @param local Used to indicate if generate file should reference roslibrust as 'crate'
pub fn generate_messages(opts: &MessageGenOpts) -> Result<(), Box<dyn std::error::Error>> {
    let data = generate_messages_str(opts)?;
    let out_file = opts
        .destination
        .as_ref()
        .expect("Attempted to generate to a file without specifying an output file");
    std::fs::write(out_file, data)?;
    if opts.format {
        std::process::Command::new("rustfmt")
            .arg(&out_file)
            .output()?;
    }
    Ok(())
}

/// Generates rust type code for convenient access to messages
/// returns the generated code as a string
// TODO better error handling here
pub fn generate_messages_str(opts: &MessageGenOpts) -> Result<String, anyhow::Error> {
    // Parsing is implemented as iterative queue consumers
    // Multiple sequential queues Load Files -> Parse Files -> Resolve Remotes
    // Resolving remotes can add more more files to the load queue
    // Iteration is continued until load queue is depleted
    let mut files = VecDeque::from(opts.targets.clone());
    let mut parsed: BTreeMap<String, MessageFile> = BTreeMap::new();
    let installed = match opts.msg_source {
        MsgSource::Environment => match util::get_installed_msgs() {
            Ok(installed) => {
                debug!("Using msgs from ROS_PACKAGE_PATH");
                debug!("Found {} installed msgs", installed.len());
                let mut installed = installed.clone();
                installed.append(&mut util::get_special_msgs());
                installed
            }
            Err(e) => {
                warn!("Failed to find installed ros messages: {:?}", e);
                vec![]
            }
        },
        MsgSource::Internal => {
            debug!("Using locally installed msgs");
            let msgs = util::get_local_msgs();
            debug!("Found {} locally installed msgs", msgs.len());
            msgs
        }
    };

    while let Some(entry) = files.pop_front() {
        debug!("Processing {:?}", entry);
        let file = fs::read_to_string(&entry)?;
        let name = entry.file_stem().unwrap().to_str().unwrap().to_string();
        let package = match util::find_package_from_path(&entry) {
            // Use an empty package name if we couldn't find one
            // TODO this is likely a brittle hack that will break some use cases
            None => {
                debug!("Failed to find package for file: {:?} using default", &file);
                if let Some(name) = &opts.default_package {
                    name.clone()
                } else {
                    "".to_string()
                }
            }
            Some(s) => s,
        };
        let parse_results = match entry.extension().unwrap().to_str().unwrap() {
            "srv" => parse_ros_srv_file(file, &name, &package),
            "msg" => {
                vec![parse_ros_message_file(file, name.clone(), package)]
            }
            _ => {
                bail!("Unregcognized file extension: {:?}", entry);
            }
        };

        // Resolve remotes
        // TODO could be nice iterator way of doing this
        for parse_result in parse_results.into_iter() {
            'per_file: for field in &parse_result.fields {
                if field.field_type.package_name.is_none() {
                    // Not a remote reference
                    continue;
                }
                let package_name = field.field_type.package_name.as_ref().unwrap();
                if parsed.contains_key(&field.field_type.field_type) {
                    // Reference already resolved
                    continue;
                }
                // Attempt to lookup reference in discovered installed files:
                for candidate in &installed {
                    if &candidate.package_name == package_name {
                        if &candidate
                            .path
                            .file_stem()
                            .unwrap()
                            .to_string_lossy()
                            .to_string()
                            == &field.field_type.field_type
                        {
                            // We've resolved the reference, add it to files that need to be resolved
                            files.push_back(candidate.path.clone());
                            continue 'per_file;
                        }
                    }
                }
                panic!("Failed to resolve reference for: {:?}", field);
            }
            // Done processing move it away
            parsed.insert(name.clone(), parse_result);
        }
    }

    let code = generate_rust(parsed, opts.local);
    Ok(code)
}

fn parse_ros_srv_file(data: String, name: &str, package: &str) -> Vec<MessageFile> {
    let index = data.find("---").expect(&format!(
        "Failed to find delimeter line '---' in {} name: {}",
        &data, &name
    ));
    let req_str = data[..index].to_string();
    let res_str = data[index + 3..].to_string();
    vec![
        parse_ros_message_file(req_str, format!("{}Request", &name), package.to_string()),
        parse_ros_message_file(res_str, format!("{}Response", &name), package.to_string()),
    ]
}

/// Converts a ros message file into a struct representation
fn parse_ros_message_file(data: String, name: String, package: String) -> MessageFile {
    let mut result = MessageFile {
        fields: vec![],
        constants: vec![],
        name,
        package,
    };
    for line in data.lines() {
        let line = strip_comments(line).trim();
        if line.len() == 0 {
            // Comment only line skip
            continue;
        }

        let equal_i = line.find('=');
        if let Some(equal_i) = equal_i {
            let sep = line.find(' ').unwrap();
            let mut constant_type = ros_type_to_rust(line[..sep].trim()).field_type;
            let constant_name = line[sep + 1..equal_i].trim().to_string();
            // Handling dumb case here, TODO find better spot
            if constant_type == "String" {
                constant_type = "&'static str".to_string();
            }

            let constant_value = line[equal_i + 1..].trim().to_string();
            // Is a constant
            result.constants.push(ConstantInfo {
                // NOTE: Bug here, no idea how to support constant vectors etc...
                constant_type,
                constant_name,
                constant_value,
            })
        } else {
            // Is regular field
            let mut splitter = line.split_whitespace();
            let field_type = splitter
                .next()
                .expect(&format!("Did not find field_type on line: {}", &line));
            let field_type = ros_type_to_rust(field_type);
            let field_name = splitter
                .next()
                .expect(&format!("Did not find field_name on line: {}", &line));
            result.fields.push(FieldInfo {
                field_type,
                field_name: field_name.to_string(),
            });
        }
    }
    result
}

/// Looks for # comment character and sub-slices for characters preceding it
fn strip_comments(line: &str) -> &str {
    if let Some(token) = line.find('#') {
        return &line[..token];
    }
    line
}

/// From a list of struct representations of ros msg files, generates a resulting rust module as
/// a string.
// TODO should this be public?
fn generate_rust(msg_files: BTreeMap<String, MessageFile>, local: bool) -> String {
    let mut scope = Scope::new();

    // Required imports
    scope.import("serde", "{Deserialize,Serialize}");
    if !local {
        scope.import("roslibrust", "RosMessageType");
    } else {
        scope.import("crate", "RosMessageType");
    }

    // For each message type
    for (_name, file) in &msg_files {
        let struct_t = scope
            .new_struct(file.name.as_str())
            .derive("Deserialize")
            .derive("Serialize")
            .derive("Debug")
            .derive("Default")
            .derive("Clone")
            .derive("PartialEq")
            .vis("pub");
        // TODO we could use doc to move comments from .msg files to rust code
        // TODO we could use package_name to group into modules (would then match name collision rules)
        // TODO we could detect constant naming convention and autogen enums?
        for field in &file.fields {
            if field.field_type.is_vec {
                let mut f = codegen::Field::new(
                    field.field_name.as_str(),
                    format!("Vec<{}>", &field.field_type.field_type),
                );
                // TODO I don't think annotations should be used for this...
                // Crate not offering me an alternative yet...
                f.annotation(vec!["pub"]);
                struct_t.push_field(f);
            } else {
                let mut f =
                    codegen::Field::new(field.field_name.as_str(), &field.field_type.field_type);
                f.annotation(vec!["pub"]);
                struct_t.push_field(f);
            }
        }

        // Implement ros message type
        let imp = scope.new_impl(file.name.as_str());
        imp.impl_trait("RosMessageType");
        imp.associate_const(
            "ROS_TYPE_NAME",
            "&'static str",
            &["\"", &file.package, "/", &file.name, "\""].join(""),
            "",
        );

        // Implement associated constants
        // TODO: Enums?
        if file.constants.len() > 0 {
            let imp = scope.new_impl(file.name.as_str());
            for constant in &file.constants {
                imp.associate_const(
                    constant.constant_name.as_str(),
                    constant.constant_type.clone(),
                    constant.constant_value.as_str(),
                    "pub",
                );
            }
        }
    }
    scope.to_string()
}

/// Basic mapping of ros type strings to rust types
fn ros_type_to_rust(t: &str) -> FieldType {
    let arr_index = t.find("[");
    let is_vec = arr_index.is_some();
    let i = arr_index.unwrap_or(t.len());
    let t = &t[..i];
    match t {
        "bool" => FieldType {
            package_name: None,
            field_type: "bool".to_string(),
            is_vec,
        },
        "int8" => FieldType {
            package_name: None,
            field_type: "i8".to_string(),
            is_vec,
        },
        "uint8" => FieldType {
            package_name: None,
            field_type: "u8".to_string(),
            is_vec,
        },
        "int16" => FieldType {
            package_name: None,
            field_type: "i16".to_string(),
            is_vec,
        },
        "uint16" => FieldType {
            package_name: None,
            field_type: "u16".to_string(),
            is_vec,
        },
        "int32" => FieldType {
            package_name: None,
            field_type: "i32".to_string(),
            is_vec,
        },
        "uint32" => FieldType {
            package_name: None,
            field_type: "u32".to_string(),
            is_vec,
        },
        "int64" => FieldType {
            package_name: None,
            field_type: "i64".to_string(),
            is_vec,
        },
        "uint64" => FieldType {
            package_name: None,
            field_type: "u64".to_string(),
            is_vec,
        },
        "float32" => FieldType {
            package_name: None,
            field_type: "f32".to_string(),
            is_vec,
        },
        "float64" => FieldType {
            package_name: None,
            field_type: "f64".to_string(),
            is_vec,
        },
        "string" => FieldType {
            package_name: None,
            field_type: "String".to_string(),
            is_vec,
        },
        "time" => FieldType {
            package_name: Some("std_msgs".into()),
            field_type: "TimeI".into(),
            is_vec,
        },
        "duration" => FieldType {
            package_name: Some("std_msgs".into()),
            field_type: "DurationI".into(),
            is_vec,
        },
        "Header" => FieldType {
            package_name: Some("std_msgs".into()),
            field_type: "Header".into(),
            is_vec,
        },
        _ => {
            // Reference to another package
            let tokens: Vec<&str> = t.split("/").collect();
            if tokens.len() < 2 {
                // Local reference, directly grab type
                FieldType {
                    // TODO should place local package here. This is a bug!
                    package_name: None,
                    field_type: tokens[0].to_string(),
                    is_vec,
                }
            } else {
                FieldType {
                    package_name: Some(tokens[0].to_string()),
                    field_type: tokens[1].to_string(),
                    is_vec,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strip_comments_test() {
        assert_eq!(strip_comments("#"), "");
        assert_eq!(strip_comments("some content#"), "some content");
        assert_eq!(strip_comments("ablbl # asdf ###alsdkj\naslkdj #"), "ablbl ");
    }

    #[test]
    fn parse_ros_message_file_test() {
        assert_eq!(
            parse_ros_message_file(
                r"
            # Other comment
            uint32 name # Some Comment
            time[3] field
            # Some comment"
                    .to_string(),
                "name".to_string(),
                "package".to_string()
            ),
            MessageFile {
                fields: vec![
                    FieldInfo {
                        field_type: FieldType {
                            package_name: None,
                            field_type: "u32".to_string(),
                            is_vec: false
                        },
                        field_name: "name".to_string()
                    },
                    FieldInfo {
                        field_type: FieldType {
                            package_name: Some("std_msgs".to_string()),
                            field_type: "TimeI".to_string(),
                            is_vec: true
                        },
                        field_name: "field".to_string()
                    },
                ],
                name: "name".to_string(),
                constants: vec![],
                package: "package".to_string(),
            }
        )
    }
}
