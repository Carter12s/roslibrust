use std::collections::{BTreeMap, VecDeque};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

use codegen::Scope;
use log::*;

use super::util;

/*
Note: Dear god, I tried to use &str in these parse structs. It was painful...
 */

#[derive(PartialEq, Eq, Hash, Debug, Clone)]
pub struct FieldType {
    package_name: Option<String>,
    field_type: String,
    is_vec: bool,
}

#[derive(PartialEq, Debug)]
pub struct FieldInfo {
    field_type: FieldType,
    field_name: String,
}

#[derive(PartialEq, Debug)]
pub struct ConstantInfo {
    constant_type: String,
    constant_name: String,
    constant_value: String,
}

#[derive(PartialEq, Debug)]
pub struct MessageFile {
    name: String,
    fields: Vec<FieldInfo>,
    constants: Vec<ConstantInfo>,
}

/// Generates rust files (.rs) containing type representing the ros messages
pub fn generate_messages(files: Vec<PathBuf>, out_file: &Path) {
    // Parsing is implemented as iterative queue consumers
    // Multiple sequential queues Load Files -> Parse Files -> Resolve Remotes
    // Resolving remotes can add more more files to the load queue
    // Iteration is continued until load queue is depleted
    let mut files = VecDeque::from(files);
    let mut parsed: BTreeMap<String, MessageFile> = BTreeMap::new();
    let installed = util::get_installed_msgs();

    while let Some(entry) = files.pop_front() {
        let file = fs::read_to_string(&entry).expect("Could not read file.");
        let name = entry.file_stem().unwrap().to_str().unwrap().to_string();
        let parse_result = parse_ros_message_file(file, name.clone());

        // Resolve remotes
        // TODO could be nice iterator way of doing this
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

    let code = generate_rust(parsed);
    let mut file = std::fs::File::create(out_file).expect("Could not open output file.");
    file.write_all(code.as_bytes())
        .expect("Failed to write to file");
}

/// Converts a ros message file into a struct representation
fn parse_ros_message_file(data: String, name: String) -> MessageFile {
    let mut result = MessageFile {
        fields: vec![],
        constants: vec![],
        name,
    };
    for line in data.lines() {
        let line = strip_comments(line).trim();
        if line.len() == 0 {
            // Comment only line skip
            continue;
        }
        let equal_i = line.find('=');
        if let Some(_equal_i) = equal_i {
            warn!("Constants not supported yet: {} will be omitted", line);
            continue;
        }

        let mut splitter = line.split_whitespace();
        let field_type = splitter.next().expect("Did not find field_type on line.");
        let field_type = ros_type_to_rust(field_type);
        let field_name = splitter.next().expect("Did not find field_name on line.");
        if let Some(extra) = splitter.next() {
            panic!(
                "Extra data detected on line past field_name: {}\n source_line: {}",
                extra, line
            );
        }
        result.fields.push(FieldInfo {
            field_type,
            field_name: field_name.to_string(),
        });
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
pub fn generate_rust(msg_files: BTreeMap<String, MessageFile>) -> String {
    let mut scope = Scope::new();
    scope.import("serde", "{Deserialize,Serialize}");
    for (_name, file) in &msg_files {
        let struct_t = scope
            .new_struct(file.name.as_str())
            .derive("Deserialize")
            .derive("Serialize")
            .derive("Debug")
            .vis("pub");
        // TODO we could use doc to move comments from .msg files to rust code
        // TODO we could use package_name to group into modules (would then match name collision rules)
        // TODO we could detect constant naming convention and autogen enums?
        for field in &file.fields {
            if field.field_type.is_vec {
                struct_t.field(
                    field.field_name.as_str(),
                    format!("Vec<{}>" , &field.field_type.field_type ));
            } else {
                struct_t.field(field.field_name.as_str(), &field.field_type.field_type);
            }
        }

        // TODO disabled for now, feature pending on codegen lib
        // for constant in &file.constants {
        // https://github.com/carllerche/codegen/issues/27
        // struct_t.field(constant.constant_name, &constant.constant_type);
        // }
    }
    scope.to_string()
}

/// Basic mapping of ros type strings to rust types
pub fn ros_type_to_rust(t: &str) -> FieldType {
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
                "name".to_string()
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
                constants: vec![]
            }
        )
    }
}
