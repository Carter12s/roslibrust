use std::collections::HashSet;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

use codegen::{Field, Scope};
use log::*;

use super::util;

#[derive(PartialEq, Eq, Hash, Debug, Clone)]
pub struct FieldType {
    package_name: Option<String>,
    field_type: String,
}

#[derive(PartialEq, Debug)]
pub struct FieldInfo<'a> {
    field_type: FieldType,
    field_name: &'a str,
}

#[derive(PartialEq, Debug)]
pub struct ConstantInfo<'a> {
    constant_type: String,
    constant_name: &'a str,
    constant_value: &'a str,
}

#[derive(PartialEq, Debug)]
pub struct MessageFile<'a> {
    name: &'a str,
    fields: Vec<FieldInfo<'a>>,
    constants: Vec<ConstantInfo<'a>>,
}

/// Generates rust files (.rs) containing type representing the ros messages
pub fn generate_messages(files: &Vec<PathBuf>, out_file: &Path) {
    // Load files
    let mut raw_data: Vec<(String, String)> = vec![];
    for entry in files {
        raw_data.push((
            fs::read_to_string(entry).expect("Could not read file."),
            entry.file_stem().unwrap().to_str().unwrap().to_string(),
        ));
    }

    // Parse into tokens
    let mut parsed: Vec<MessageFile> = vec![];
    for (file, name) in &raw_data {
        let parse_result = parse_ros_message_file(file, name);
        parsed.push(parse_result);
    }

    // Resolve references to other files
    // TODO could be nice iterator way of doing this, but i'mma be lazy
    let mut remotes = HashSet::<FieldType>::new();
    for f in &parsed {
        for field in &f.fields {
            if let Some(p) = &field.field_type.package_name {
                remotes.insert(field.field_type.clone());
            }
        }
    }

    // Attempt to locate references is ROS_PACKAGE_PATH
    let installed = util::get_installed_msgs();
    let mut used_remotes = vec![];
    for remote in &remotes {
        for candidate in &installed {
            if &candidate.package_name == remote.package_name.as_ref().unwrap() {
                if candidate
                    .path
                    .file_stem()
                    .unwrap()
                    .to_string_lossy()
                    .to_string()
                    == remote.field_type
                {
                    used_remotes.push(candidate);
                }
            }
        }
    }

    let mut remote_data = vec![];
    for remote in used_remotes {
        remote_data.push((
            fs::read_to_string(&remote.path).expect("Could not read file."),
            remote
                .path
                .file_stem()
                .unwrap()
                .to_string_lossy()
                .to_string(),
        ));
    }
    for (file, name) in &remote_data {
        let parse_result = parse_ros_message_file(file, name);
        parsed.push(parse_result);
    }

    let code = generate_rust(parsed);
    let mut file = std::fs::File::create(out_file).expect("Could not open output file.");
    file.write_all(code.as_bytes())
        .expect("Failed to write to file");
}

/// Converts a ros message file into a struct representation
fn parse_ros_message_file<'a>(data: &'a str, name: &'a str) -> MessageFile<'a> {
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

        let mut splitter = line.split(' ');
        let field_type = splitter.next().expect("Did not find field_type on line.");
        let field_type = ros_type_to_rust(field_type);
        let field_name = splitter.next().expect("Did not find field_name on line.");
        if let Some(extra) = splitter.next() {
            panic!("Extra data detected on line past field_name: {}", extra);
        }
        result.fields.push(FieldInfo {
            field_type,
            field_name,
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
pub fn generate_rust(msg_file: Vec<MessageFile>) -> String {
    let mut scope = Scope::new();
    scope.import("serde", "{Deserialize,Serialize}");
    for file in &msg_file {
        let struct_t = scope
            .new_struct(file.name)
            .derive("Deserialize")
            .derive("Serialize")
            .derive("Debug")
            .vis("pub");
        // TODO we could use doc to move comments from .msg files to rust code
        for field in &file.fields {
            struct_t.field(field.field_name, &field.field_type.field_type);
        }
        // TODO we could detect constant naming convention and autogen enums?
        // for constant in &file.constants {
        // TODO disabled for now, feature pending on codegen lib
        // https://github.com/carllerche/codegen/issues/27
        // struct_t.field(constant.constant_name, &constant.constant_type);
        // }
    }
    scope.to_string()
}

/// Basic mapping of ros type strings to rust types
pub fn ros_type_to_rust(t: &str) -> FieldType {
    let arr_index = t.find("[");
    if let Some(arr_index) = arr_index {
        // Get type preceding square bracket and then wrap in Vec
        let mut sub_type = ros_type_to_rust(&t[..arr_index]);
        sub_type.field_type = format! {"Vec<{}>", sub_type.field_type};
        sub_type;
    }
    match t {
        "bool" => FieldType {
            package_name: None,
            field_type: "bool".to_string(),
        },
        "int8" => FieldType {
            package_name: None,
            field_type: "i8".to_string(),
        },
        "uint8" => FieldType {
            package_name: None,
            field_type: "u8".to_string(),
        },
        "int16" => FieldType {
            package_name: None,
            field_type: "i16".to_string(),
        },
        "int32" => FieldType {
            package_name: None,
            field_type: "i32".to_string(),
        },
        "uint32" => FieldType {
            package_name: None,
            field_type: "u32".to_string(),
        },
        "int64" => FieldType {
            package_name: None,
            field_type: "i64".to_string(),
        },
        "uint64" => FieldType {
            package_name: None,
            field_type: "u64".to_string(),
        },
        "float32" => FieldType {
            package_name: None,
            field_type: "f32".to_string(),
        },
        "float64" => FieldType {
            package_name: None,
            field_type: "f64".to_string(),
        },
        "string" => FieldType {
            package_name: None,
            field_type: "String".to_string(),
        },
        "time" => FieldType {
            package_name: Some("std_msgs".into()),
            field_type: "time".into(),
        },
        "duration" => FieldType {
            package_name: Some("std_msgs".into()),
            field_type: "duration".into(),
        },
        "Header" => FieldType {
            package_name: Some("std_msgs".into()),
            field_type: "Header".into(),
        },
        _ => {
            // Reference to another package
            let mut s = t.split("/");
            FieldType {
                package_name: Some(
                    s.next()
                        .expect(&format!("Could not determine package name for {}", t))
                        .into(),
                ),
                field_type: s
                    .next()
                    .expect(&format!("Could not determine field type for {}", t))
                    .into(),
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
            field name # Some Comment
            additional field
            # Some comment",
                "name"
            ),
            MessageFile {
                fields: vec![
                    FieldInfo {
                        field_type: FieldType::Local("field".to_string()),
                        field_name: "name"
                    },
                    FieldInfo {
                        field_type: FieldType::Local("additional".to_string()),
                        field_name: "field"
                    },
                ],
                name: "name",
                constants: vec![]
            }
        )
    }
}
