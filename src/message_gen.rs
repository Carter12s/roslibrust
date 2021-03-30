use std::fs;
use std::path::{Path, PathBuf};

use log::*;
use walkdir::{DirEntry, WalkDir};
use codegen::Scope;
use std::io::Write;

#[derive(PartialEq, Debug)]
pub struct FieldInfo<'a> {
    field_type: String,
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

    let code = generate_rust(parsed);
    let mut file = std::fs::File::create(out_file).expect("Could not open output file.");
    file.write_all(code.as_bytes()).expect("Failed to write to file");
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
        if let Some(equal_i) = equal_i {
            warn!("Constants not supported yet: {} will be omitted", line);
            continue;
        }

        let mut splitter = line.split(' ');
        let field_type = splitter.next().expect("Did not find field_type on line.");
        let field_type = ros_type_to_rust(field_type);
        let field_name = splitter.next().expect("Did not find field_name on line.");
        if let Some(extra) = splitter.next() {
            panic!("Extra data detected on line past field_name");
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
pub fn generate_rust(msg_file: Vec<MessageFile>) -> String {
    let mut scope = Scope::new();
    scope.import("serde","{Deserialize,Serialize}");
    for file in &msg_file {
        let struct_t = scope.new_struct(file.name)
            .derive("Deserialize")
            .derive("Serialize")
            .derive("Debug")
            .vis("pub");
        // TODO we could use doc to move comments from .msg files to rust code
        for field in &file.fields {
            struct_t.field(field.field_name, &field.field_type);
        }
        // TODO we could detect constant naming convention and autogen enums?
        for constant in &file.constants {
            // TODO disabled for now, feature pending on codegen lib
            // https://github.com/carllerche/codegen/issues/27
            // struct_t.field(constant.constant_name, &constant.constant_type);
        }
    }
    scope.to_string()
}

/// Searches in all sub-folders of a directory for files matching the supplied predicate
// TODO figure out / test for how this handles symlinks and recursion?
pub fn recursive_find_files(path: &Path, predicate: fn(&DirEntry) -> bool) -> Vec<PathBuf> {
    WalkDir::new(".")
        .follow_links(true)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| predicate(e))
        .map(|e| e.path().to_path_buf())
        .collect()
}

/// Basic mapping of ros type strings to rust types
pub fn ros_type_to_rust(t: &str) -> String {
    let arr_index = t.find("[]");
    // TODO fixed size arrays?? Ignore and just use vecs, should still do better parsing
    if let Some(arr_index) = arr_index {
        return format!("Vec<{}>", ros_type_to_rust(&t[..arr_index]))
    }
    match t {
        "bool" => {
            "bool"
        },
        "int8" => {
            "i8"
        },
        "uint8" => {
            "u8"
        },
        "int16" => {
            "i16"
        },
        "int32" => {
            "i32"
        },
        "uint32" => {
            "u32"
        },
        "int64" => {
            "i64"
        },
        "uint64" => {
            "u64"
        },
        "float32" => {
            "f32"
        },
        "float64" => {
            "f64"
        },
        "string" => {
            "String"
        },
        "time" => {
            panic!("TODO implement time!")
            //TODO wtf why are this integral types and not part of std_msgs
        },
        "duration" => {
            panic!("TODO implement duration!")
        }
        _ => {
            // Assume this is a reference to another type, use directly
            // TODO how to separate out package specifiers
            t
        }
    }.to_string()
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
                        field_type: "field",
                        field_name: "name"
                    },
                    FieldInfo {
                        field_type: "additional",
                        field_name: "field"
                    },
                ],
                name: "name"
            }
        )
    }
}
