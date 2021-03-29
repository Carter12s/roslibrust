use std::fs;
use std::path::{Path, PathBuf};
use walkdir::{DirEntry, WalkDir};

#[derive(PartialEq, Debug)]
pub struct FieldInfo<'a> {
    field_type: &'a str,
    field_name: &'a str,
}

#[derive(PartialEq, Debug)]
pub struct MessageFile<'a> {
    fields: Vec<FieldInfo<'a>>,
}

/// Generates rust files (.rs) containing type representing the ros messages
pub fn generate_messages(files: &Vec<PathBuf>, out_dir: &Path) {
    let mut parsed: Vec<MessageFile> = vec![];
    for entry in files {
        let content = fs::read_to_string(entry).expect("Could not read file.");
        let file = parse_ros_message_file(&content);
        parsed.push(file);
    }
}

pub fn parse_ros_message_file(data: &str) -> MessageFile {
    let mut result = MessageFile { fields: vec![] };
    for line in data.lines() {
        let line = strip_comments(line).trim();
        if line.len() == 0 {
            // Comment only line skip
            continue;
        }

        let mut splitter = line.split(' ');
        let field_type = splitter.next().expect("Did not find field_type on line.");
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

pub fn generates_rust_file(msg_file: Vec<MessageFile>, out_dir: &Path) {
    
}

pub fn recursive_find_files(path: &Path, predicate: fn(&DirEntry) -> bool) -> Vec<PathBuf> {
    WalkDir::new(".")
        .follow_links(true)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| predicate(e))
        .map(|e| e.path().to_path_buf())
        .collect()
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
            # Some comment"
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
                ]
            }
        )
    }
}
