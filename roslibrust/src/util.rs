use log::*;
use std::env;
use std::path::{Path, PathBuf};
use walkdir::{DirEntry, WalkDir};

/// Identifier for ros data files which is combination of package_name and path
#[derive(Debug, Clone)]
pub struct RosFile {
    pub package_name: String,
    pub path: PathBuf,
}

/// Searches in all sub-folders of a directory for files matching the supplied predicate
/// Only applicable for finding files within packages
/// Returns both file and package it belongs to.
// TODO figure out / test for how this handles symlinks and recursion?
pub fn recursive_find_files(path: &Path, predicate: fn(&DirEntry) -> bool) -> Vec<RosFile> {
    WalkDir::new(path)
        .follow_links(true)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| predicate(e))
        .map(|e| e.path().to_path_buf())
        .map(|e| {
            debug!("Found installed ros file @ {:?}", &e);
            let pkg_name = find_package_from_path(&e).unwrap_or("".to_string());
            RosFile {
                path: e,
                package_name: pkg_name,
            }
        })
        .collect()
}

/// Finds package name be walking up directory until package.xml is found
/// Returns None if package.xml is not found
pub fn find_package_from_path(e: &PathBuf) -> Option<String> {
    let mut package_name: Option<String> = None;
    for dir in e.ancestors() {
        if dir.join("package.xml").exists() {
            package_name = Some(
                dir.into_iter()
                    .last()
                    .unwrap()
                    .to_string_lossy()
                    .to_string(),
            );
        }
    }
    if package_name.is_none() {
        debug!(
            "Found ros file, but could not determine package name: {:?}",
            e
        );
        return None;
    }
    Some(package_name.unwrap())
}

pub fn recursive_find_msg_files(path: &Path) -> Vec<RosFile> {
    recursive_find_files(path, |e| e.file_name().to_string_lossy().ends_with(".msg"))
}

pub fn recursive_find_srv_files(path: &Path) -> Vec<RosFile> {
    recursive_find_files(path, |e| e.file_name().to_string_lossy().ends_with(".srv"))
}

pub fn recursive_find_action_files(path: &Path) -> Vec<RosFile> {
    recursive_find_files(path, |e| {
        e.file_name().to_string_lossy().ends_with(".action")
    })
}

/// Looks up all messages installed in ros paths
pub fn get_installed_msgs() -> Result<Vec<RosFile>, Box<dyn std::error::Error>> {
    let rpp = env::var("ROS_PACKAGE_PATH")?;
    let rpp = rpp + concat!(":", env!("CARGO_MANIFEST_DIR"), "local_msgs/std_msgs");

    // Assuming unix path delimiter, please don't ask me to make this work on windows...
    let paths = rpp.split(":");

    let mut res: Vec<RosFile> = vec![];
    for path in paths {
        res.append(&mut recursive_find_msg_files(Path::new(path)));
    }
    Ok(res)
}

/// Get the locally installed message definitions within the package
/// This is useful for message generation in environments where ROS is not installed
pub fn get_local_msgs() -> Vec<RosFile> {
    let rpp = concat!(env!("CARGO_MANIFEST_DIR"), "/local_msgs");
    debug!("Looking for locally installed msgs in: {}", &rpp);
    recursive_find_msg_files(Path::new(rpp))
}

/// This returns a subset of the local messages that define special types e.g. time
pub fn get_special_msgs() -> Vec<RosFile> {
    let rpp = concat!(env!("CARGO_MANIFEST_DIR"), "/local_msgs");
    vec![
        RosFile {
            package_name: "std_msgs".to_string(),
            path: Path::new(rpp).join("std_msgs/msg/TimeI.msg"),
        },
        RosFile {
            package_name: "std_msgs".to_string(),
            path: Path::new(rpp).join("std_msgs/msg/DurationI.msg"),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    //TODO this test only works if you have ROS installed and ROS_PACKAGE_PATH defined
    //How to test this better?
    #[test]
    fn get_installed_msgs_test() {
        if let Ok(v) = get_installed_msgs() {
            print!("Installed msgs: {:?}", v);
        }
    }
}
