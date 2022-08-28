use std::io;
use std::path::{Path, PathBuf};

#[derive(Clone, Debug)]
pub struct Package {
    pub name: String,
    pub path: PathBuf,
}

const CATKIN_IGNORE: &'static str = "CATKIN_IGNORE";
const PACKAGE_FILE_NAME: &'static str = "package.xml";
const ROS_PACKAGE_PATH_ENV_VAR: &'static str = "ROS_PACKAGE_PATH";

pub fn get_search_paths() -> Vec<PathBuf> {
    if let Ok(paths) = std::env::var(ROS_PACKAGE_PATH_ENV_VAR) {
        #[cfg(unix)]
        let separator = ":";
        #[cfg(windows)]
        let separator = ";";

        paths
            .split(separator)
            .map(|path| PathBuf::from(path))
            .collect::<Vec<PathBuf>>()
    } else {
        eprintln!("No ROS_PACKAGE_PATH defined.");
        vec![]
    }
}

/// Finds ROS packages within a list of search paths.
///
/// This function may panic if it reaches a maximum search depth. If this function
/// panics while you're using it, you may have some infinite loop in your paths
/// due to symlinking.
pub fn crawl(search_paths: Vec<PathBuf>) -> Vec<Package> {
    let mut packages = vec![];

    for path in search_paths {
        const MAX_RECURSION_DEPTH: u16 = 1000;
        if let Ok(found_packages) = packages_from_path(path, MAX_RECURSION_DEPTH) {
            packages = [packages, found_packages].concat();
        }
    }

    packages
}

fn packages_from_path(mut path: PathBuf, depth: u16) -> io::Result<Vec<Package>> {
    let mut found_packages = vec![];

    if depth == 0 {
        eprintln!(
            "Reached depth limit in: {}",
            path.as_os_str().to_string_lossy()
        );
        return Err(io::ErrorKind::Other.into());
    }

    if path.as_path().is_dir() {
        // We have a valid path
        path.push(CATKIN_IGNORE);
        // We'll only check this directory if no CATKIN_IGNORE file is present
        if !path.as_path().is_file() {
            assert!(path.pop());

            path.push(PACKAGE_FILE_NAME);
            if path.as_path().is_file() {
                // And there's a package.xml here!
                assert!(path.pop());
                found_packages.push(Package {
                    name: String::from(path.file_name().unwrap().to_string_lossy()),
                    path: path,
                });
            } else {
                // No file here, we'll have to go deeper
                assert!(path.pop());
                for subdir in std::fs::read_dir(path)
                    .unwrap()
                    .filter(|entry| match entry {
                        Ok(entry) => entry.path().as_path().is_dir(),
                        Err(_err) => false,
                    })
                    .map(|entry| entry.unwrap())
                {
                    found_packages = [
                        found_packages,
                        packages_from_path(subdir.path(), depth - 1)?,
                    ]
                    .concat()
                }
            }
        }
    } else {
        eprintln!("{} is not a directory", path.to_string_lossy())
    }

    Ok(found_packages)
}

pub fn get_message_files(pkg: &Package) -> io::Result<Vec<PathBuf>> {
    message_files_from_path(pkg.path.as_path(), "msg")
}

pub fn get_service_files(pkg: &Package) -> io::Result<Vec<PathBuf>> {
    message_files_from_path(pkg.path.as_path(), "srv")
}

fn message_files_from_path(path: &Path, ext: &str) -> io::Result<Vec<PathBuf>> {
    let mut msg_files = vec![];
    for entry in std::fs::read_dir(path)? {
        if let Ok(entry) = entry {
            if entry.path().as_path().is_dir() {
                msg_files = [
                    msg_files,
                    message_files_from_path(entry.path().as_path(), ext)?,
                ]
                .concat()
            } else if entry.path().as_path().is_file()
                && entry.path().extension().unwrap().to_str().unwrap() == ext
            {
                msg_files.push(entry.path())
            }
        }
    }

    Ok(msg_files)
}
