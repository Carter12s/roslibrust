use std::borrow::Cow;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};

const ROS_1_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../assets/ros1_common_interfaces"
);

const ROS_2_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../assets/ros2_common_interfaces"
);

/// This main function is used to generate the contents of ros1.rs, ros2.rs
fn main() {
    let tokens = roslibrust_codegen::find_and_generate_ros_messages(vec![ROS_1_PATH.into()]);
    let source = format_rust_source(tokens.to_string().as_str()).to_string();
    let _ = std::fs::write(concat!(env!("CARGO_MANIFEST_DIR"), "/src/ros1.rs"), source);

    let tokens = roslibrust_codegen::find_and_generate_ros_messages(vec![ROS_2_PATH.into()]);
    let source = format_rust_source(tokens.to_string().as_str()).to_string();
    let _ = std::fs::write(concat!(env!("CARGO_MANIFEST_DIR"), "/src/ros2.rs"), source);
}

fn format_rust_source(source: &str) -> Cow<'_, str> {
    if let Ok(mut process) = Command::new("rustfmt")
        .arg("--emit=stdout")
        .arg("--edition=2021")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
    {
        {
            let stdin = process.stdin.as_mut().unwrap();
            stdin.write_all(source.as_bytes()).unwrap()
        }
        if let Ok(output) = process.wait_with_output() {
            if output.status.success() {
                return std::str::from_utf8(&output.stdout[..])
                    .unwrap()
                    .to_owned()
                    .into();
            }
        }
    }
    Cow::Borrowed(source)
}

#[cfg(test)]
mod test {
    use crate::*;

    /// Confirms that codegen has been run and changes committed
    #[test]
    fn ros1_lib_is_up_to_date() {
        let tokens = roslibrust_codegen::find_and_generate_ros_messages(vec![ROS_1_PATH.into()]);
        let source = format_rust_source(tokens.to_string().as_str()).to_string();
        let lib_path = env!("CARGO_MANIFEST_DIR").to_string() + "/src/ros1.rs";
        let lib_contents =
            std::fs::read_to_string(lib_path).expect("Failed to load current ros1.rs contents");

        // Creating a diff so if there are changes output in CI is sane
        let diff = diffy::create_patch(&source, &lib_contents);
        println!("Diff is \n{}", diff);

        if source.trim() != lib_contents.trim() {
            panic!("Changes detected see diff!");
        }
    }

    /// Confirms that codegen has been run and changes committed
    #[test]
    fn ros2_lib_is_up_to_date() {
        let tokens = roslibrust_codegen::find_and_generate_ros_messages(vec![ROS_2_PATH.into()]);
        let source = format_rust_source(tokens.to_string().as_str()).to_string();
        let lib_path = env!("CARGO_MANIFEST_DIR").to_string() + "/src/ros2.rs";
        let lib_contents =
            std::fs::read_to_string(lib_path).expect("Failed to load current ros2.rs contents");

        // Creating a diff so if there are changes output in CI is sane
        let diff = diffy::create_patch(&source, &lib_contents);
        println!("Diff is \n{}", diff);

        if source.trim() != lib_contents.trim() {
            panic!("Changes detected see diff!");
        }
    }
}
