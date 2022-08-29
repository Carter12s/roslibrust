use std::borrow::Cow;
use std::io::Write;
use std::process::{Command, Stdio};

fn main() {
    let tokens = roslibrust_codegen::find_and_generate_ros_messages(vec!["roslibrust".into()]);
    let source = format_rust_source(tokens.to_string().as_str()).to_string();
    println!("{}", source);
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
