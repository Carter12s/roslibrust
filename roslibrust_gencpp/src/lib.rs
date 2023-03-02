use minijinja::{context, Environment, Template};
use roslibrust_codegen::MessageFile;
use std::path::{Path, PathBuf};

mod helpers;
mod spec;

use spec::MessageSpecification;

const MESSAGE_HEADER_TMPL: &str =
    include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/msg.h.j2"));

#[derive(Clone, Debug)]
pub struct IncludedNamespace {
    /// The package namespace being included
    pub package: String,
    /// The path to the package
    pub path: PathBuf,
}

pub struct MessageGenOpts {
    /// The package namespace for the generated message
    pub package: String,
    /// Include packages for resolving message dependencies
    pub includes: Vec<IncludedNamespace>,
}

pub fn generate_message(
    msg_path: &Path,
    opts: &MessageGenOpts,
) -> Result<String, minijinja::Error> {
    let search_paths = opts.includes.iter().map(|inc| inc.path.clone()).collect();
    let (messages, services) = roslibrust_codegen::find_and_parse_ros_messages(search_paths)
        .unwrap_or_else(|_| panic!("Unable to find ROS messages"));
    let (messages, _) = roslibrust_codegen::resolve_dependency_graph(messages, services).unwrap();

    let mut env = Environment::new();
    env.add_function("has_header", helpers::has_header);
    env.add_function("is_fixed_length", helpers::is_fixed_length);
    env.add_template("msg.h", MESSAGE_HEADER_TMPL).unwrap();

    let message_name = msg_path.file_stem().unwrap().to_str().unwrap().to_owned();
    match messages
        .iter()
        .find(|msg| msg.get_short_name() == message_name && msg.get_package_name() == opts.package)
    {
        Some(msg) => fill_message_template(&env.get_template("msg.h").unwrap(), msg),
        None => Err(minijinja::Error::new(
            minijinja::ErrorKind::UndefinedError,
            "Message not found in search paths",
        )),
    }
}

fn fill_message_template(
    template: &Template,
    msg_data: &MessageFile,
) -> Result<String, minijinja::Error> {
    let context = context! {
        spec => MessageSpecification::from(msg_data),
    };
    template.render(&context)
}
