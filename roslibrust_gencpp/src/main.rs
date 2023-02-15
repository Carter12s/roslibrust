use clap::Parser;
use roslibrust_gencpp::IncludedNamespace;
use std::{io::Write, path::PathBuf};

#[derive(Parser, Debug)]
struct Args {
    /// Path to the input .msg file
    #[arg(long = "msg", short = 'm')]
    msg_path: PathBuf,
    /// The package namespace for the generated message
    #[arg(long, short)]
    package: String,
    /// Output directory for generated code
    #[arg(long, short)]
    output: PathBuf,
    /// Include namespaces for message dependencies
    #[arg(long, short = 'I', value_parser = include_namespace_parse)]
    include: Option<Vec<IncludedNamespace>>,
}

fn include_namespace_parse(s: &str) -> Result<IncludedNamespace, String> {
    let components = s.split(':').collect::<Vec<&str>>();
    if components.len() == 2 {
        let package = components[0].to_owned();
        let path = PathBuf::from(components[1]);
        Ok(IncludedNamespace { package, path })
    } else {
        Err(String::from("Expected format: 'PACKAGE:/some/path'"))
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();
    let opts = roslibrust_gencpp::MessageGenOpts {
        package: args.package,
        includes: args.include.unwrap_or_default(),
    };
    let generated_source = roslibrust_gencpp::generate_message(&args.msg_path, &opts)?;
    let mut out_file = std::fs::File::create(args.output)?;
    out_file.write_all(generated_source.as_bytes())?;
    Ok(())
}
