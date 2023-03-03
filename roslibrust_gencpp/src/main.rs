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

    // Verify output path is an existing directory
    if !args.output.exists() {
        log::error!("Output path does not exist: {}", args.output.display());
        std::process::exit(1);
    }
    if !args.output.is_dir() {
        log::error!("Output path must be a directory: {}", args.output.display());
        std::process::exit(1);
    }

    let short_name = args.msg_path.file_stem().unwrap().to_str().unwrap();
    let opts = roslibrust_gencpp::MessageGenOpts {
        package: args.package,
        includes: args.include.unwrap_or_default(),
    };
    let extension = args.msg_path.extension().unwrap().to_str().unwrap();
    match extension {
        "msg" => {
            let generated_source = roslibrust_gencpp::generate_message(&args.msg_path, &opts)?;
            let mut out_file_path = args.output;
            out_file_path.push(format!("{short_name}.h"));

            let mut out_file = std::fs::File::create(out_file_path)?;
            out_file.write_all(generated_source.as_bytes())?;
        }
        "srv" => {
            let generated_source = roslibrust_gencpp::generate_service(&args.msg_path, &opts)?;
            let mut srv_out_path = args.output.clone();
            srv_out_path.push(format!("{short_name}.h"));
            let mut srv_request_out_path = args.output.clone();
            srv_request_out_path.push(format!("{short_name}Request.h"));
            let mut srv_response_out_path = args.output;
            srv_response_out_path.push(format!("{short_name}Response.h"));

            let mut out_file = std::fs::File::create(srv_out_path)?;
            out_file.write_all(generated_source.srv_header.as_bytes())?;
            let mut out_file = std::fs::File::create(srv_request_out_path)?;
            out_file.write_all(generated_source.request_msg_header.as_bytes())?;
            let mut out_file = std::fs::File::create(srv_response_out_path)?;
            out_file.write_all(generated_source.response_msg_header.as_bytes())?;
        }
        _ => {
            log::error!(
                "Unrecognized extension: {extension} in provided input file: {}",
                args.msg_path.display()
            );
            std::process::exit(1);
        }
    }
    Ok(())
}
