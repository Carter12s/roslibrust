use clap::Parser;
use roslibrust_genmsg::IncludedNamespace;
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
    let opts = roslibrust_genmsg::MessageGenOpts {
        package: args.package,
        includes: args.include.unwrap_or_default(),
    };
    let extension = args.msg_path.extension().unwrap().to_str().unwrap();
    match extension {
        "msg" => {
            let generated_source =
                roslibrust_genmsg::generate_cpp_messages(&[&args.msg_path], &opts)?;
            let generated_source = generated_source.get(0).unwrap();
            let mut out_file_path = args.output;
            out_file_path.push(format!("{short_name}.h"));

            let mut out_file = std::fs::File::create(out_file_path)?;
            out_file.write_all(generated_source.message_source.as_bytes())?;
        }
        "srv" => {
            let generated_source =
                roslibrust_genmsg::generate_cpp_services(&[&args.msg_path], &opts)?;
            let generated_source = generated_source.get(0).unwrap();
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

#[cfg(test)]
mod test {
    use const_format::concatcp;
    use roslibrust_genmsg::{IncludedNamespace, MessageGenOpts};
    use std::path::PathBuf;

    const ROS_1_PATH: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../assets/ros1_common_interfaces"
    );
    const STD_MSGS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/std_msgs");
    const GEOMETRY_MSGS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/common_msgs/geometry_msgs");
    const SENSOR_MSGS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/common_msgs/sensor_msgs");
    const STD_SRVS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/ros_comm_msgs/std_srvs");

    const HEADER_MSG_PATH: &str = concatcp!(STD_MSGS_PKG_PATH, "/msg/Header.msg");
    const BATTERY_STATE_MSG_PATH: &str = concatcp!(SENSOR_MSGS_PKG_PATH, "/msg/BatteryState.msg");
    const TRIGGER_SRV_PATH: &str = concatcp!(STD_SRVS_PKG_PATH, "/srv/Trigger.srv");
    const TRANSFORM_STAMPED_MSG_PATH: &str =
        concatcp!(GEOMETRY_MSGS_PKG_PATH, "/msg/TransformStamped.msg");

    fn remove_whitespace(s: &str) -> String {
        s.split_whitespace().collect()
    }

    #[test]
    fn std_msgs_header_up_to_date() {
        let options = MessageGenOpts {
            package: "std_msgs".into(),
            includes: vec![IncludedNamespace {
                package: "std_msgs".into(),
                path: STD_MSGS_PKG_PATH.into(),
            }],
        };
        let generated_source =
            roslibrust_genmsg::generate_cpp_messages(&[&PathBuf::from(HEADER_MSG_PATH)], &options)
                .unwrap();

        let current_source = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/test_package/include/std_msgs/Header.h"
        ))
        .unwrap();
        assert_eq!(
            remove_whitespace(&generated_source[0].message_source),
            remove_whitespace(&current_source)
        );
    }

    #[test]
    fn sensor_msgs_battery_state_up_to_date() {
        let options = MessageGenOpts {
            package: "sensor_msgs".into(),
            includes: vec![
                IncludedNamespace {
                    package: "std_msgs".into(),
                    path: STD_MSGS_PKG_PATH.into(),
                },
                IncludedNamespace {
                    package: "geometry_msgs".into(),
                    path: GEOMETRY_MSGS_PKG_PATH.into(),
                },
                IncludedNamespace {
                    package: "sensor_msgs".into(),
                    path: SENSOR_MSGS_PKG_PATH.into(),
                },
            ],
        };
        let generated_source = roslibrust_genmsg::generate_cpp_messages(
            &[&PathBuf::from(BATTERY_STATE_MSG_PATH)],
            &options,
        )
        .unwrap();

        let current_source = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/test_package/include/sensor_msgs/BatteryState.h"
        ))
        .unwrap();
        assert_eq!(
            remove_whitespace(&generated_source[0].message_source),
            remove_whitespace(&current_source)
        );
    }

    #[test]
    fn std_srvs_trigger_up_to_date() {
        let options = MessageGenOpts {
            package: "std_srvs".into(),
            includes: vec![IncludedNamespace {
                package: "std_srvs".into(),
                path: STD_SRVS_PKG_PATH.into(),
            }],
        };
        let generated_source =
            roslibrust_genmsg::generate_cpp_services(&[&PathBuf::from(TRIGGER_SRV_PATH)], &options)
                .unwrap();

        let current_source = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/test_package/include/std_srvs/Trigger.h"
        ))
        .unwrap();
        assert_eq!(
            remove_whitespace(&generated_source[0].srv_header),
            remove_whitespace(&current_source)
        );
    }

    #[test]
    fn transform_stamped_with_user_template() {
        let options = MessageGenOpts {
            package: "geometry_msgs".into(),
            includes: vec![
                IncludedNamespace {
                    package: "std_msgs".into(),
                    path: STD_MSGS_PKG_PATH.into(),
                },
                IncludedNamespace {
                    package: "geometry_msgs".into(),
                    path: GEOMETRY_MSGS_PKG_PATH.into(),
                },
            ],
        };
        let mapping =
            std::collections::HashMap::from([("string".to_owned(), "NativeString".to_owned())]);
        let generated_source = roslibrust_genmsg::generate_messages_with_templates(
            &[&PathBuf::from(TRANSFORM_STAMPED_MSG_PATH)],
            &options,
            mapping,
            r#"
            pub struct {{ spec.short_name }} {
                {% for field in spec.fields %}
                {{ field.name }}: {{ field.field_type|typename_conversion }},
                {%- endfor %}        
            }
            "#,
        )
        .unwrap();

        let current_source = r#"
            pub struct TransformStamped {
                header: Header,
                child_frame_id: NativeString,
                transform: Transform,
            }
        "#;
        assert_eq!(
            remove_whitespace(&generated_source[0].message_source),
            remove_whitespace(&current_source)
        );
    }
}
