use clap::Parser;
use roslibrust_genmsg::IncludedNamespace;
use std::{
    io::Write,
    path::{Path, PathBuf},
};

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
    let extension = args.msg_path.extension().unwrap().to_str().unwrap();
    let msg_paths = args
        .include
        .unwrap_or_default()
        .into_iter()
        .map(|inc| inc.path)
        .collect::<Vec<_>>();
    let generator = roslibrust_genmsg::make_cpp_generator(&msg_paths).unwrap();

    match extension {
        "msg" => {
            let generated_source = generator.generate_messages().unwrap();
            let msg_source = generated_source
                .iter()
                .find(|msg| msg.message_name == short_name && msg.package_name == args.package)
                .unwrap();
            write_source_file(
                &args.output,
                &format!("{short_name}.h"),
                &msg_source.message_source,
            )?;
        }
        "srv" => {
            let generated_source = generator.generate_services().unwrap();
            let srv_source = generated_source
                .iter()
                .find(|srv| srv.service_name == short_name && srv.package_name == args.package)
                .unwrap();
            write_source_file(
                &args.output,
                &format!("{short_name}.h"),
                &srv_source.service_source,
            )?;
            write_source_file(
                &args.output,
                &format!("{short_name}Request.h"),
                &srv_source.request_source,
            )?;
            write_source_file(
                &args.output,
                &format!("{short_name}Response.h"),
                &srv_source.response_source,
            )?;
        }
        "action" => {
            let expected_messages = [
                short_name.to_string(),
                format!("{short_name}Goal"),
                format!("{short_name}Result"),
                format!("{short_name}Feedback"),
                format!("{short_name}ActionGoal"),
                format!("{short_name}ActionResult"),
                format!("{short_name}ActionFeedback"),
            ];

            let generated_source = generator.generate_messages().unwrap();
            let action_sources = generated_source
                .into_iter()
                .filter(|msg| {
                    expected_messages.contains(&msg.message_name)
                        && msg.package_name == args.package
                })
                .collect::<Vec<_>>();
            if action_sources.len() == expected_messages.len() {
                action_sources.into_iter().try_for_each(|src| {
                    write_source_file(
                        &args.output,
                        &format!("{}.h", src.message_name),
                        &src.message_source,
                    )
                })?;
            } else {
                log::error!(
                    "Improperly generated action messages, generated: {:?}",
                    action_sources
                        .into_iter()
                        .map(|src| src.message_name)
                        .collect::<Vec<_>>()
                );
                std::process::exit(1);
            }
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

fn write_source_file(out_path: &Path, filename: &str, source: &str) -> std::io::Result<()> {
    let mut out_file_path = out_path.to_owned();
    out_file_path.push(filename);

    let mut out_file = std::fs::File::create(out_file_path)?;
    out_file.write_all(source.as_bytes())
}

#[cfg(test)]
mod test {
    use const_format::concatcp;

    const ROS_1_PATH: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../assets/ros1_common_interfaces"
    );
    const STD_MSGS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/std_msgs");
    const GEOMETRY_MSGS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/common_msgs/geometry_msgs");
    const SENSOR_MSGS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/common_msgs/sensor_msgs");
    const STD_SRVS_PKG_PATH: &str = concatcp!(ROS_1_PATH, "/ros_comm_msgs/std_srvs");

    fn remove_whitespace(s: &str) -> String {
        s.split_whitespace().collect()
    }

    #[test]
    fn std_msgs_header_up_to_date() {
        let generated_source = roslibrust_genmsg::make_cpp_generator(&[STD_MSGS_PKG_PATH])
            .unwrap()
            .generate_messages()
            .unwrap();
        let header = generated_source
            .iter()
            .find(|msg| &msg.message_name == "Header" && &msg.package_name == "std_msgs")
            .unwrap();

        let current_source = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/test_package/include/std_msgs/Header.h"
        ))
        .unwrap();
        assert_eq!(
            remove_whitespace(&header.message_source),
            remove_whitespace(&current_source)
        );
    }

    #[test]
    fn sensor_msgs_battery_state_up_to_date() {
        let generated_source = roslibrust_genmsg::make_cpp_generator(&[
            STD_MSGS_PKG_PATH,
            GEOMETRY_MSGS_PKG_PATH,
            SENSOR_MSGS_PKG_PATH,
        ])
        .unwrap()
        .generate_messages()
        .unwrap();

        let battery_state = generated_source
            .iter()
            .find(|msg| &msg.message_name == "BatteryState" && &msg.package_name == "sensor_msgs")
            .unwrap();

        let current_source = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/test_package/include/sensor_msgs/BatteryState.h"
        ))
        .unwrap();
        assert_eq!(
            remove_whitespace(&battery_state.message_source),
            remove_whitespace(&current_source)
        );
    }

    #[test]
    fn std_srvs_trigger_up_to_date() {
        let generated_source = roslibrust_genmsg::make_cpp_generator(&[STD_SRVS_PKG_PATH])
            .unwrap()
            .generate_services()
            .unwrap();
        let trigger_srv = generated_source
            .iter()
            .find(|srv| &srv.service_name == "Trigger" && &srv.package_name == "std_srvs")
            .unwrap();

        let current_source = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/test_package/include/std_srvs/Trigger.h"
        ))
        .unwrap();
        assert_eq!(
            remove_whitespace(&trigger_srv.service_source),
            remove_whitespace(&current_source)
        );
    }

    #[test]
    fn transform_stamped_with_user_template() {
        let mapping =
            std::collections::HashMap::from([("string".to_owned(), "NativeString".to_owned())]);
        let generator = roslibrust_genmsg::CodeGeneratorBuilder::new(
            &[STD_MSGS_PKG_PATH, GEOMETRY_MSGS_PKG_PATH],
            r#"
            pub struct {{ spec.short_name }} {
                {% for field in spec.fields %}
                {{ field.name }}: {{ field.field_type|map_type }},
                {%- endfor %}        
            }
            "#,
        )
        .add_filter("map_type", move |v| {
            let value = serde_json::to_value(v).unwrap();
            let ros_type = value.as_str().unwrap();
            let typename = match mapping.get(ros_type) {
                Some(native_type) => native_type.clone(),
                None => ros_type.to_string(),
            };
            typename.into()
        })
        .build()
        .unwrap();

        let sources = generator.generate_messages().unwrap();
        let transform_stamped = sources
            .iter()
            .find(|msg| {
                &msg.message_name == "TransformStamped" && &msg.package_name == "geometry_msgs"
            })
            .unwrap();

        let current_source = r#"
            pub struct TransformStamped {
                header: Header,
                child_frame_id: NativeString,
                transform: Transform,
            }
        "#;
        assert_eq!(
            remove_whitespace(&transform_stamped.message_source),
            remove_whitespace(&current_source)
        );
    }
}
