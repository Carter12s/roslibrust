use log::*;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use std::collections::{BTreeMap, VecDeque};
use std::path::PathBuf;
use std::str::FromStr;
use syn::parse_quote;
use utils::Package;

mod parse;
use parse::*;

pub mod utils;

pub mod integral_types;
pub use integral_types::*;

use serde::de::DeserializeOwned;
use serde::Serialize;
use std::fmt::Debug;

/// Fundamental traits for message types this crate works with
/// This trait will be satisfied for any types generated with this crate's message_gen functionality
pub trait RosMessageType:
    'static + DeserializeOwned + Default + Send + Serialize + Sync + Clone + Debug
{
    /// Expected to be the combination pkg_name/type_name string describing the type to ros
    /// Example: std_msgs/Header
    const ROS_TYPE_NAME: &'static str;
}

// This special impl allows for services with no args / returns
impl RosMessageType for () {
    const ROS_TYPE_NAME: &'static str = "";
}

/// Fundamental traits for service types this crate works with
/// This trait will be satisfied for any services definitions generated with this crate's message_gen functionality
pub trait RosServiceType {
    /// Name of the ros service e.g. `rospy_tutorials/AddTwoInts`
    const ROS_SERVICE_NAME: &'static str;
    /// The type of data being sent in the request
    type Request: RosMessageType;
    /// The type of the data
    type Response: RosMessageType;
}

/// Searches a list of paths for ROS packages and generates struct definitions
/// and implementations for message files and service files in packages it finds.
///
/// * `additional_search_paths` -- A list of additional paths to search beyond those
/// found in ROS_PACKAGE_PATH environment variable.
pub fn find_and_generate_ros_messages(additional_search_paths: Vec<PathBuf>) -> TokenStream {
    let mut search_paths = utils::get_search_paths();
    search_paths.extend(additional_search_paths.into_iter());
    debug!(
        "Codegen is looking in following paths for files: {:?}",
        &search_paths
    );
    let mut packages = utils::crawl(search_paths.clone());
    packages.dedup_by(|a, b| a.name == b.name);

    if packages.len() == 0 {
        eprintln!(
            "Warning: No packages found while searching in: {search_paths:?}, relative to {:?}",
            std::env::current_dir()
        );
    }

    let mut message_files = packages
        .iter()
        .map(|pkg| {
            utils::get_message_files(pkg)
                .unwrap_or_else(|err| {
                    eprintln!(
                        "Unable to get paths to message files for {}: {}",
                        pkg.name, err
                    );
                    // Return an empty vec so that one package doesn't necessarily fail the process
                    vec![]
                })
                .into_iter()
                .map(|path| (pkg.clone(), path))
        })
        .flatten()
        .collect::<Vec<_>>();
    let service_files = packages
        .iter()
        .map(|pkg| {
            utils::get_service_files(pkg)
                .unwrap_or_else(|err| {
                    eprintln!(
                        "Unable to get paths to service files for {}: {}",
                        pkg.name, err
                    );
                    // Return an empty vec so that one package doesn't necessarily fail the process
                    vec![]
                })
                .into_iter()
                .map(|path| (pkg.clone(), path))
        })
        .flatten()
        .collect::<Vec<_>>();
    message_files.extend_from_slice(&service_files[..]);
    let (messages, services) = parse_ros_files(message_files).unwrap();
    let mut modules_to_struct_definitions: BTreeMap<String, Vec<TokenStream>> = BTreeMap::new();

    // Convert messages files into rust token streams and insert them into BTree organized by package
    messages.into_iter().for_each(|(_, message)| {
        let pkg_name = message.package.clone();
        let definition = generate_struct(message);
        if let Some(entry) = modules_to_struct_definitions.get_mut(&pkg_name) {
            entry.push(definition);
        } else {
            modules_to_struct_definitions.insert(pkg_name, vec![definition]);
        }
    });
    // Do the same for services
    services.into_iter().for_each(|service| {
        let pkg_name = service.package.clone();
        let definition = generate_service(service);
        if let Some(entry) = modules_to_struct_definitions.get_mut(&pkg_name) {
            entry.push(definition);
        } else {
            modules_to_struct_definitions.insert(pkg_name, vec![definition]);
        }
    });
    // Now generate modules to wrap all of the TokenStreams in a module for each package
    let all_pkgs = modules_to_struct_definitions
        .iter()
        .map(|(k, _)| k)
        .cloned()
        .collect::<Vec<String>>();
    let module_definitions = modules_to_struct_definitions
        .into_iter()
        .map(|(pkg, struct_defs)| generate_mod(pkg, struct_defs, &all_pkgs[..]))
        .collect::<Vec<_>>();

    quote! {
        #(#module_definitions)*

    }
}

struct MessageMetadata {
    pub package: String,
    pub path: PathBuf,
    pub seen_count: i32,
    pub parsed: MessageFile,
    pub unparsed: String,
}

/// Parses all ROS file types and returns a final expanded set
/// Currently supports service files and message files, no planned support for actions
/// The returned BTree will contain all messages files including those buried within the service definitions
/// and will have fully expanded and resolved referenced types in other packages.
/// * `msg_paths` -- List of tuple (Package, Path to File) for each file to parse
fn parse_ros_files(
    msg_paths: Vec<(Package, PathBuf)>,
) -> std::io::Result<(BTreeMap<String, MessageFile>, Vec<ServiceFile>)> {
    let mut parsed_messages = VecDeque::new();
    let mut parsed_services = Vec::new();
    for (pkg, path) in msg_paths {
        let contents = std::fs::read_to_string(&path)?;
        let name = path.file_stem().unwrap().to_str().unwrap();
        match path.extension().unwrap().to_str().unwrap() {
            "srv" => {
                let srv_file = parse_ros_service_file(contents, name.to_string(), &pkg);
                // TODO stop cloning with reckless abandon
                parsed_messages.push_back(MessageMetadata {
                    package: pkg.name.clone(),
                    path: path.clone(),
                    seen_count: 0,
                    parsed: srv_file.request_type.clone(),
                    unparsed: srv_file.request_type_raw.clone(),
                });
                parsed_messages.push_back(MessageMetadata {
                    package: pkg.name.clone(),
                    path: path.clone(),
                    seen_count: 0,
                    parsed: srv_file.response_type.clone(),
                    unparsed: srv_file.response_type_raw.clone(),
                });
                parsed_services.push(srv_file);
            }
            "msg" => {
                let msg = parse_ros_message_file(contents.clone(), name.to_string(), &pkg);
                parsed_messages.push_back(MessageMetadata {
                    package: pkg.name,
                    path: path.clone(),
                    seen_count: 0,
                    parsed: msg,
                    unparsed: contents,
                });
            }
            _ => {
                panic!("File extension not recognized as a ROS file: {path:?}");
            }
        }
    }
    parsed_services.sort_by(|a, b| a.name.cmp(&b.name));
    Ok((
        resolve_message_dependencies(parsed_messages),
        parsed_services,
    ))
}

fn resolve_message_dependencies(
    mut parsed_msgs: VecDeque<MessageMetadata>,
) -> BTreeMap<String, MessageFile> {
    const MAX_PARSE_ITER_LIMIT: i32 = 2048;
    let mut message_map = BTreeMap::new();

    while let Some(MessageMetadata {
        package,
        path,
        seen_count,
        parsed,
        unparsed,
    }) = parsed_msgs.pop_front()
    {
        if seen_count > MAX_PARSE_ITER_LIMIT {
            panic!("Unable to resolve dependencies after reaching iteration limit ({MAX_PARSE_ITER_LIMIT}).\n\
                    Message: {package}/{}\n\
                    Message Definition:\n\
                    {unparsed}\n\n\
                    Parsed: {parsed:#?}",
                    parsed.name);
        }

        // Check if each dependency of the message is a primitive or has been resolved
        let fully_resolved = parsed.fields.iter().all(|field| {
            ROS_TYPE_TO_RUST_TYPE_MAP.contains_key(field.field_type.field_type.as_str())
                || message_map.contains_key(
                    format!(
                        "{}/{}",
                        field.field_type.package_name.as_ref().unwrap(),
                        &field.field_type.field_type
                    )
                    .as_str(),
                )
        });

        if fully_resolved {
            // We can remove it if the whole tree is understood
            message_map.insert(
                format!("{}/{}", package, parsed.name),
                replace_ros_types_with_rust_types(parsed),
            );
        } else {
            // Otherwise put it back in the queue
            parsed_msgs.push_back(MessageMetadata {
                package,
                path,
                seen_count: seen_count + 1,
                parsed,
                unparsed,
            });
        }
    }

    message_map
}

fn derive_attrs() -> Vec<syn::Attribute> {
    vec![
        parse_quote! { #[derive(::serde::Deserialize)] },
        parse_quote! { #[derive(::serde::Serialize)] },
        parse_quote! { #[derive(Debug)] },
        parse_quote! { #[derive(Default)] },
        parse_quote! { #[derive(Clone)] },
        parse_quote! { #[derive(PartialEq)] },
    ]
}

/// Generates the service for a given service file
/// The service definition defines a struct representing the service an an implementation
/// of the RosServiceType trait for that struct
fn generate_service(service: ServiceFile) -> TokenStream {
    let service_type_name = format!("{}/{}", &service.package, &service.name);
    let struct_name = format_ident!("{}", service.name);
    let request_name = format_ident!("{}", service.request_type.name);
    let response_name = format_ident!("{}", service.response_type.name);
    quote! {
        pub struct #struct_name {

        }
        impl ::roslibrust_codegen::RosServiceType for #struct_name {
            const ROS_SERVICE_NAME: &'static str = #service_type_name;
            type Request = #request_name;
            type Response = #response_name;
        }
    }
}

fn generate_struct(msg: MessageFile) -> TokenStream {
    let attrs = derive_attrs();
    let fields = msg
        .fields
        .into_iter()
        .map(|mut field| {
            field.field_type.field_type = match field.field_type.package_name {
                Some(pkg) => {
                    if pkg.as_str() == msg.package.as_str() {
                        format!("self::{}", field.field_type.field_type)
                    } else {
                        format!("{}::{}", pkg, field.field_type.field_type)
                    }
                }
                None => field.field_type.field_type,
            };
            let field_type = if field.field_type.is_vec {
                format!("::std::vec::Vec<{}>", field.field_type.field_type)
            } else {
                field.field_type.field_type
            };
            let field_type = TokenStream::from_str(field_type.as_str()).unwrap();

            let field_name = format_ident!("r#{}", field.field_name);
            quote! { pub #field_name: #field_type, }
        })
        .collect::<Vec<TokenStream>>();

    let constants = msg
        .constants
        .into_iter()
        .map(|constant| {
            let constant_name = format_ident!("r#{}", constant.constant_name);
            let (constant_type, constant_value) =
                if constant.constant_type == "::std::string::String" {
                    let constant_value = constant.constant_value;
                    (String::from("&'static str"), quote! { #constant_value })
                } else {
                    (
                        constant.constant_type,
                        TokenStream::from_str(constant.constant_value.as_str()).unwrap(),
                    )
                };
            let constant_type = TokenStream::from_str(constant_type.as_str()).unwrap();
            quote! { pub const #constant_name: #constant_type = #constant_value; }
        })
        .collect::<Vec<TokenStream>>();

    let struct_name = format_ident!("{}", msg.name);
    let ros_type_name = format!("{}/{}", msg.package, struct_name);

    let mut base = quote! {
        #[allow(non_snake_case)]
        #(#attrs )*
        pub struct #struct_name {
            #(#fields )*
        }

        impl ::roslibrust_codegen::RosMessageType for #struct_name {
            const ROS_TYPE_NAME: &'static str = #ros_type_name;
        }
    };

    // Only if we have constants append the impl
    if !constants.is_empty() {
        base.extend(quote! {
            impl #struct_name {
                #(#constants )*
            }
        });
    }
    base
}

fn generate_mod(
    pkg_name: String,
    struct_definitions: Vec<TokenStream>,
    all_pkgs: &[String],
) -> TokenStream {
    let mod_name = format_ident!("{}", &pkg_name);
    let all_pkgs = all_pkgs
        .iter()
        .filter(|item| item.as_str() != pkg_name.as_str())
        .map(|pkg| format_ident!("{}", pkg))
        .collect::<Vec<_>>();

    quote! {
        #[allow(unused_imports)]
        pub mod #mod_name {
            #(use super::#all_pkgs; )*

            #(#struct_definitions )*
        }
    }
}

#[cfg(test)]
mod test {
    use crate::find_and_generate_ros_messages;

    /// Confirms we don't panic on ros1 parsing
    #[test]
    fn generate_ok_on_ros1() {
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/ros1_common_interfaces");

        find_and_generate_ros_messages(vec![assets_path.into()]);
    }

    /// Confirms we don't panic on ros2 parsing
    #[test]
    fn generate_ok_on_ros2() {
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/ros2_common_interfaces");

        find_and_generate_ros_messages(vec![assets_path.into()]);
    }

    /// Confirms we don't panic on test_msgs parsing
    #[test]
    fn generate_ok_on_test_msgs() {
        let assets_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/test_msgs");

        find_and_generate_ros_messages(vec![assets_path.into()]);
    }

}
