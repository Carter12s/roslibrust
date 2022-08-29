use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use roslibrust_rospack as rospack;
use std::collections::{BTreeMap, VecDeque};
use std::path::PathBuf;
use std::str::FromStr;
use syn::parse_quote;

mod parse;
use parse::*;

/// Searches a list of paths for ROS packages and generates struct definitions
/// and implementations for message files and service files in packages it finds.
///
/// * `additional_search_paths` - A list of additional paths to search beyond those
/// found in ROS_PACKAGE_PATH environment variable.
///
pub fn find_and_generate_ros_messages(additional_search_paths: Vec<PathBuf>) -> TokenStream {
    let mut search_paths = rospack::get_search_paths();
    search_paths.extend(additional_search_paths.into_iter());
    let mut packages = rospack::crawl(search_paths);
    packages.dedup_by(|a, b| a.name == b.name);
    let mut message_files = packages
        .iter()
        .map(|pkg| {
            rospack::get_message_files(pkg)
                .unwrap_or_else(|err| {
                    eprintln!(
                        "Unable to get paths to message files for {}: {}",
                        pkg.name, err
                    );
                    // Return an empty vec so that one package doesn't necessarily fail the process
                    vec![]
                })
                .into_iter()
                .map(|path| (pkg.name.clone(), path))
        })
        .flatten()
        .collect::<Vec<_>>();
    let service_files = packages
        .iter()
        .map(|pkg| {
            rospack::get_service_files(pkg)
                .unwrap_or_else(|err| {
                    eprintln!(
                        "Unable to get paths to service files for {}: {}",
                        pkg.name, err
                    );
                    // Return an empty vec so that one package doesn't necessarily fail the process
                    vec![]
                })
                .into_iter()
                .map(|path| (pkg.name.clone(), path))
        })
        .flatten()
        .collect::<Vec<_>>();
    message_files.extend_from_slice(&service_files[..]);
    let messages = parse_message_files(message_files).unwrap();
    let mut modules_to_struct_definitions: BTreeMap<String, Vec<TokenStream>> = BTreeMap::new();

    messages.into_iter().for_each(|(_, message)| {
        let pkg_name = message.package.clone();
        let definition = generate_struct(message);
        if let Some(entry) = modules_to_struct_definitions.get_mut(&pkg_name) {
            entry.push(definition);
        } else {
            modules_to_struct_definitions.insert(pkg_name, vec![definition]);
        }
    });
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

fn parse_message_files(
    msg_paths: Vec<(String, PathBuf)>,
) -> std::io::Result<BTreeMap<String, MessageFile>> {
    let mut parsed_messages = VecDeque::new();
    for (pkg, path) in msg_paths {
        let contents = std::fs::read_to_string(&path)?;
        let name = path.file_stem().unwrap().to_str().unwrap();
        match path.extension().unwrap().to_str().unwrap() {
            "srv" => {
                parse_ros_service_file(contents, name.to_string(), &pkg)
                    .into_iter()
                    .for_each(|(parsed, unparsed)| {
                        parsed_messages.push_back(MessageMetadata {
                            package: pkg.clone(),
                            path: path.clone(),
                            seen_count: 0,
                            parsed,
                            unparsed,
                        });
                    });
            }
            "msg" => {
                let msg = parse_ros_message_file(contents.clone(), name.to_string(), &pkg);
                parsed_messages.push_back(MessageMetadata {
                    package: pkg.clone(),
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
    Ok(resolve_message_dependencies(parsed_messages))
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
                format!("std::vec::Vec<{}>", field.field_type.field_type)
            } else {
                field.field_type.field_type
            };
            let field_type = TokenStream::from_str(field_type.as_str()).unwrap();

            let field_name = if field.field_name.as_str() == "type" {
                format_ident!("r#{}", field.field_name)
            } else {
                format_ident!("{}", field.field_name)
            };
            quote! { pub #field_name: #field_type, }
        })
        .collect::<Vec<TokenStream>>();

    let constants = msg
        .constants
        .into_iter()
        .map(|constant| {
            let constant_name = if constant.constant_name.as_str() == "type" {
                format_ident!("r#{}", constant.constant_name)
            } else {
                format_ident!("{}", constant.constant_name)
            };
            let (constant_type, constant_value) = if constant.constant_type == "std::string::String" {
                let constant_value = constant.constant_value;
                (String::from("&'static str"), quote!{ #constant_value })
            } else {
                (constant.constant_type, TokenStream::from_str(constant.constant_value.as_str()).unwrap())
            };
            let constant_type = TokenStream::from_str(constant_type.as_str()).unwrap();
            quote! { pub const #constant_name: #constant_type = #constant_value; }
        })
        .collect::<Vec<TokenStream>>();

    let struct_name = format_ident!("{}", msg.name);
    let ros_type_name = format!("{}/{}", msg.package, struct_name);

    quote! {
        #[allow(non_snake_case)]
        #(#attrs )*
        pub struct #struct_name {
            #(#fields )*
        }

        impl ::roslibrust::RosMessageType for #struct_name {
            const ROS_TYPE_NAME: &'static str = #ros_type_name;
        }

        impl #struct_name {
            #(#constants )*
        }

    }
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
