use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use roslibrust_rospack as rospack;
use std::collections::{BTreeMap, VecDeque};
use std::path::PathBuf;
use std::str::FromStr;
use syn::parse_quote;

mod parse;
use parse::*;

pub fn find_and_generate_ros_messages(additional_search_paths: Vec<PathBuf>) -> TokenStream {
    let mut search_paths = rospack::get_search_paths();
    search_paths.extend(additional_search_paths.into_iter());
    let mut packages = rospack::crawl(search_paths);
    packages.dedup_by(|a, b| a.name == b.name);
    let message_files = packages
        .iter()
        .map(|pkg| {
            rospack::get_message_files(pkg)
                .unwrap()
                .into_iter()
                .map(|path| (pkg.name.clone(), path))
        })
        .flatten()
        .collect::<Vec<_>>();
    let messages = parse_message_files(message_files).unwrap();
    let struct_definitions = messages
        .into_iter()
        .map(|(_, message)| generate_struct(message))
        .collect::<Vec<TokenStream>>();

    quote! {
        #![allow(non_snake_case)]

        #(#struct_definitions)*

    }
}

fn parse_message_files(
    msg_paths: Vec<(String, PathBuf)>,
) -> std::io::Result<BTreeMap<String, MessageFile>> {
    let mut paths = msg_paths
        .into_iter()
        .map(|(pkg, msg_path)| (pkg, msg_path, 0i32))
        .collect::<VecDeque<_>>();
    let mut message_map = BTreeMap::new();

    const MAX_PARSE_ITER_LIMIT: usize = 2048;
    let mut parse_iter_counter = 0;

    while let Some((pkg, msg_path, seen_count)) = paths.pop_front() {
        let msg_definition = std::fs::read_to_string(&msg_path)?;
        let msg_name = msg_path.file_stem().unwrap().to_str().unwrap();
        let msg = parse_ros_message_file(msg_definition.clone(), msg_name.to_string(), &pkg);

        if seen_count > 5 {
            msg.fields.iter().for_each(|field| {
                if ROS_TYPE_TO_RUST_TYPE_MAP.contains_key(field.field_type.field_type.as_str()) {
                    println!("RESOLVED PRIMITIVE: {}", field.field_type.field_type);
                } else if message_map.contains_key(&field.field_type.field_type) {
                    println!("RESOLVED MSG: {}", field.field_type.field_type);
                } else {
                    println!("UNRESOLVED: {}", field.field_type.field_type);
                }
            });
            panic!("Dependency loop discovered, {} message: {pkg}/{msg_name}: \nMessage Definition:\n{msg_definition}\n\nParsed: {msg:#?}", paths.len());
        }

        // Check if each dependency of the message is a primitive or has been resolved
        let fully_resolved = msg.fields.iter().all(|field| {
            ROS_TYPE_TO_RUST_TYPE_MAP.contains_key(field.field_type.field_type.as_str())
                || message_map.contains_key(&format!(
                    "{}/{}",
                    field.field_type.package_name.as_ref().unwrap(),
                    &field.field_type.field_type
                ))
        });

        if fully_resolved {
            // We can remove it if the whole tree is understood
            message_map.insert(
                format!("{}/{}", pkg.clone(), msg_name),
                replace_ros_types_with_rust_types(msg),
            );
        } else {
            // Otherwise put it back in the queue
            paths.push_back((pkg, msg_path, seen_count + 1));
        }

        parse_iter_counter += 1;
        if parse_iter_counter >= MAX_PARSE_ITER_LIMIT {
            panic!("Possibly encountered infinite loop");
        }
    }

    Ok(message_map)
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
        .map(|field| {
            let field_type = TokenStream::from_str(field.field_type.field_type.as_str()).unwrap();

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
            let constant_type = if constant.constant_type == "std::string::String" {
                String::from("&'static str")
            } else {
                constant.constant_type
            };
            let constant_type = TokenStream::from_str(constant_type.as_str()).unwrap();
            let constant_value = TokenStream::from_str(constant.constant_value.as_str()).unwrap();
            quote! { pub const #constant_name: #constant_type = #constant_value; }
        })
        .collect::<Vec<TokenStream>>();

    let struct_name = format_ident!("{}", msg.name);
    let ros_type_name = format!("{}/{}", msg.package, struct_name);

    quote! {
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
