use proc_macro2::TokenStream;
use roslibrust_rospack as rospack;
use std::collections::{BTreeMap, VecDeque};
use std::ops::RangeBounds;
use std::path::PathBuf;
use syn::parse_quote;

mod parse;
use parse::*;

static PRIMITIVE_ROS_TYPES: &[&'static str] = &["bool", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "string", "time", "duration"];

enum SourceElement {
    StructSource(syn::DataStruct),
    ImplBlockSource(()),
}

pub fn find_and_generate_ros_messages(additional_search_paths: Vec<PathBuf>) -> TokenStream {
    let mut search_paths = rospack::get_search_paths();
    search_paths.extend(additional_search_paths.into_iter());
    let mut packages = rospack::crawl(search_paths);
    packages.dedup_by(|a, b| a.name == b.name);
    let message_files = packages
        .iter()
        .map(|pkg| rospack::get_message_files(pkg).unwrap().into_iter().map(|path| (pkg.name.clone(), path)))
        .flatten()
        .collect::<Vec<_>>();
    let messages = parse_message_files(message_files).unwrap();
    "".parse().unwrap()
}

fn parse_message_files(msg_paths: Vec<(String, PathBuf)>) -> std::io::Result<BTreeMap<String, MessageFile>> {
    let mut paths = VecDeque::from(msg_paths);
    let mut message_map = BTreeMap::new();

    const MAX_PARSE_ITER_LIMIT: usize = 2048;
    let mut parse_iter_counter = 0; 

    while let Some((pkg, msg_path)) = paths.pop_front() {
        let msg_definition = std::fs::read_to_string(&msg_path)?;
        let msg_name = msg_path.file_stem().unwrap().to_str().unwrap();
        let msg = parse_ros_message_file(msg_definition, msg_name.to_string(), &pkg);

        // Check if each dependency of the message is a primitive or has been resolved
        let fully_resolved = msg.fields.iter().all(|field| {
            PRIMITIVE_ROS_TYPES.iter().find(|&primitive| primitive == &field.field_type.field_type).is_some()
            || message_map.contains_key(&field.field_type.field_type)
        });

        if fully_resolved {
            // We can remove it if the whole tree is understood
            message_map.insert(format!("{}/{}", pkg.clone(), msg_name), msg);
        } else {
            // Otherwise put it back in the queue
            paths.push_back((pkg, msg_path));
        }

        parse_iter_counter += 1;
        if parse_iter_counter >= MAX_PARSE_ITER_LIMIT {
            panic!("Possibly encountered infinite loop");
        }
    }

    Ok(message_map)
}

fn generate_source_for_message() {
    unimplemented!()
}

fn derive_attrs() -> Vec<syn::Attribute> {
    vec![
        parse_quote! { #[derive(Deserialize)] },
        parse_quote! { #[derive(Serialize)] },
        parse_quote! { #[derive(Debug)] },
        parse_quote! { #[derive(Default)] },
        parse_quote! { #[derive(Clone)] },
        parse_quote! { #[derive(PartialEq)] },
    ]
}
