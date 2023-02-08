use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use std::collections::HashMap;
use std::str::FromStr;
use syn::parse_quote;

use crate::parse::{
    ParsedMessageFile, ParsedServiceFile, ROS_2_TYPE_TO_RUST_TYPE_MAP, ROS_TYPE_TO_RUST_TYPE_MAP,
};
use crate::utils::RosVersion;

fn derive_attrs() -> Vec<syn::Attribute> {
    // TODO we should look into using $crate here...
    // The way we're currently doing it leaks a dependency on these crates to users...
    // However using $crate breaks the generated code in non-macro usage
    // Pass a flag in "if_macro"?
    vec![
        parse_quote! { #[derive(::serde::Deserialize)] },
        parse_quote! { #[derive(::serde::Serialize)] },
        parse_quote! { #[derive(::smart_default::SmartDefault)] },
        parse_quote! { #[derive(Debug)] },
        parse_quote! { #[derive(Clone)] },
        parse_quote! { #[derive(PartialEq)] },
    ]
}

/// Generates the service for a given service file
/// The service definition defines a struct representing the service an an implementation
/// of the RosServiceType trait for that struct
pub fn generate_service(service: ParsedServiceFile) -> TokenStream {
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

pub fn generate_struct(msg: ParsedMessageFile) -> TokenStream {
    let msg = replace_ros_types_with_rust_types(msg);
    let attrs = derive_attrs();
    let fields = msg
        .fields
        .into_iter()
        .map(|mut field| {
            field.field_type.field_type = match field.field_type.package_name {
                Some(ref pkg) => {
                    if pkg.as_str() == msg.package.as_str() {
                        format!("self::{}", field.field_type.field_type)
                    } else {
                        format!("{}::{}", pkg, field.field_type.field_type)
                    }
                }
                None => field.field_type.field_type.clone(),
            };
            let field_type = if field.field_type.is_vec {
                format!("::std::vec::Vec<{}>", field.field_type.field_type)
            } else {
                field.field_type.field_type.clone()
            };
            let field_type = TokenStream::from_str(field_type.as_str()).unwrap();

            let field_name = format_ident!("r#{}", field.field_name);
            if let Some(ref default_val) = field.default {
                if field.field_type.is_vec {
                    // For vectors use smart_defaults "dynamic" style
                    quote! {
                        #[default(_code = #default_val)]
                        pub #field_name: #field_type,
                    }
                } else {
                    // For non vectors use smart_default's constant style
                    quote! {
                      #[default(#default_val)]
                      pub #field_name: #field_type,
                    }
                }
            } else {
                quote! { pub #field_name: #field_type, }
            }
        })
        .collect::<Vec<TokenStream>>();

    let constants = msg
        .constants
        .into_iter()
        .map(|constant| {
            let constant_name = format_ident!("r#{}", constant.constant_name);
            let constant_type = if constant.constant_type == "::std::string::String" {
                String::from("&'static str")
            } else {
                constant.constant_type
            };
            let constant_type = TokenStream::from_str(constant_type.as_str()).unwrap();
            let constant_value = constant.constant_value;
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

pub fn generate_mod(
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

pub fn replace_ros_types_with_rust_types(mut msg: ParsedMessageFile) -> ParsedMessageFile {
    const INTERNAL_STD_MSGS: [&str; 1] = ["Header"];

    // Select which type conversion map to use depending on ros version
    let prop_map: &HashMap<&'static str, &'static str> = match msg.version {
        Some(RosVersion::ROS1) => &ROS_TYPE_TO_RUST_TYPE_MAP,
        Some(RosVersion::ROS2) => &ROS_2_TYPE_TO_RUST_TYPE_MAP,
        None => {
            // If we couldn't determine the package type, assume ROS1 for now
            &ROS_TYPE_TO_RUST_TYPE_MAP
        }
    };

    msg.constants = msg
        .constants
        .into_iter()
        .map(|mut constant| {
            if prop_map.contains_key(constant.constant_type.as_str()) {
                constant.constant_type = prop_map
                    .get(constant.constant_type.as_str())
                    .unwrap()
                    .to_string();
                // We do not need to consider the package for constants as they're required
                // to be built-in types other than Time and Duration (I think Header is not
                // technically built-in)
            }
            constant
        })
        .collect();
    msg.fields = msg
        .fields
        .into_iter()
        .map(|mut field| {
            field.field_type.field_type = prop_map
                .get(field.field_type.field_type.as_str())
                .unwrap_or(&field.field_type.field_type.as_str())
                .to_string();
            for std_msg in INTERNAL_STD_MSGS {
                if field.field_type.field_type.as_str() == std_msg {
                    field.field_type.package_name = Some("std_msgs".into());
                }
            }
            field
        })
        .collect();
    msg
}
