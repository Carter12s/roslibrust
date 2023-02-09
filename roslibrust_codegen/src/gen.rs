use proc_macro2::TokenStream;
use quote::{format_ident, quote, ToTokens};
use serde::de::DeserializeOwned;
use std::str::FromStr;
use syn::parse_quote;

use crate::parse::{
    convert_ros_type_to_rust_type, ConstantInfo, FieldInfo,
    ParsedMessageFile, ParsedServiceFile, RosLiteral,
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
    //let msg = replace_ros_types_with_rust_types(msg);
    let attrs = derive_attrs();
    let fields = msg
        .fields
        .into_iter()
        .map(|field| {
            generate_field_definition(field, &msg.package, msg.version.unwrap_or(RosVersion::ROS1))
        })
        .collect::<Vec<TokenStream>>();

    let constants = msg
        .constants
        .into_iter()
        .map(|constant| {
            generate_constant_field_definition(constant, msg.version.unwrap_or(RosVersion::ROS1))
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

fn generate_field_definition(field: FieldInfo, msg_pkg: &str, version: RosVersion) -> TokenStream {
    let rust_field_type = match field.field_type.package_name {
        Some(ref pkg) => {
            if pkg.as_str() == msg_pkg {
                format!("self::{}", field.field_type.field_type)
            } else {
                format!("{}::{}", pkg, field.field_type.field_type)
            }
        }
        None => convert_ros_type_to_rust_type(version, &field.field_type.field_type)
            .expect(&format!("No Rust type for {}", field.field_type))
            .to_owned(),
    };
    let rust_field_type = if field.field_type.is_vec {
        format!("::std::vec::Vec<{}>", rust_field_type)
    } else {
        rust_field_type
    };
    let rust_field_type = TokenStream::from_str(rust_field_type.as_str()).unwrap();

    let field_name = format_ident!("r#{}", field.field_name);
    if let Some(ref default_val) = field.default {
        let default_val = ros_literal_to_rust_literal(
            &field.field_type.field_type,
            default_val,
            field.field_type.is_vec,
        );
        if field.field_type.is_vec {
            // For vectors use smart_defaults "dynamic" style
            quote! {
                #[default(_code = #default_val)]
                pub #field_name: #rust_field_type,
            }
        } else {
            // For non vectors use smart_default's constant style
            quote! {
              #[default(#default_val)]
              pub #field_name: #rust_field_type,
            }
        }
    } else {
        quote! { pub #field_name: #rust_field_type, }
    }
}

fn generate_constant_field_definition(constant: ConstantInfo, version: RosVersion) -> TokenStream {
    let constant_name = format_ident!("r#{}", constant.constant_name);
    let constant_rust_type =
        convert_ros_type_to_rust_type(version, &constant.constant_type).unwrap();
    let constant_rust_type = if constant_rust_type == "::std::string::String" {
        String::from("&'static str")
    } else {
        // Oof it's ugly in here
        constant_rust_type.to_owned()
    };
    let constant_rust_type = TokenStream::from_str(constant_rust_type.as_str()).unwrap();
    let constant_value =
        ros_literal_to_rust_literal(&constant.constant_type, &constant.constant_value, false);

    quote! { pub const #constant_name: #constant_rust_type = #constant_value; }
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
    //const INTERNAL_STD_MSGS: [&str; 1] = ["Header"];

    // If we couldn't determine the package type, assume ROS1 for now
    let pkg_version = msg.version.unwrap_or(RosVersion::ROS1);

    msg.constants = msg
        .constants
        .into_iter()
        .map(|mut constant| {
            if let Some(rust_type) =
                convert_ros_type_to_rust_type(pkg_version, constant.constant_type.as_str())
            {
                constant.constant_type = rust_type.to_owned();
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
            if let Some(rust_type) =
                convert_ros_type_to_rust_type(pkg_version, field.field_type.field_type.as_str())
            {
                field.field_type.field_type = rust_type.to_owned();
            }
            // for std_msg in INTERNAL_STD_MSGS {
            //     if field.field_type.field_type.as_str() == std_msg {
            //         field.field_type.package_name = Some("std_msgs".into());
            //     }
            // }
            field
        })
        .collect();
    msg
}

fn ros_literal_to_rust_literal(ros_type: &str, literal: &RosLiteral, is_vec: bool) -> TokenStream {
    // TODO: The naming of all the functions under this tree seems inaccurate
    parse_ros_value(ros_type, &literal.inner, is_vec)
}

// Converts a ROS string to a literal value
// Not intended to be called directly, but only via parse_ros_value.
// Wraps a serde_json deserialize call with our style of error handling.
fn generic_parse_value<T: DeserializeOwned + ToTokens + std::fmt::Debug>(
    value: &str,
    is_vec: bool,
) -> TokenStream {
    if is_vec {
        let parsed: Vec<T> = serde_json::from_str(value).expect(
            &format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to {}", std::any::type_name::<T>())
        );
        let vec_str = format!("vec!{parsed:?}");
        quote! { #vec_str }
    } else {
        let parsed: T = serde_json::from_str(value).expect(
            &format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to {}", std::any::type_name::<T>())
        );
        quote! { #parsed }
    }
}

/// For a given, which is either a ROS constant or default, parse the constant and convert it into a rust TokenStream
/// which represents the same literal value. This handles frustrating edge cases that are not well documented features
/// in either ROS1 or ROS2 such as:
/// - `f32[] MY_CONST_ARRAY=[0, 1]` we have to convert the value of the constant to `vec![0.0, 1.0]`
/// - `string[] names_field ['first', "second"]` we have to convert the default value to `vec!["first".to_string(), "second".to_string()]
/// Note: I could find NO documentation from ROS about what was actually legal or expected for these value expressions
/// Note: ROS says "strings are not escaped". No idea how the eff I'm actually supposed to handle that so likely bugs...
/// Note: No idea of "constant arrays" are intended to be supported in ROS...
/// `ros_type` -- Expects the string key of the determined rust type to hold the value. Should come from one of the type map constants.
/// `value` -- Expects the trimmed string containing only the value expression
/// `is_vec` -- True iff the type is an array type
/// TODO I'd like this to take FieldType, but want it to also work with constants...
fn parse_ros_value(ros_type: &str, value: &str, is_vec: bool) -> TokenStream {
    match ros_type {
        "bool" => generic_parse_value::<bool>(value, is_vec),
        "float64" => generic_parse_value::<f64>(value, is_vec),
        "float32" => generic_parse_value::<f32>(value, is_vec),
        "uint8" | "char" | "byte" => generic_parse_value::<u8>(value, is_vec),
        "int8" => generic_parse_value::<i8>(value, is_vec),
        "uint16" => generic_parse_value::<u16>(value, is_vec),
        "int16" => generic_parse_value::<i16>(value, is_vec),
        "uint32" => generic_parse_value::<u32>(value, is_vec),
        "int32" => generic_parse_value::<i32>(value, is_vec),
        "uint64" => generic_parse_value::<u64>(value, is_vec),
        "int64" => generic_parse_value::<i64>(value, is_vec),
        "string" => {
            // String is a special case because of quotes and to_string()
            if is_vec {
                // TODO there is a bug here, no idea how I should be attempting to convert / escape single quotes here...
                let parsed: Vec<String> = serde_json::from_str(value).expect(
            &format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to Vec<String>"));
                let vec_str = format!("{parsed:?}.iter().map(|x| x.to_string()).collect()");
                quote! { #vec_str }
            } else {
                // Halfass attempt to deal with ROS's string escaping / quote bullshit
                let value = &value.replace("\'", "\"");
                let parsed: String = serde_json::from_str(value).expect(&format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to String"));
                quote! { #parsed }
            }
        }
        _ => {
            panic!("Found default for type which does not support default: {ros_type}");
        }
    }
}
