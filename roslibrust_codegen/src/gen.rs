use proc_macro2::TokenStream;
use quote::{format_ident, quote, ToTokens};
use serde::de::DeserializeOwned;
use std::str::FromStr;
use syn::parse_quote;

use crate::parse::convert_ros_type_to_rust_type;
use crate::utils::RosVersion;
use crate::{bail, Error};
use crate::{ConstantInfo, FieldInfo, MessageFile, RosLiteral, ServiceFile};

fn derive_attrs() -> Vec<syn::Attribute> {
    vec![
        parse_quote! { #[derive(::roslibrust_codegen::Deserialize)] },
        parse_quote! { #[derive(::roslibrust_codegen::Serialize)] },
        parse_quote! { #[derive(::roslibrust_codegen::SmartDefault)] },
        parse_quote! { #[derive(Debug)] },
        parse_quote! { #[derive(Clone)] },
        parse_quote! { #[derive(PartialEq)] },
        parse_quote! { #[serde(crate = "::roslibrust_codegen::serde")] },
    ]
}

/// Generates the service for a given service file
/// The service definition defines a struct representing the service an an implementation
/// of the RosServiceType trait for that struct
pub fn generate_service(service: ServiceFile) -> Result<TokenStream, Error> {
    let service_type_name = service.get_full_name();
    let service_md5sum = service.md5sum;
    let struct_name = format_ident!("{}", service.parsed.name);
    let request_name = format_ident!("{}", service.parsed.request_type.name);
    let response_name = format_ident!("{}", service.parsed.response_type.name);

    let request_msg = generate_struct(service.request)?;
    let response_msg = generate_struct(service.response)?;
    Ok(quote! {

        #request_msg
        #response_msg

        #[allow(dead_code)]
        pub struct #struct_name {

        }
        impl ::roslibrust_common::RosServiceType for #struct_name {
            const ROS_SERVICE_NAME: &'static str = #service_type_name;
            const MD5SUM: &'static str = #service_md5sum;
            type Request = #request_name;
            type Response = #response_name;
        }
    })
}

/// Turns a string into a TokenStream that represents a raw string literal of the string
pub fn generate_raw_string_literal(value: &str) -> TokenStream {
    let wrapped = format!("r#\"{}\"#", value);
    TokenStream::from_str(&wrapped).unwrap()
}

pub fn generate_struct(msg: MessageFile) -> Result<TokenStream, Error> {
    let ros_type_name = msg.get_full_name();
    let attrs = derive_attrs();
    let fields = msg
        .parsed
        .fields
        .into_iter()
        .map(|field| {
            generate_field_definition(
                field,
                &msg.parsed.package,
                msg.parsed.version.unwrap_or(RosVersion::ROS1),
            )
        })
        .collect::<Result<Vec<TokenStream>, _>>()?;

    let constants = msg
        .parsed
        .constants
        .into_iter()
        .map(|constant| {
            generate_constant_field_definition(
                constant,
                msg.parsed.version.unwrap_or(RosVersion::ROS1),
            )
        })
        .collect::<Result<Vec<TokenStream>, _>>()?;

    let struct_name = format_ident!("{}", msg.parsed.name);
    let md5sum = msg.md5sum;
    let definition = msg.definition;

    // Raw here is only used to make the generated code look better.
    let raw_message_definition = generate_raw_string_literal(&definition);

    let mut base = quote! {
        #[allow(non_snake_case)]
        #(#attrs )*
        pub struct #struct_name {
            #(#fields )*
        }

        impl ::roslibrust_common::RosMessageType for #struct_name {
            const ROS_TYPE_NAME: &'static str = #ros_type_name;
            const MD5SUM: &'static str = #md5sum;
            const DEFINITION: &'static str = #raw_message_definition;
        }
    };

    // Only if we have constants append the impl
    if !constants.is_empty() {
        base.extend(quote! {
            #[allow(unused)]
            impl #struct_name {
                #(#constants )*
            }
        });
    }
    Ok(base)
}

fn generate_field_definition(
    field: FieldInfo,
    msg_pkg: &str,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    let rust_field_type = match field.field_type.package_name {
        Some(ref pkg) => {
            if pkg.as_str() == msg_pkg {
                format!("self::{}", field.field_type.field_type)
            } else {
                format!("{}::{}", pkg, field.field_type.field_type)
            }
        }
        None => convert_ros_type_to_rust_type(version, &field.field_type.field_type)
            .ok_or(Error::new(format!("No Rust type for {}", field.field_type)))?
            .to_owned(),
    };
    // Wrap type in appropriate Vec or array wrapper based on array information
    let rust_field_type = match field.field_type.array_info {
        Some(None) => {
            format!("::std::vec::Vec<{rust_field_type}>")
        }
        Some(Some(fixed_length)) => format!("[{rust_field_type}; {fixed_length}]"),
        None => rust_field_type,
    };
    let rust_field_type = TokenStream::from_str(rust_field_type.as_str()).expect(
        "Somehow we generate a rust type that isn't valid rust syntax. This should not happen!",
    );

    let field_name = format_ident!("r#{}", field.field_name);
    let property_line = quote! { pub #field_name: #rust_field_type, };
    let default_line = if let Some(ref default_val) = field.default {
        let default_val = ros_literal_to_rust_literal(
            &field.field_type.field_type,
            default_val,
            field.field_type.array_info,
            version,
        )?;
        if field.field_type.array_info.is_some() {
            // For vectors use smart_defaults "dynamic" style
            quote! {
                #[default(_code = #default_val)]
            }
        } else {
            // For non vectors use smart_default's constant style
            quote! {
              #[default(#default_val)]
            }
        }
    } else {
        // Okay this is messy, so default isn't defined for fixed sized arrays > 32 in length
        // so we have to manually provide a default if one isn't provided for arrays that large
        if let Some(Some(fixed_array_length)) = field.field_type.array_info {
            if fixed_array_length > 32 {
                // Doing some evil indirection here with the _code directive and Deafult::default()
                // to generate the default value for a single member of the array type, and then
                // broadcasting that with an array constant. I can't believe this works...
                let default_str = format!("[Default::default(); {fixed_array_length}]");
                quote! { #[default(_code = #default_str)]}
            } else {
                quote! {}
            }
        } else {
            quote! {}
        }
    };
    // This is the largest size of fixed sized array for which macros automatically implement traits
    // Until serde supports const generics we need to use serde_big_array for fixed size arrays
    // Larger than 32.
    const MAX_FIXED_ARRAY_LEN: usize = 32;
    let serde_line = match field.field_type.array_info {
        Some(None) => {
            // Special case for Vec<u8>, which massively benefit from optimizations in serde_bytes
            // This makes deserializing an Image ~97% faster
            if field.field_type.field_type == "uint8" {
                quote! { #[serde(with = "::roslibrust_codegen::serde_bytes")] }
            } else {
                quote! {}
            }
        }
        Some(Some(fixed_array_len)) if fixed_array_len > MAX_FIXED_ARRAY_LEN => {
            quote! { #[serde(with = "::roslibrust_codegen::BigArray")] }
        }
        _ => quote! {},
    };
    Ok(quote! {
        #default_line
        #serde_line
        #property_line
    })
}

fn generate_constant_field_definition(
    constant: ConstantInfo,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    let constant_name = format_ident!("r#{}", constant.constant_name);
    let constant_rust_type = convert_ros_type_to_rust_type(version, &constant.constant_type)
        .ok_or(Error::new(format!(
            "A constant was detected {constant:?} for which no valid rust type was found."
        )))?;
    let constant_rust_type = if constant_rust_type == "::std::string::String" {
        String::from("&'static str")
    } else {
        // Oof it's ugly in here
        constant_rust_type.to_owned()
    };
    let constant_rust_type = TokenStream::from_str(constant_rust_type.as_str()).map_err(|err| {
        Error::with(
            format!("Failed to parse {constant_rust_type} into valid rust syntax").as_str(),
            err,
        )
    })?;
    let constant_value = ros_literal_to_rust_literal(
        &constant.constant_type,
        &constant.constant_value,
        None,
        version,
    )?;
    Ok(quote! { pub const #constant_name: #constant_rust_type = #constant_value; })
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

fn ros_literal_to_rust_literal(
    ros_type: &str,
    literal: &RosLiteral,
    array_info: Option<Option<usize>>,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    // TODO: The naming of all the functions under this tree seems inaccurate
    parse_ros_value(ros_type, &literal.inner, array_info, version)
}

// Converts a ROS string to a literal value
// Not intended to be called directly, but only via parse_ros_value.
// Wraps a serde_json deserialize call with our style of error handling.
fn generic_parse_value<T: DeserializeOwned + ToTokens + std::fmt::Debug>(
    value: &str,
    is_vec: bool,
) -> Result<TokenStream, Error> {
    if is_vec {
        let parsed: Vec<T> = serde_json::from_str(value).map_err(|e|
            Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to {}", std::any::type_name::<T>()).as_str(), e)
        )?;
        let vec_str = format!("vec!{parsed:?}");
        Ok(quote! { #vec_str })
    } else {
        let parsed: T = serde_json::from_str(value).map_err(|e|
            Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to {}", std::any::type_name::<T>()).as_str(), e)
        )?;
        Ok(quote! { #parsed })
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
fn parse_ros_value(
    ros_type: &str,
    value: &str,
    array_info: Option<Option<usize>>,
    version: RosVersion,
) -> Result<TokenStream, Error> {
    let is_vec = array_info.is_some();
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
                let parsed: Vec<String> = serde_json::from_str(value).map_err(|e|
                    Error::with(format!("Failed to parse a literal value in a message file to the corresponding rust type: {value} to Vec<String>").as_str(), e)
                )?;
                let vec_str = format!("{parsed:?}.iter().map(|x| x.to_string()).collect()");
                Ok(quote! { #vec_str })
            } else {
                match version {
                    RosVersion::ROS1 => {
                        // For ROS1 then entire contents except for leading and trailing whitespace are used
                        let value = value.trim();
                        Ok(quote! { #value })
                    }
                    RosVersion::ROS2 => {
                        // For ROS2 value must be in quotes, and either single or double quotes are okay
                        // Strings are no escaped (we think)
                        let value = value.trim();
                        if value.len() < 2 {
                            // TODO would like to provide source file and callsite information for debug, but pretty hard to
                            // Maybe we wrap that in calling function?
                            bail!("String constant must at least include start and end quotes, cannot be empty: {value}");
                        }
                        let first = value.chars().nth(0).unwrap(); // Unwrap is okay due to previous length check
                        let last = value.chars().last().unwrap(); // Unwrap is okay due to previous length check
                        if first != last || !(first == '\'' || first == '\"') {
                            bail!("ROS2 String constant was found that was not enclosed in single or double quotes: {value}");
                        }
                        let parsed = value[1..value.len() - 1].to_string();
                        Ok(quote! { #parsed })
                    }
                }
            }
        }
        _ => {
            bail!("Found default for type which does not support default: {ros_type}");
        }
    }
}
