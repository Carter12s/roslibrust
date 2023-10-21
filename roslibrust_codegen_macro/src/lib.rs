use proc_macro::TokenStream;
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Token};

struct RosLibRustMessagePaths {
    paths: Vec<std::path::PathBuf>,
}

/// Parses a comma-separated list of str literals specifying paths.
impl Parse for RosLibRustMessagePaths {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut paths = vec![];
        while let Ok(path) = input.parse::<syn::LitStr>() {
            paths.push(path.value().into());
            if input.parse::<Token![,]>().is_ok() {
                continue;
            } else {
                break;
            }
        }
        Ok(Self { paths })
    }
}

/// Given a list of paths, generates struct definitions and trait impls for any
/// ros messages found within those paths.
/// Paths are relative to where rustc is being invoked from your mileage may vary.
///
/// In addition to provided paths, this will search paths found in the environment
/// variable ROS_PACKAGE_PATH.
#[proc_macro]
pub fn find_and_generate_ros_messages(input_stream: TokenStream) -> TokenStream {
    // Note: there is not currently a way for proc_macros to indicate that they need to be re-generated
    // We discard the "dependent_paths" part of the response here...
    let RosLibRustMessagePaths { paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);
    match roslibrust_codegen::find_and_generate_ros_messages(paths) {
        Ok((source, _dependent_paths)) => source.into(),
        Err(e) => {
            let error_msg = e.to_string();
            quote::quote!(compile_error!(#error_msg);).into()
        }
    }
}

/// Does the same as find_and_generate_ros_messages, but interprets relative paths
/// as relative to the root of this crate. No idea if this is useful, but I needed it!
#[proc_macro]
pub fn find_and_generate_ros_messages_relative_to_manifest_dir(
    input_stream: TokenStream,
) -> TokenStream {
    let RosLibRustMessagePaths { mut paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);

    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).expect("Failed to set working dir");
    for path in &mut paths {
        *path = path.canonicalize().unwrap_or_else(|err| {
            panic!("Failed to canonicalize path {path:?}: {}", err.to_string())
        });
    }

    match roslibrust_codegen::find_and_generate_ros_messages(paths) {
        // Note: there is not currently a way for proc_macros to indicate that they need to be re-generated
        // We discard the "dependent_paths" part of the response here...
        Ok((source, _dependent_paths)) => source.into(),
        Err(e) => {
            let error_msg = e.to_string();
            quote::quote!(compile_error!(#error_msg);).into()
        }
    }
}

/// Similar to `find_and_generate_ros_messages`, but does not search the
/// `ROS_PACKAGE_PATH` environment variable paths (useful in some situations).
#[proc_macro]
pub fn find_and_generate_ros_messages_without_ros_package_path(
    input_stream: TokenStream,
) -> TokenStream {
    let RosLibRustMessagePaths { paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);
    match roslibrust_codegen::find_and_generate_ros_messages_without_ros_package_path(paths) {
        // Note: there is not currently a way for proc_macros to indicate that they need to be re-generated
        // We discard the "dependent_paths" part of the response here...
        Ok((source, _dependent_paths)) => source.into(),
        Err(e) => {
            let error_msg = e.to_string();
            quote::quote!( compile_error!(#error_msg); ).into()
        }
    }
}
