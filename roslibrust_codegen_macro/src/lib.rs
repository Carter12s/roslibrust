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
            if let Ok(_) = input.parse::<Token![,]>() {
                continue;
            } else {
                break;
            }
        }
        Ok(Self { paths })
    }
}

#[proc_macro]
pub fn find_and_generate_ros_messages(input_stream: TokenStream) -> TokenStream {
    let RosLibRustMessagePaths { paths } =
        parse_macro_input!(input_stream as RosLibRustMessagePaths);
    roslibrust::find_and_generate_ros_messages(paths).into()
}
