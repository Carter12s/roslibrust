use std::path::PathBuf;
use proc_macro2::TokenStream;
use syn::parse_quote;

pub fn find_and_generate_ros_messages(additional_search_paths: Vec<PathBuf>) -> TokenStream {

    "".parse().unwrap()
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