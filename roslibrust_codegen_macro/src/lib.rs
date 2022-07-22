use proc_macro::TokenStream;

#[proc_macro]
pub fn find_and_generate_ros_messages(input_stream: TokenStream) -> TokenStream {
    // 1. Parse the input stream for a potential additional path to search
    // 2. Grab ROS_PACKAGE_PATH from rospack
    // 3. Search paths and get a list of .msg files
    // 4. Parse each msg file into a Message type with parameter information
    // 5. For each Message, generate a struct
    // 6. Combine all of the generated Messages into a single TokenStream
    // All of the above steps can be done in a single function
    println!("{:?}", input_stream);
    "extern crate self as roslibrust_special_internal_crate_name;\n\n".parse().unwrap()
}