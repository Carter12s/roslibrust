use roslibrust_codegen_macro::find_and_generate_ros_messages;

//find_and_generate_ros_messages!("..");

fn main() {
    let tokens = roslibrust_codegen::find_and_generate_ros_messages(vec!["..".into()]);
    println!("{}", tokens);
}
