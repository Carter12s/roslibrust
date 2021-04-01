use std::module_path;
use std::path::Path;

use log::*;
use simple_logger::SimpleLogger;

use roslibrust::message_gen;
use roslibrust::util;

/// Basic example of manually calling code generation
/// Takes messages defined in /test_msgs and generates them into src
/// TODO: Use a cargo workspace to have this generate into a different crate
fn main() {
    SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .init()
        .unwrap();
    // let source_path = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/test_msgs"));
    let source_path = Path::new("/home/carter/catkin_ws/src/axon/core/amp_msgs");
    let dest_path = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/src/gen_msgs_amp.rs"));
    let files = util::recursive_find_msg_files(source_path);
    info!("Running on files: {:?}", files);

    message_gen::generate_messages(files.into_iter().map(|e| e.path).collect(), dest_path)
}
