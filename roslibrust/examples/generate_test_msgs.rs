use std::module_path;
use std::path::Path;

use log::*;

use roslibrust::message_gen;
use roslibrust::message_gen::MsgSource;
use roslibrust::util;

/// Example showing manual execution of the message generation code
/// This example is actually used to generate the test_msgs file stored with the source code
/// and used for testing purposes.
fn main() {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .without_timestamps() // Required for running in wsl2
        .init()
        .unwrap();

    let source_path = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/test_msgs"));
    let dest_path = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/src/test_msgs.rs"));
    // Here we use a utility function to find message files, but then convert to just regular paths
    let files = util::recursive_find_msg_files(source_path)
        .into_iter()
        .map(|f| f.path)
        .collect();
    info!("Running on files: {:?}", files);

    let opts = message_gen::MessageGenOpts::new(files)
        .destination(dest_path)
        .format(true)
        .std_msgs_source(MsgSource::Internal)
        .local(true);
    message_gen::generate_messages(&opts).expect("Failed to generate");
}
