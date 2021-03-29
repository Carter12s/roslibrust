use std::module_path;
use std::path::Path;

use log::*;
use simple_logger::SimpleLogger;

use roslibrust::message_gen::*;

fn main() {
    SimpleLogger::new()
        .with_level(log::LevelFilter::Debug)
        .init()
        .unwrap();
    let source_path = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/amp_msgs"));
    let dest_path = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/out"));
    let files = recursive_find_files(source_path, |e| {
        e.file_name().to_string_lossy().ends_with(".msg")
    });
    info!("Running on files: {:?}", files);

    generate_messages(&files, dest_path)
}
