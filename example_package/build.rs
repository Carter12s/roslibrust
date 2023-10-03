// This is an example / template build script that other projects looking to incorporate roslibrust into their codebase
// are recommended to adopt. Using build.rs is currently the preferred / recommended approach as there is not currently
// a mechanism for proc_macros to indicate they need to be re-run when an external file changes. With build.rs it is
// possible to robustly trigger re-builds when dependent message files are changed.
// It is highly recommended to read the Cargo Book section on build scripts before attempting this approach as some
// care is needed: https://doc.rust-lang.org/cargo/reference/build-scripts.html
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Define our search paths, for this example we're using out test_msgs and std_msgs for ros1
    // These can be pulled either from ROS_PACKAGE_PATH or given manually.
    // While ROS_PACKAGE_PATH can be very convenient, it can also be VERY confusing
    // We recommend using explicit paths only for more reliable and reproducible builds.
    let p = vec![
        "../assets/ros1_common_interfaces/std_msgs".into(),
        "../assets/ros1_test_msgs".into(),
    ];

    // Actually invoke code generation on our search paths.
    // What we get back is a TokenStream the type normally returned by a proc_macro in rust.
    // For a build.rs we typically want to serialize this data to a file for later import
    // MAJOR TODO need to incorporate error handling here when that PR is merged
    let tokens = roslibrust_codegen::find_and_generate_ros_messages_without_ros_package_path(p);

    // It is important for build scripts to only output files to OUT_DIR.
    // This guidance can be ignored for end applications. However, crates published and downloaded with cargo
    // will not work if they rely on output files to other folders.
    let out_dir = std::env::var_os("OUT_DIR").unwrap();
    let dest_path = std::path::Path::new(&out_dir).join("messages.rs");
    // Write the generate code to disk
    // Note: it will not be nicely formatted at this point which can affect readability when debugging
    std::fs::write(dest_path, tokens.to_string())?;
    // Optionally rustfmt could be invoked to format the file at this point
    // Or https://github.com/dtolnay/prettyplease used on the TokenStream ahead of writing to disk

    // If we stopped at this point, our code would still work, but Cargo would not know to rebuild
    // our package when a message file changed.
    // MAJOR TODO need to get codegen methods to return list of dependent files
    // Also probably want to merge down function names to single function with more args / builder

    Ok(())
}
