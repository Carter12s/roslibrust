# Changelog

All notable changes to this project will be documented in this file.

## Release Instructions:

Steps:

- Starting on master
- Edit change log
- Revise the version numbers in Cargo.toml files
- Commit the changes
- Release packages in order: roslibrust_codegen -> roslibrust_codegen_macro -> roslibrust
- Push to master
- Tag and push tag

Note: need to publish with `cargo publish --all-features`

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## Current - Unreleased

### Added

- The build.rs example in example_package now correctly informs cargo of filesystem dependencies

### Fixed

### Changed

 - Removed `find_and_generate_ros_messages_relative_to_manifest_dir!` this proc_macro was changing the current working directory of the compilation job resulting in a variety of strange compilation behaviors. Build.rs scripts are recommended for use cases requiring fine
 grained control of message generation.
 - The function interface for top level generation functions in `roslibrust_codegen` have been changed to include the list of dependent
filesystem paths that should trigger re-running code generation. Note: new files added to the search paths will not be automatically detected.

## 0.8.0 - October 4th, 2023

### Added

 - Experimental support for ROS1 native communication behind the `ros1` feature flag
 - Generation of C++ source added via `roslibrust_genmsg` along with arbitrary languages via passed in templates
 - Generation of Rust source for actions
 - Example for custom generic message usage with rosbridge
 - Example for async native ROS1 listener
 - Example for async native ROS1 publisher


### Fixed

 - Incorrect handling of ROS1 message string constants

### Changed

 - `crawl` function in `roslibrust_codegen` updated to a more flexible API
 - Overhaul of error handling in roslibrust_codegen to bubble errors up, and remove use of panic! and unwrap(). Significantly better error messages should be produced from proc_macros and build.rs files. Direct usages of the API will need to be updated to handle the returned error type.
 - RosMessageType trait now has associated constants for MD5SUM and DEFINITION to enable ROS1 native support. These constants are optional at this time with a default value of "" provided.

## 0.7.0 - March 13, 2022

### Added

 - Support for default field values in ROS2 messages
 - Added public APIs for getting message data from search and for generating Rust code given message data in roslibrust_codegen
 - More useful logs available when running codegen
 - Refactor some of the public APIs and types in roslibrust_codegen (concept of `ParsedMessageFile` vs `MessageFile`)
 - Added a method `get_md5sum` to `MessageFile`
 - Additional code generation API and macro which excludes `ROS_PACKAGE_PATH`

### Fixed

- Bug causing single quoted string constants in message files to not be parsed correctly
- Bug causing float constants in message files to cause compiler errors because `f32 = 0;` is not allowed in rust
- Bug where packages were not properly de-duplicated during discovery.

### Changed

- `advertise_service` and `subscribe` methods on ClientHandle were changed from needing `&mut self` to `&self`

## 0.6.0 - December 12, 2022

### Added

- Initial support for ROS2 message generation
- Initial integration testing for ROS2 all basic functionality covered
- CI testing for Humble

### Fixed

- The generated `char` type within rust is now u8.
- Package names are now determined by the `name` tag within package.xml instead of by directory name

## 0.5.2 - October 31, 2022

### Changed

- No longer generate empty `impl` blocks from message structs that have not associated constants
- Significant improvement to documentation with expanded examples

### Fixed

- `advertise_service` no longer panics if multiple advertise attempts made to same topic

## 0.5.1 - September 18, 2022

Fix to docs.rs build.

## 0.5.0 - September 18, 2022

### Added

- Service server example

### Changed

- `Client` is now `ClientHandle`
- All identifiers in generated code are now escaped with `r#`
- `advertise_service` now returns a `ServiceHandle` which controls the lifetime of the service

### Fixed

- Fixed issue where the spin and reconnect context would never drop even if there were no more `ClientHandle`s
- Fixed parsing issue with triple dashes in comments of service files
- Fixed bug in generation where message properties or constants had names conflicting with Rust reserved keywords

## 0.4.0 - September 18, 2022

Yanked version due to failed publish

## 0.3.0 - September 18, 2022

Yanked version due to failed publish

## 0.2.0 - September 1, 2022

### Added

- Support for service servers
- Into<> helpers for Time and Duration for tokio and std

### Fixed

- Failure in message generation caused by files without extensions
- Failure in message generation caused by failing to quote around string constants
- Failure in message generation caused by bad design of integral types TimeI and DurationI

## 0.1.0 - August 28, 2022

Initial public release

## 0.0.3 - Unreleased

## 0.0.2 - Unreleased

## 0.0.1 - Unreleased
