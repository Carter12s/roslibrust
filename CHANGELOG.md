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
 - Support for default field values in ROS2 messages

### Fixed
 - Bug causing single quoted string constants in message files to not be parsed correctly
 - Bug causing float constants in message files to cause compiler errors because `f32 = 0;` is not allowed in rust

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
