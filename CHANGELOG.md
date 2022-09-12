# Changelog
All notable changes to this project will be documented in this file.

## Release Intstructions:
TODO Find a better way that this:

Steps:
- Make sure change log is up to date or master
- Create new branch for the release
- Modify Cargo.toml's to point not to workspace path but to named version
- Commit changes to new branch
- Release packages in order: roslibrust -> codegen_macro

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Current - Unreleased

### Changed
 - `Client` is now `ClientHandle`
 - All identifiers in generated code are now escaped with `r#`

### Fixed
 - Fixed issue where the spin and reconnect context would never drop even if there were no more `ClientHandle`s
 - Fixed parsing issue with triple dashes in comments of service files
 - Fixed bug in generation where message properties or constants had names conflicting with Rust reserved keywords

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
