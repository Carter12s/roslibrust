# Changelog
All notable changes to this project will be documented in this file.

## Release Intstructions:
TODO Find a better way that this:

Steps:
- Make sure change log is up to date or master
- Create new branch for the release
- Modify Cargo.toml's to point not to workspace path but to named version
- Commit changes to new branch
- Release packages in order: rospack -> codegen -> codegen\_macro -> main lib

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Current - Unreleased

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
