# RosLibRust
[![Noetic](https://github.com/Carter12s/roslibrust/actions/workflows/noetic.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/noetic.yml)
[![Galactic](https://github.com/Carter12s/roslibrust/actions/workflows/galactic.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/galactic.yml)
[![Humble](https://github.com/Carter12s/roslibrust/actions/workflows/humble.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/humble.yml)
[![License:MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This package aims to provide a convenient intermediary between ROS1's rosbridge and Rust similar to roslibpy and roslibjs.

Information about the protocol can be found [here](https://github.com/RobotWebTools/rosbridge_suite).

Note on documentation:
All information about the crate itself (examples, documentation, tutorials, etc.) lives in the source code and can be viewed on [docs.rs](https://docs.rs/roslibrust).
This readme is for "Meta" information about developing for the crate.

Fully Supported: Noetic, Galactic, Humble.

## Roadmap

Rough overview of the features planned to built for this crate in what order:

| Feature                      | Status                                                      |
|------------------------------|-------------------------------------------------------------|
| examples                     | ✅ |
| message_gen                  | ✅ |
| advertise                    | ✅ |
| unadvertise                  | ✅ |
| publish                      | ✅ |
| subscribe                    | ✅ |
| unsubscribe                  | ✅ |
| services                     | ✅ |
| rosapi                       | ✅ (ROS1 only for now) |
| TLS / wss://                 | Should be working, untested |
| ROS2 msgs length limits      | Planned |
| cbor                         | Planned |
| rosbridge status access      | Planned |
| rosout logger                | Planned |
| auth                         | Planned |
| fragment / png               | Uncertain if this package will support |
| cbor-raw                     | Uncertain if this package will support |
| ros1 TCPROS / raw            | Uncertain if this package will support |
| ros2 DDS / raw               | Uncertain if this package will support |
