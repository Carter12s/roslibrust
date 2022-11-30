# RosLibRust
[![Rust](https://github.com/Carter12s/roslibrust/actions/workflows/rust.yml/badge.svg)](https://github.com/Carter12s/roslibrust/actions/workflows/rust.yml)
[![License:MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This package aims to provide a convenient intermediary between ROS1's rosbridge and Rust similar to roslibpy and roslibjs.

Information about the protocol can be found [here](https://github.com/RobotWebTools/rosbridge_suite).

Note on documentation:
All information about the crate itself (examples, documentation, tutorials, etc.) lives in the source code and can be viewed on [docs.rs](https://docs.rs/roslibrust).
This readme is for "Meta" information about developing for the crate.

Current Status: Reliable in production, API still changing

Fully Supported: Noetic

Partial Support: Galactic, Rolling (Partial message generation, handwritten messages do work)


| Feature                      | Status                                                      |
|------------------------------|-------------------------------------------------------------|
| tutorials                    | In Progress |
| message_gen                  | ✅ |
| advertise                    | ✅ |
| unadvertise                  | ✅ |
| publish                      | ✅ |
| subscribe                    | ✅ |
| unsubscribe                  | ✅ |
| services                     | ✅ |
| fragment / png / cbor        | ✅ |
| rosapi                       | ✅ |
| ros2 rosbridge               | In Progress |
| cbor-raw                     | Planned |
| ros1 TCPROS / raw            | Planned |
| rosbridge status access      | Planned |
| ros2 DDS / raw               | Uncertain if this package will support |

