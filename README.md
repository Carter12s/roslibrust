# RosLibRust
This package aims to provide a convenient intermediary between ROS1's rosbridge and Rust similar to roslibpy.

Information about the protocol can be found [here](https://github.com/RobotWebTools/rosbridge_suite).

Current Status: Not Ready for Production
Intended Support: Noetic, Galactic, Rolling. Development currently focused on Noetic

| Feature                      | Status                                                      |
|------------------------------|-------------------------------------------------------------|
| message_gen                  | Working and tested, some non-compliance with ROS likely     |
| advertise                    | Working and tested, needs documentation and error reporting |
| unadvertise                  | Not started, planned                                        |
| publish                      | Working and tested, needs documentation and error reporting |
| subscribe                    | Working and tested, needs documentation and error reporting |
| unsubscribe                  | Working and tested, needs documentation and error reporting | 
| services                     | Not started, planned                                        |
| fragment / png / compression | No support planned                                          |
| automatic integration tests  | Started not complete                                        |
| rosbridge status access      | Not started, planned                                        |

## Message Generation
Working! Not macro based, API for message gen which can be called with build.rs or manually ahead of compilation

//TODO documentation on how to integrate message generation into your project, and environment variables

## Testing
//TODO In progress automatic integration tests


