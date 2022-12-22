# This message is specifically for testing generating of default values
# Examples based on https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]

# More complicated examples to stress the system, floats with mixed precision
float32[] f_samples [-200, -1.0, 0]
string[] s_vec ["hello", "world"]
# This may or may not be valid ROS, it probably is, but we don't handle yet
# TODO handle this somehow
# string[] s_vec_2 ['hello', 'world']

# TODO ROS says this is valid, but we currently don't handle
#string single_quote 'Jane Doe'
