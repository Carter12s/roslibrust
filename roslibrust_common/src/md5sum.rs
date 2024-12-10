use anyhow::{anyhow, bail, Error};
use std::collections::{HashMap, HashSet};

// TODO(lucasw) this deserves a lot of str vs String cleanup
/// This function will calculate the md5sum of an expanded message definition.
/// The expanded message definition is the output of `gendeps --cat` see: <https://wiki.ros.org/roslib/gentools>
/// This definition is typically sent in the connection header of a ros1 topic and is also stored in bag files.
/// This can be used to calculate the md5sum when message definitions aren't available at compile time.
pub fn message_definition_to_md5sum(msg_name: &str, full_def: &str) -> Result<String, Error> {
    if full_def.is_empty() {
        bail!("empty input definition");
    }

    // Split the full definition into sections per message
    let sep: &str =
        "================================================================================\n";
    let sections = full_def.split(sep).collect::<Vec<&str>>();
    if sections.is_empty() {
        // Carter: this error is impossible, split only gives empty iterator when input string is empty
        // which we've already checked for above
        bail!("empty sections");
    }

    // Split the overall definition into separate sub-messages sorted by message type (including package name)
    let mut sub_messages: HashMap<&str, String> = HashMap::new();
    // Note: the first section doesn't contain the "MSG: <type>" line so we don't need to strip it here
    let clean_root = clean_msg(sections[0]);
    if clean_root.is_empty() {
        bail!("empty cleaned root definition");
    }
    sub_messages.insert(msg_name, clean_root);

    for section in &sections[1..] {
        let line0 = section.lines().next().ok_or(anyhow!("empty section"))?;
        if !line0.starts_with("MSG: ") {
            bail!("bad section {section} -> {line0} doesn't start with 'MSG: '");
        }
        // TODO(lucasw) the full text definition doesn't always have the full message types with
        // the package name,
        // but I think this is only when the message type is Header or the package of the message
        // being define is the same as the message in the field
        // Carter: I agree with this, we found the same when dealing with this previously
        let section_type = line0.split_whitespace().collect::<Vec<&str>>()[1];
        let end_of_first_line = section
            .find('\n')
            .ok_or(anyhow!("No body found in section"))?;
        let body = clean_msg(&section[end_of_first_line + 1..]);
        sub_messages.insert(section_type, body);
    }

    // TODO MAJOR(carter): I'd like to convert this loop to a recursive function where we pass in the map of hashes
    // and update them as we go, this tripple loop is stinky to my eye.
    // TODO(carter) we should be able to do this in close to one pass if we iterate the full_def backwards
    let mut hashed = HashMap::new();
    let hash = message_definition_to_md5sum_recursive(msg_name, &sub_messages, &mut hashed)?;

    Ok(hash)
}

/// Calculates the hash of the specified message type by recursively calling itself on all dependencies
/// Uses defs as the list of message definitions available for it (expects them to already be cleaned)
/// Uses hashes as the cache of already calculated hashes so we don't redo work
fn message_definition_to_md5sum_recursive(
    msg_type: &str,
    defs: &HashMap<&str, String>,
    hashes: &mut HashMap<String, String>,
) -> Result<String, Error> {
    let base_types: HashSet<String> = HashSet::from_iter(
        [
            "bool", "byte", "int8", "int16", "int32", "int64", "uint8", "uint16", "uint32",
            "uint64", "float32", "float64", "time", "duration", "string",
        ]
        .map(|name| name.to_string()),
    );
    let def = defs
        .get(msg_type)
        .ok_or(anyhow!("Couldn't find message type: {msg_type}"))?;
    let pkg_name = msg_type.split('/').collect::<Vec<&str>>()[0];
    // We'll store the expanded hash definition in this string as we go
    let mut field_def = "".to_string();
    for line_raw in def.lines() {
        let line_split = line_raw.split_whitespace().collect::<Vec<&str>>();
        if line_split.len() < 2 {
            bail!("bad line to split '{line_raw}'");
        }
        let (raw_field_type, _field_name) = (line_split[0], line_split[1]);
        // leave array characters alone, could be [] [C] where C is a constant
        let field_type = raw_field_type.split('[').collect::<Vec<&str>>()[0].to_string();

        let full_field_type;
        let line;
        if base_types.contains(&field_type) {
            line = line_raw.to_string();
        } else {
            // TODO(lucasw) are there other special message types besides header- or is it anything in std_msgs?
            if field_type == "Header" {
                full_field_type = "std_msgs/Header".to_string();
            } else if !field_type.contains('/') {
                full_field_type = format!("{pkg_name}/{field_type}");
            } else {
                full_field_type = field_type;
            }

            match hashes.get(&full_field_type) {
                Some(hash_value) => {
                    // Hash already exists in cache so we can use it
                    line = line_raw.replace(raw_field_type, hash_value).to_string();
                }
                None => {
                    // Recurse! To calculate hash of this field type
                    let hash =
                        message_definition_to_md5sum_recursive(&full_field_type, defs, hashes)?;
                    line = line_raw.replace(raw_field_type, &hash).to_string();
                }
            }
        }
        field_def += &format!("{line}\n");
    }
    field_def = field_def.trim().to_string();
    let md5sum = md5::compute(field_def.trim_end().as_bytes());
    let md5sum_text = format!("{md5sum:x}");
    // Insert our hash into the cache before we return
    hashes.insert(msg_type.to_string(), md5sum_text.clone());

    Ok(md5sum_text)
}

/// Taking in a message definition
/// reformat it according to the md5sum algorithm: <https://wiki.ros.org/ROS/Technical%20Overview#Message_serialization_and_msg_MD5_sums>
/// - Comments removed
/// - Extra whitespace removed
/// - package names of dependencies removed
/// - constants reordered to be at the front
fn clean_msg(msg: &str) -> String {
    let mut result_params = vec![];
    let mut result_constants = vec![];
    for line in msg.lines() {
        let line = line.trim();
        // Skip comment lines
        if line.starts_with('#') {
            continue;
        }
        // Strip comment from the end of the line (if present)
        let line = line.split('#').collect::<Vec<&str>>()[0].trim();
        // Remove any extra whitespace from inside the line
        let line = line.split_whitespace().collect::<Vec<&str>>().join(" ");
        // Remove any whitespace on either side of the "=" for constants
        let line = line.replace(" = ", "=");
        // Skip any empty lines
        if line.is_empty() {
            continue;
        }
        // Determine if constant or not
        if line.contains('=') {
            result_constants.push(line);
        } else {
            result_params.push(line);
        }
    }
    format!(
        "{}\n{}",
        result_constants.join("\n"),
        result_params.join("\n")
    )
    .trim()
    .to_string() // Last trim here is lazy, but gets job done
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Confirm md5sum from the connection header message definition matches normally
    /// generated checksums
    #[test]
    fn msg_def_to_md5() {
        {
            let def = "byte DEBUG=1\nbyte INFO=2\nbyte WARN=4\nbyte ERROR=8\nbyte FATAL=16\n\
                2176decaecbce78abc3b96ef049fabed header\n\
                byte level\nstring name\nstring msg\nstring file\nstring function\nuint32 line\nstring[] topics";
            let expected = "acffd30cd6b6de30f120938c17c593fb";
            let md5sum = format!("{:x}", md5::compute(def.trim_end().as_bytes()));
            assert_eq!(md5sum, expected, "partially checksumed rosgraph_msgs/Log");
        }

        {
            let msg_type = "bad_msgs/Empty";
            let def = "";
            let _md5sum = message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            let msg_type = "bad_msgs/CommentSpacesOnly";
            let def =
                "# message with only comments and whitespace\n# another line comment\n\n    \n";
            let _md5sum = message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            let msg_type = "fake_msgs/MissingSectionMsg";
            let def = "string name\nstring msg\n================================================================================\n# message with only comments and whitespace\n# another line comment\n\n    \n";
            let _md5sum = message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            let msg_type = "bad_msgs/BadLog";
            let def = "##
## Severity level constants
byte DEUG=1 #debug level
byte FATAL=16 #fatal/critical level
##
## Fields
##
Header header
byte level
string name # name of the node
uint32 line # line the message came from
string[] topics # topic names that the node publishes

================================================================================
MSG: std_msgs/badHeader
# Standard metadata for higher-level stamped data types.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
time stamp
";
            let _md5sum = message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        {
            // TODO(lucasw) not sure if this is an ok message, currently it passes
            let expected = "96c44a027b586ee888fe95ac325151ae";
            let msg_type = "fake_msgs/CommentSpacesOnlySection";
            let def = "string name\nstring msg\n================================================================================\nMSG: foo/bar\n# message with only comments and whitespace\n# another line comment\n\n    \n";
            let md5sum = message_definition_to_md5sum(msg_type.into(), def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "fake_msgs/Garbage";
            let def = r#"
fsdajklf

==                   #fdjkl

MSG:    jklfd
# 
================================================================================
f

vjk
"#;
            let _md5sum = message_definition_to_md5sum(msg_type.into(), def.into()).unwrap_err();
        }

        // TODO(lucasw) it would be nice to pull these out of the real messages, but to avoid
        // dependencies just storing expected definition and md5sum
        // from roslib.message import get_message_class
        // msg = get_message_class("std_msgs/Header")
        // md5 = msg._md5sum
        // def = msg._full_text
        //

        {
            let msg_type = "sensor_msgs/CameraInfo";
            // This definition contains double quotes, so representing it with r# and newlines which is nicer
            // for limited width text editing anyhow
            let def = r#"
# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficient.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] P # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify

"#;
            let expected = "c9a58c1b0b154e0e6da7578cb991d214";
            let md5sum = message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "std_msgs/Header";
            let def = "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
            let expected = "2176decaecbce78abc3b96ef049fabed";
            let md5sum = message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "rosgraph_msgs/Log";
            let def = "##\n## Severity level constants\n##\nbyte DEBUG=1 #debug level\nbyte INFO=2  #general level\nbyte WARN=4  #warning level\nbyte ERROR=8 #error level\nbyte FATAL=16 #fatal/critical level\n##\n## Fields\n##\nHeader header\nbyte level\nstring name # name of the node\nstring msg # message \nstring file # file the message came from\nstring function # function the message came from\nuint32 line # line the message came from\nstring[] topics # topic names that the node publishes\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
            let expected = "acffd30cd6b6de30f120938c17c593fb";
            let md5sum = message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }

        {
            let msg_type = "nav_msgs/Odometry";
            let def = "# This represents an estimate of a position and velocity in free space.  \n# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n# The twist in this message should be specified in the coordinate frame given by the child_frame_id\nHeader header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n================================================================================\nMSG: geometry_msgs/TwistWithCovariance\n# This expresses velocity in free space with uncertainty.\n\nTwist twist\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
            let expected = "cd5e73d190d741a2f92e81eda573aca7";
            let md5sum = message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected);
        }

        {
            let msg_type = "tf2_msgs/TFMessage";
            let def = r#"
geometry_msgs/TransformStamped[] transforms

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href="http://wiki.ros.org/tf">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"#;
            let expected = "94810edda583a504dfda3829e70d7eec";
            let md5sum = message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected);
        }

        {
            let msg_type = "vision_msgs/Detection3DArray";
            let def = r#"
# A list of 3D detections, for a multi-object 3D detector.

Header header

# A list of the detected proposals. A multi-proposal detector might generate
#   this list with many candidate detections generated from a single input.
Detection3D[] detections

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: vision_msgs/Detection3D
# Defines a 3D detection result.
#
# This extends a basic 3D classification by including position information,
#   allowing a classification result for a specific position in an image to
#   to be located in the larger image.

Header header

# Class probabilities. Does not have to include hypotheses for all possible
#   object ids, the scores for any ids not listed are assumed to be 0.
ObjectHypothesisWithPose[] results

# 3D bounding box surrounding the object.
BoundingBox3D bbox

# The 3D data that generated these results (i.e. region proposal cropped out of
#   the image). This information is not required for all detectors, so it may
#   be empty.
sensor_msgs/PointCloud2 source_cloud

================================================================================
MSG: vision_msgs/ObjectHypothesisWithPose
# An object hypothesis that contains position information.

# The unique numeric ID of object detected. To get additional information about
#   this ID, such as its human-readable name, listeners should perform a lookup
#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
int64 id

# The probability or confidence value of the detected object. By convention,
#   this value should lie in the range [0-1].
float64 score

# The 6D pose of the object hypothesis. This pose should be
#   defined as the pose of some fixed reference point on the object, such a
#   the geometric center of the bounding box or the center of mass of the
#   object.
# Note that this pose is not stamped; frame information can be defined by
#   parent messages.
# Also note that different classes predicted for the same input data may have
#   different predicted 6D poses.
geometry_msgs/PoseWithCovariance pose
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: vision_msgs/BoundingBox3D
# A 3D bounding box that can be positioned and rotated about its center (6 DOF)
# Dimensions of this box are in meters, and as such, it may be migrated to
#   another package, such as geometry_msgs, in the future.

# The 3D position and orientation of the bounding box center
geometry_msgs/Pose center

# The size of the bounding box, in meters, surrounding the object's center
#   pose.
geometry_msgs/Vector3 size

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

"#;
            let expected = "05c51d9aea1fb4cfdc8effb94f197b6f";
            let md5sum = message_definition_to_md5sum(msg_type, def.into()).unwrap();
            println!("{msg_type}, computed {md5sum}, expected {expected}");
            assert_eq!(md5sum, expected, "{msg_type}");
        }
    }

    // Basic test of clean_msg function
    #[test]
    fn clean_msg_test() {
        let test_msg = r#"
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6 # Random Comment

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field


uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

"#;
        let result = clean_msg(test_msg);
        let expected = r#"uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count"#;
        assert_eq!(result, expected);
    }
}
