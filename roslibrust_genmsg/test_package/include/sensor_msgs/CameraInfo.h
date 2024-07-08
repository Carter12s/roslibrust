#ifndef SENSOR_MSGS_MESSAGE_CAMERAINFO
#define SENSOR_MSGS_MESSAGE_CAMERAINFO

#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/RegionOfInterest.h>

namespace sensor_msgs {

template <class ContainerAllocator>
struct CameraInfo_
{
  typedef CameraInfo_<ContainerAllocator> Type;
  CameraInfo_()
    : header()
    , height()
    , width()
    , distortion_model()
    , D()
    , K()
    , R()
    , P()
    , binning_x()
    , binning_y()
    , roi() {
  }

  CameraInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , height(_alloc)
    , width(_alloc)
    , distortion_model(_alloc)
    , D(_alloc)
    , K(_alloc)
    , R(_alloc)
    , P(_alloc)
    , binning_x(_alloc)
    , binning_y(_alloc)
    , roi(_alloc) {
    (void)_alloc;
  }
    
        typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
    _header_type header;
    
        typedef uint32_t _height_type;
    _height_type height;
    
        typedef uint32_t _width_type;
    _width_type width;
    
        typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _distortion_model_type;
    _distortion_model_type distortion_model;
    
        typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _D_type;
    _D_type D;
    
        typedef boost::array<double, 9> _K_type;
    _K_type K;
    
        typedef boost::array<double, 9> _R_type;
    _R_type R;
    
        typedef boost::array<double, 12> _P_type;
    _P_type P;
    
        typedef uint32_t _binning_x_type;
    _binning_x_type binning_x;
    
        typedef uint32_t _binning_y_type;
    _binning_y_type binning_y;
    
        typedef ::sensor_msgs::RegionOfInterest_<ContainerAllocator> _roi_type;
    _roi_type roi;

  

  typedef boost::shared_ptr< ::sensor_msgs::CameraInfo_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr< ::sensor_msgs::CameraInfo_<ContainerAllocator> const> ConstPtr;

}; // struct CameraInfo_

typedef ::sensor_msgs::CameraInfo_<std::allocator<void>> CameraInfo;

typedef boost::shared_ptr< ::sensor_msgs::CameraInfo> CameraInfoPtr;
typedef boost::shared_ptr< ::sensor_msgs::CameraInfo const> CameraInfoConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor_msgs::CameraInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensor_msgs::CameraInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sensor_msgs::CameraInfo_<ContainerAllocator1> & lhs, const ::sensor_msgs::CameraInfo_<ContainerAllocator2> & rhs)
{
  return
    lhs.header == rhs.header &&
    lhs.height == rhs.height &&
    lhs.width == rhs.width &&
    lhs.distortion_model == rhs.distortion_model &&
    lhs.D == rhs.D &&
    lhs.K == rhs.K &&
    lhs.R == rhs.R &&
    lhs.P == rhs.P &&
    lhs.binning_x == rhs.binning_x &&
    lhs.binning_y == rhs.binning_y &&
    lhs.roi == rhs.roi &&
    true;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sensor_msgs::CameraInfo_<ContainerAllocator1> & lhs, const ::sensor_msgs::CameraInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sensor_msgs

namespace ros
{
namespace message_traits
{
template <class ContainerAllocator>
struct IsMessage< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_msgs::CameraInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
  : FalseType
    { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::CameraInfo_<ContainerAllocator> const>
  : FalseType
    { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
  : TrueType
    { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::CameraInfo_<ContainerAllocator> const>
  : TrueType
    { };

template<class ContainerAllocator>
struct MD5Sum< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
{
  static constexpr char const * value()
  {
    return "c9a58c1b0b154e0e6da7578cb991d214";
  }

  static const char* value(const ::sensor_msgs::CameraInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc9a58c1b0b154e0eULL;
  static const uint64_t static_value2 = 0x6da7578cb991d214ULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
{
  static constexpr char const* value()
  {
    return "sensor_msgs/CameraInfo";
  }

  static const char* value(const ::sensor_msgs::CameraInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
{
  static constexpr char const* value()
  {
    return "# This message defines meta information for a camera. It should be in a"
"# camera namespace on topic \"camera_info\" and accompanied by up to five"
"# image topics named:"
"#"
"#   image_raw - raw data from the camera driver, possibly Bayer encoded"
"#   image            - monochrome, distorted"
"#   image_color      - color, distorted"
"#   image_rect       - monochrome, rectified"
"#   image_rect_color - color, rectified"
"#"
"# The image_pipeline contains packages (image_proc, stereo_image_proc)"
"# for producing the four processed image topics from image_raw and"
"# camera_info. The meaning of the camera parameters are described in"
"# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo."
"#"
"# The image_geometry package provides a user-friendly interface to"
"# common operations using this meta information. If you want to, e.g.,"
"# project a 3d point into image coordinates, we strongly recommend"
"# using image_geometry."
"#"
"# If the camera is uncalibrated, the matrices D, K, R, P should be left"
"# zeroed out. In particular, clients may assume that K[0] == 0.0"
"# indicates an uncalibrated camera."
""
"#######################################################################"
"#                     Image acquisition info                          #"
"#######################################################################"
""
"# Time of image acquisition, camera coordinate frame ID"
"Header header    # Header timestamp should be acquisition time of image"
"                 # Header frame_id should be optical frame of camera"
"                 # origin of frame should be optical center of camera"
"                 # +x should point to the right in the image"
"                 # +y should point down in the image"
"                 # +z should point into the plane of the image"
""
""
"#######################################################################"
"#                      Calibration Parameters                         #"
"#######################################################################"
"# These are fixed during camera calibration. Their values will be the #"
"# same in all messages until the camera is recalibrated. Note that    #"
"# self-calibrating systems may \"recalibrate\" frequently.              #"
"#                                                                     #"
"# The internal parameters can be used to warp a raw (distorted) image #"
"# to:                                                                 #"
"#   1. An undistorted image (requires D and K)                        #"
"#   2. A rectified image (requires D, K, R)                           #"
"# The projection matrix P projects 3D points into the rectified image.#"
"#######################################################################"
""
"# The image dimensions with which the camera was calibrated. Normally"
"# this will be the full camera resolution in pixels."
"uint32 height"
"uint32 width"
""
"# The distortion model used. Supported models are listed in"
"# sensor_msgs/distortion_models.h. For most cameras, \"plumb_bob\" - a"
"# simple model of radial and tangential distortion - is sufficient."
"string distortion_model"
""
"# The distortion parameters, size depending on the distortion model."
"# For \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3)."
"float64[] D"
""
"# Intrinsic camera matrix for the raw (distorted) images."
"#     [fx  0 cx]"
"# K = [ 0 fy cy]"
"#     [ 0  0  1]"
"# Projects 3D points in the camera coordinate frame to 2D pixel"
"# coordinates using the focal lengths (fx, fy) and principal point"
"# (cx, cy)."
"float64[9]  K # 3x3 row-major matrix"
""
"# Rectification matrix (stereo cameras only)"
"# A rotation matrix aligning the camera coordinate system to the ideal"
"# stereo image plane so that epipolar lines in both stereo images are"
"# parallel."
"float64[9]  R # 3x3 row-major matrix"
""
"# Projection/camera matrix"
"#     [fx'  0  cx' Tx]"
"# P = [ 0  fy' cy' Ty]"
"#     [ 0   0   1   0]"
"# By convention, this matrix specifies the intrinsic (camera) matrix"
"#  of the processed (rectified) image. That is, the left 3x3 portion"
"#  is the normal camera intrinsic matrix for the rectified image."
"# It projects 3D points in the camera coordinate frame to 2D pixel"
"#  coordinates using the focal lengths (fx', fy') and principal point"
"#  (cx', cy') - these may differ from the values in K."
"# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will"
"#  also have R = the identity and P[1:3,1:3] = K."
"# For a stereo pair, the fourth column [Tx Ty 0]' is related to the"
"#  position of the optical center of the second camera in the first"
"#  camera's frame. We assume Tz = 0 so both cameras are in the same"
"#  stereo image plane. The first camera always has Tx = Ty = 0. For"
"#  the right (second) camera of a horizontal stereo pair, Ty = 0 and"
"#  Tx = -fx' * B, where B is the baseline between the cameras."
"# Given a 3D point [X Y Z]', the projection (x, y) of the point onto"
"#  the rectified image is given by:"
"#  [u v w]' = P * [X Y Z 1]'"
"#         x = u / w"
"#         y = v / w"
"#  This holds for both images of a stereo pair."
"float64[12] P # 3x4 row-major matrix"
""
""
"#######################################################################"
"#                      Operational Parameters                         #"
"#######################################################################"
"# These define the image region actually captured by the camera       #"
"# driver. Although they affect the geometry of the output image, they #"
"# may be changed freely without recalibrating the camera.             #"
"#######################################################################"
""
"# Binning refers here to any camera setting which combines rectangular"
"#  neighborhoods of pixels into larger \"super-pixels.\" It reduces the"
"#  resolution of the output image to"
"#  (width / binning_x) x (height / binning_y)."
"# The default values binning_x = binning_y = 0 is considered the same"
"#  as binning_x = binning_y = 1 (no subsampling)."
"uint32 binning_x"
"uint32 binning_y"
""
"# Region of interest (subwindow of full camera resolution), given in"
"#  full resolution (unbinned) image coordinates. A particular ROI"
"#  always denotes the same window of pixels on the camera sensor,"
"#  regardless of binning settings."
"# The default setting of roi (all values 0) is considered the same as"
"#  full resolution (roi.width = width, roi.height = height)."
"RegionOfInterest roi"
"================================================================================"
"MSG: sensor_msgs/RegionOfInterest"
"# This message is used to specify a region of interest within an image."
"#"
"# When used to specify the ROI setting of the camera when the image was"
"# taken, the height and width fields should either match the height and"
"# width fields for the associated image; or height = width = 0"
"# indicates that the full resolution image was captured."
""
"uint32 x_offset  # Leftmost pixel of the ROI"
"                 # (0 if the ROI includes the left edge of the image)"
"uint32 y_offset  # Topmost pixel of the ROI"
"                 # (0 if the ROI includes the top edge of the image)"
"uint32 height    # Height of ROI"
"uint32 width     # Width of ROI"
""
"# True if a distinct rectified ROI should be calculated from the \"raw\""
"# ROI in this message. Typically this should be False if the full image"
"# is captured (ROI not used), and True if a subwindow is captured (ROI"
"# used)."
"bool do_rectify"
"================================================================================"
"MSG: std_msgs/Header"
"# Standard metadata for higher-level stamped data types."
"# This is generally used to communicate timestamped data "
"# in a particular coordinate frame."
"# "
"# sequence ID: consecutively increasing ID "
"uint32 seq"
"#Two-integer timestamp that is expressed as:"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')"
"# time-handling sugar is provided by the client library"
"time stamp"
"#Frame this data is associated with"
"string frame_id";
  }

  static const char* value(const ::sensor_msgs::CameraInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator>
struct Serializer< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.height);
    stream.next(m.width);
    stream.next(m.distortion_model);
    stream.next(m.D);
    stream.next(m.K);
    stream.next(m.R);
    stream.next(m.P);
    stream.next(m.binning_x);
    stream.next(m.binning_y);
    stream.next(m.roi);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct CameraInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor_msgs::CameraInfo_<ContainerAllocator>>
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor_msgs::CameraInfo_<ContainerAllocator>& v)
  {

    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator>>::stream(s, indent + "  ", v.header);

    s << indent << "height: ";
    Printer< uint32_t>::stream(s, indent + "  ", v.height);

    s << indent << "width: ";
    Printer< uint32_t>::stream(s, indent + "  ", v.width);

    s << indent << "distortion_model: ";
    Printer< std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.distortion_model);
    
    s << indent << "D[]" << std::endl;
    for (size_t i = 0; i < v.D.size(); ++i)
    {
      s << indent << "  D[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.D[i]);
    }

    s << indent << "K: ";
    Printer< double>::stream(s, indent + "  ", v.K);

    s << indent << "R: ";
    Printer< double>::stream(s, indent + "  ", v.R);

    s << indent << "P: ";
    Printer< double>::stream(s, indent + "  ", v.P);

    s << indent << "binning_x: ";
    Printer< uint32_t>::stream(s, indent + "  ", v.binning_x);

    s << indent << "binning_y: ";
    Printer< uint32_t>::stream(s, indent + "  ", v.binning_y);

    s << indent << "roi: ";
    s << std::endl;
    Printer< ::sensor_msgs::RegionOfInterest_<ContainerAllocator>>::stream(s, indent + "  ", v.roi);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_MSGS_MESSAGE_CAMERAINFO