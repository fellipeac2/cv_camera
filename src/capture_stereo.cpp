// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture_stereo.h"
#include <sstream>
#include <string>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

CaptureStereo::CaptureStereo(ros::NodeHandle &node, 
    const std::string &topic_name_left, 
    const std::string &topic_name_right,
    int32_t buffer_size,
    const std::string &frame_id_left,
    const std::string &frame_id_right)    : 
      node_(node),
      it_(node_),
      topic_name_left_(topic_name_left),
      topic_name_right_(topic_name_right),
      buffer_size_(buffer_size),
      frame_id_left_(frame_id_left),
      frame_id_right_(frame_id_right),
      info_manager_left_(node_, topic_name_left),
      info_manager_right_(node_, topic_name_right),
      capture_delay_(ros::Duration(node_.param("capture_delay", 0.0)))
{
  ROS_INFO("%s", topic_name_left.c_str());
}

void CaptureStereo::loadCameraInfo()
{
  std::string url_left;
  if (node_.getParam("left/camera_info_url", url_left))
  {
  ROS_INFO("%s", url_left.c_str());
    if (info_manager_left_.validateURL(url_left))
    {
      info_manager_left_.loadCameraInfo(url_left);
    }
    else {
      info_manager_left_.loadCameraInfo(url_left);
        ROS_INFO("URL doesn't validated");
    }
  }
  std::string url_right;
  if (node_.getParam("right/camera_info_url", url_right))
  {
    if (info_manager_right_.validateURL(url_right))
    {
      info_manager_right_.loadCameraInfo(url_right);
    }
  }

    info_left_ = info_manager_left_.getCameraInfo();
    info_right_ = info_manager_right_.getCameraInfo();
  rescale_camera_info_ = node_.param<bool>("rescale_camera_info", false);

  for (int i = 0;; ++i)
  {
    int code = 0;
    double value = 0.0;
    std::stringstream stream;
    stream << "property_" << i << "_code";
    const std::string param_for_code = stream.str();
    stream.str("");
    stream << "property_" << i << "_value";
    const std::string param_for_value = stream.str();
    if (!node_.getParam(param_for_code, code) || !node_.getParam(param_for_value, value))
    {
      break;
    }
    if (!cap_.set(code, value))
    {
      ROS_ERROR_STREAM("Setting with code " << code << " and value " << value << " failed"
                                            << std::endl);
    }
  }
}

/*void CaptureStereo::rescaleCameraInfo(int width, int height)
{
  double width_coeff = static_cast<double>(width) / info_.width;
  double height_coeff = static_cast<double>(height) / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.K[0] *= width_coeff;
  info_.K[2] *= width_coeff;
  info_.K[4] *= height_coeff;
  info_.K[5] *= height_coeff;

  info_.P[0] *= width_coeff;
  info_.P[2] *= width_coeff;
  info_.P[5] *= height_coeff;
  info_.P[6] *= height_coeff;
}*/

void CaptureStereo::open(int32_t device_id)
{
  cap_.open(device_id);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_left_ = it_.advertiseCamera(topic_name_left_, buffer_size_);
  pub_right_ = it_.advertiseCamera(topic_name_right_, buffer_size_);

  loadCameraInfo();
}

void CaptureStereo::open(const std::string &device_path)
{
  cap_.open(device_path, cv::CAP_V4L);
  if (!cap_.isOpened())
  {
    throw DeviceError("device_path " + device_path + " cannot be opened");
  }
  pub_left_ = it_.advertiseCamera(topic_name_left_, buffer_size_);
  pub_right_ = it_.advertiseCamera(topic_name_right_, buffer_size_);

  loadCameraInfo();
}

void CaptureStereo::open()
{
  open(0);
}

void CaptureStereo::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_left_ = it_.advertiseCamera(topic_name_left_, buffer_size_);
  pub_right_ = it_.advertiseCamera(topic_name_right_, buffer_size_);

  std::string url_left;
  if (node_.getParam("left/camera_info_url", url_left))
  {
    if (info_manager_left_.validateURL(url_left))
    {
      info_manager_left_.loadCameraInfo(url_left);
    }
  }
  std::string url_right;
  if (node_.getParam("right/camera_info_url", url_right))
  {
    if (info_manager_right_.validateURL(url_right))
    {
      info_manager_right_.loadCameraInfo(url_right);
    }
  }
}


bool CaptureStereo::capture()
{
	ros::Time stamp = ros::Time::now() - capture_delay_;
	return capture(stamp);
}

bool CaptureStereo::capture(ros::Time stamp)
{
    cv::Rect left_rect(0, 0, info_left_.width, info_left_.height);
    cv::Rect right_rect(info_right_.width, 0, info_right_.width, info_right_.height);
  cv::Mat image_raw;
  if (cap_.read(image_raw))
  { 
   bridge_left_.image = image_raw(left_rect);
   bridge_right_.image = image_raw(right_rect);


    bridge_left_.encoding = enc::BGR8;
    bridge_left_.header.stamp = stamp;
    bridge_left_.header.frame_id = frame_id_left_;

    bridge_right_.encoding = enc::BGR8;
    bridge_right_.header.stamp = stamp;
    bridge_right_.header.frame_id = frame_id_right_;

    info_left_ = info_manager_left_.getCameraInfo();
    info_right_ = info_manager_right_.getCameraInfo();

    // LEFT
    if (info_left_.height == 0 && info_left_.width == 0)
    {
      info_left_.height = bridge_left_.image.rows;
      info_left_.width = bridge_left_.image.cols;
    }
    else if (info_left_.height != bridge_left_.image.rows || info_left_.width != bridge_left_.image.cols)
    {
      //if (rescale_camera_info_)
      //{
      //  int old_width = info_left_.width;
      //  int old_height = info_left_.height;
      //  rescaleCameraInfo(bridge_left_.image.cols, bridge_left_.image.rows);
      //  ROS_INFO_ONCE("Camera left calibration automatically rescaled from %dx%d to %dx%d",
      //                old_width, old_height, bridge_left_.image.cols, bridge_left_.image.rows);
      //}
      //else
      //{
        ROS_WARN_ONCE("Calibration left resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_left_.width, info_left_.height, bridge_left_.image.cols, bridge_left_.image.rows);
      //}
    }

    // RIGHT

    if (info_right_.height == 0 && info_right_.width == 0)
    {
      info_right_.height = bridge_right_.image.rows;
      info_right_.width = bridge_right_.image.cols;
    }
    else if (info_right_.height != bridge_right_.image.rows || info_right_.width != bridge_right_.image.cols)
    {
    //  if (rescale_camera_info_)
    //  {
    //    int old_width = info_right_.width;
    //    int old_height = info_right_.height;
    //    rescaleCameraInfo(bridge_right_.image.cols, bridge_right_.image.rows);
    //    ROS_INFO_ONCE("Camera right calibration automatically rescaled from %dx%d to %dx%d",
    //                  old_width, old_height, bridge_right_.image.cols, bridge_right_.image.rows);
    //  }
    //  else
    //  {
        ROS_WARN_ONCE("Calibration right resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_right_.width, info_right_.height, bridge_right_.image.cols, bridge_right_.image.rows);
    //  }
    }

    info_left_.header.stamp = stamp;
    info_left_.header.frame_id = frame_id_left_;
    info_right_.header.stamp = stamp;
    info_right_.header.frame_id = frame_id_right_;

    return true;
  }
  return false;
}

void CaptureStereo::publish()
{
  pub_left_.publish(*getImageLeftMsgPtr(), info_left_);
  pub_right_.publish(*getImageRightMsgPtr(), info_right_);
}


bool CaptureStereo::setPropertyFromParam(int property_id, const std::string &param_name)
{
  if (cap_.isOpened())
  {
    double value = 0.0;
    if (node_.getParam(param_name, value))
    {
      ROS_INFO("setting property %s = %lf", param_name.c_str(), value);
      return cap_.set(property_id, value);
    }
  }
  return true;
}

} // namespace cv_camera
