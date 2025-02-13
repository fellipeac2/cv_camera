// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <sstream>
#include <string>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id),
      info_manager_(node_, topic_name),
      capture_delay_(ros::Duration(node_.param("capture_delay", 0.0)))
{
}

Capture::Capture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id, const std::string &frame_id_left, const std::string &frame_id_right)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id),
      frame_id_left_(frame_id_left),
      frame_id_right_(frame_id_right),
      info_manager_(node_, topic_name),
      capture_delay_(ros::Duration(node_.param("capture_delay", 0.0)))
{
	std::string left("left/");
	std::string right("right/");
	left.append(topic_name);
	right.append(topic_name);
	topic_name_left_ = left;
	topic_name_right_ = right;
}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }

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

void Capture::rescaleCameraInfo(int width, int height)
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
}

void Capture::open(int32_t device_id)
{
  cap_.open(device_id);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  pub_left_ = it_.advertiseCamera(topic_name_left_, buffer_size_);
  pub_right_ = it_.advertiseCamera(topic_name_right_, buffer_size_);

  loadCameraInfo();
}

void Capture::open(const std::string &device_path)
{
  cap_.open(device_path, cv::CAP_V4L);
  if (!cap_.isOpened())
  {
    throw DeviceError("device_path " + device_path + " cannot be opened");
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  pub_left_ = it_.advertiseCamera(topic_name_left_, buffer_size_);
  pub_right_ = it_.advertiseCamera(topic_name_right_, buffer_size_);

  loadCameraInfo();
}

void Capture::open()
{
  open(0);
}

void Capture::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  pub_left_ = it_.advertiseCamera(topic_name_left_, buffer_size_);
  pub_right_ = it_.advertiseCamera(topic_name_right_, buffer_size_);

  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }
}

void Capture::split() {
    bridge_left_.encoding = bridge_.encoding;
    bridge_left_.header.stamp = bridge_.header.stamp;
    bridge_left_.header.frame_id = frame_id_left_;

    bridge_right_.encoding = bridge_.encoding;
    bridge_right_.header.stamp = bridge_.header.stamp;
    bridge_right_.header.frame_id = frame_id_right_;

    info_left_.height = info_.height;
    info_left_.width = info_.width / 2;
    info_left_.header.stamp = info_.header.stamp;
    info_left_.header.frame_id = frame_id_left_;
	
    info_right_.height = info_.height;
    info_right_.width = info_.width / 2;
    info_right_.header.stamp = info_.header.stamp;
    info_right_.header.frame_id = frame_id_right_;


    cv::Rect left_rect(0, 0, info_.width / 2, info_.height);
    cv::Rect right_rect(info_.width / 2, 0, info_.width / 2, info_.height);

    bridge_left_.image = bridge_.image(left_rect);
    bridge_right_.image = bridge_.image(right_rect);
}

bool Capture::capture()
{
	ros::Time stamp = ros::Time::now() - capture_delay_;
	return capture(stamp);
}

bool Capture::capture(ros::Time stamp)
{
  cv::Mat image_flipped;
  if (cap_.read(image_flipped))
  { 
    cv::flip(image_flipped, bridge_.image, -1);



    bridge_.encoding = enc::BGR8;
    bridge_.header.stamp = stamp;
    bridge_.header.frame_id = frame_id_;

    info_ = info_manager_.getCameraInfo();
    if (info_.height == 0 && info_.width == 0)
    {
      info_.height = bridge_.image.rows;
      info_.width = bridge_.image.cols;
    }
    else if (info_.height != bridge_.image.rows || info_.width != bridge_.image.cols)
    {
      if (rescale_camera_info_)
      {
        int old_width = info_.width;
        int old_height = info_.height;
        rescaleCameraInfo(bridge_.image.cols, bridge_.image.rows);
        ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, bridge_.image.cols, bridge_.image.rows);
      }
      else
      {
        ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_.width, info_.height, bridge_.image.cols, bridge_.image.rows);
      }
    }
    info_.header.stamp = stamp;
    info_.header.frame_id = frame_id_;

    return true;
  }
  return false;
}

void Capture::publish()
{
	pub_.publish(*getImageMsgPtr(), info_);
}

void Capture::publish_pair()
{
  pub_left_.publish(*getImageLeftMsgPtr(), info_left_);
  pub_right_.publish(*getImageRightMsgPtr(), info_right_);
}

bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
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
