// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver_zed.h"
#include <string>

namespace
{
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}

namespace cv_camera
{

DriverZed::DriverZed(ros::NodeHandle &private_node, ros::NodeHandle &camera_node)
    : private_node_(private_node),
      camera_node_(camera_node)
{
}

void DriverZed::setup()
{
  double hz(DEFAULT_RATE);
  int32_t device_id(0);
  std::string device_path("");
  std::string file_path("");

  private_node_.getParam("device_id", device_id);
  private_node_.getParam("rate", hz);

  std::string frame_id_left("camera_left_link");
  std::string frame_id_right("camera_right_link");
  private_node_.getParam("frame_id_left", frame_id_left);
  private_node_.getParam("frame_id_right", frame_id_right);


  int32_t image_width(640);
  int32_t image_height(480);

  ROS_INFO("device_id %d", device_id);

  camera_.reset(new CaptureStereo(camera_node_,
                            "left/image_raw",
                            "right/image_raw",
                            PUBLISHER_BUFFER_SIZE,
                            frame_id_left,
                            frame_id_right));

  if (private_node_.getParam("file", file_path) && file_path != "")
  {
    camera_->openFile(file_path);
  }
  else if (private_node_.getParam("device_path", device_path) && device_path != "")
  {
    camera_->open(device_path);
  }
  else
  {
    camera_->open(device_id);
  }
  if (private_node_.getParam("image_width", image_width))
  {
    if (!camera_->setWidth(image_width))
    {
      ROS_WARN("fail to set image_width");
    }
  }
  if (private_node_.getParam("image_height", image_height))
  {
    if (!camera_->setHeight(image_height))
    {
      ROS_WARN("fail to set image_height");
    }
  }

  camera_->setPropertyFromParam(CV_CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
  camera_->setPropertyFromParam(CV_CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
  camera_->setPropertyFromParam(CV_CAP_PROP_FPS, "cv_cap_prop_fps");
  camera_->setPropertyFromParam(CV_CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
  camera_->setPropertyFromParam(CV_CAP_PROP_FORMAT, "cv_cap_prop_format");
  camera_->setPropertyFromParam(CV_CAP_PROP_MODE, "cv_cap_prop_mode");
  camera_->setPropertyFromParam(CV_CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
  camera_->setPropertyFromParam(CV_CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
  camera_->setPropertyFromParam(CV_CAP_PROP_SATURATION, "cv_cap_prop_saturation");
  camera_->setPropertyFromParam(CV_CAP_PROP_HUE, "cv_cap_prop_hue");
  camera_->setPropertyFromParam(CV_CAP_PROP_GAIN, "cv_cap_prop_gain");
  camera_->setPropertyFromParam(CV_CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
  camera_->setPropertyFromParam(CV_CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");

  camera_->setPropertyFromParam(CV_CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
  camera_->setPropertyFromParam(CV_CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
  camera_->setPropertyFromParam(CV_CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif // CV_CAP_PROP_BUFFERSIZE

  rate_.reset(new ros::Rate(hz));
}

void DriverZed::proceed()
{
  if (camera_->capture())
  {
    camera_->publish();
  }
  rate_->sleep();
}

DriverZed::~DriverZed()
{
}

} // namespace cv_camera
