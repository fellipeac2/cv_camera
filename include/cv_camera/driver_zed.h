// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_DRIVER_H
#define CV_CAMERA_DRIVER_H

#include "cv_camera/capture_stereo.h"

namespace cv_camera
{

/**
 * @brief ROS cv camera driver.
 *
 * This wraps getting parameters and publish in specified rate.
 */
class DriverZed
{
 public:
  /**
   * @brief construct with ROS node handles.
   *
   * use private_node for getting topics like ~rate or ~device,
   * camera_node for advertise and publishing images.
   *
   * @param private_node node for getting parameters.
   * @param camera_node node for publishing.
   */
  DriverZed(ros::NodeHandle& private_node,
         ros::NodeHandle& camera_node);
  ~DriverZed();

  /**
   * @brief Setup camera device and ROS parameters.
   *
   * @throw cv_camera::DeviceError device open failed.
   */
  void setup();
  /**
   * @brief CaptureStereo, publish and sleep
   */
  void proceed();
 private:
  /**
   * @brief ROS private node for getting ROS parameters.
   */
  ros::NodeHandle private_node_;
  /**
   * @brief ROS private node for publishing images.
   */
  ros::NodeHandle camera_node_;
  /**
   * @brief wrapper of cv::VideoCaptureStereo.
   */
  boost::shared_ptr<CaptureStereo> camera_;

  /**
   * @brief publishing rate.
   */
  boost::shared_ptr<ros::Rate> rate_;
};

}  // namespace cv_camera

#endif  // CV_CAMERA_DRIVER_H
