// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_CAPTURE_H
#define CV_CAMERA_CAPTURE_H

#include "cv_camera/exception.h"
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>

/**
 * @brief namespace of this package
 */
namespace cv_camera
{

/**
 * @brief captures by cv::VideoCaptureStereo and publishes to ROS topic.
 *
 */
class CaptureStereo
{
public:
  /**
   * @brief costruct with ros node and topic settings
   *
   * @param node ROS node handle for advertise topic.
   * @param topic_name name of topic to publish (this may be image_raw).
   * @param buffer_size size of publisher buffer.
   * @param frame_id frame_id of publishing messages.
   */

  CaptureStereo(ros::NodeHandle &node,
          const std::string &topic_name_left,
          const std::string &topic_name_right,
          int32_t buffer_size,
          const std::string &frame_id_left,
          const std::string &frame_id_right);
  /**
   * @brief Open capture device with device ID.
   *
   * @param device_id id of camera device (number from 0)
   * @throw cv_camera::DeviceError device open failed
   *
   */
  void open(int32_t device_id);

  /**
   * @brief Open capture device with device name.
   *
   * @param device_path path of the camera device
   * @throw cv_camera::DeviceError device open failed
   */
  void open(const std::string &device_path);

  /**
   * @brief Load camera info from file.
   *
   * This loads the camera info from the file specified in the camera_info_url parameter.
   */
  void loadCameraInfo();

  /**
   * @brief Open default camera device.
   *
   * This opens with device 0.
   *
   * @throw cv_camera::DeviceError device open failed
   */
  void open();

  /**
   * @brief open video file instead of capture device.
   */
  void openFile(const std::string &file_path);

  /**
   * @brief capture an image and store.
   *
   * to publish the captured image, call publish();
   * @return true if success to capture, false if not captured.
   */
  bool capture();
  
  /**
   * @brief capture an image and store with stamp predefined.
   *
   * to publish the captured image, call publish();
   * @return true if success to capture, false if not captured.
   */
  bool capture(ros::Time stamp);

  /**
   * @brief split image in two.
   */
  void split();


  /**
   * @brief Publish the image that is already captured by capture().
   *
   */
  void publish();


  /**
   * @brief accessor of CameraInfo.
   *
   * you have to call capture() before call this.
   *
   * @return CameraInfo
   */
  inline const sensor_msgs::CameraInfo &getLeftInfo() const
  {
    return info_left_;
  }
  /**
   * @brief accessor of CameraInfo.
   *
   * you have to call capture() before call this.
   *
   * @return CameraInfo
   */
  inline const sensor_msgs::CameraInfo &getRightInfo() const
  {
    return info_right_;
  }

  /**
   * @brief accessor of cv::Mat
   *
   * you have to call capture() before call this.
   *
   * @return captured cv::Mat
   */
  inline const cv::Mat &getCvLeftImage() const
  {
    return bridge_left_.image;
  }
  /**
   * @brief accessor of cv::Mat
   *
   * you have to call capture() before call this.
   *
   * @return captured cv::Mat
   */
  inline const cv::Mat &getCvRightImage() const
  {
    return bridge_right_.image;
  }
  
  /**
   * @brief accessor of ROS Image message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::ImagePtr getImageRightMsgPtr() const
  {
    return bridge_right_.toImageMsg();
  }

  /**
   * @brief accessor of ROS Image message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::ImagePtr getImageLeftMsgPtr() const
  {
    return bridge_left_.toImageMsg();
  }

  /**
   * @brief try capture image width
   * @return true if success
   */
  inline bool setWidth(int32_t width)
  {
    return cap_.set(CV_CAP_PROP_FRAME_WIDTH, width);
  }

  /**
   * @brief try capture image height
   * @return true if success
   */
  inline bool setHeight(int32_t height)
  {
    return cap_.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  }

  /**
   * @brief set CV_PROP_*
   * @return true if success
   */
  bool setPropertyFromParam(int property_id, const std::string &param_name);

private:
  /**
   * @brief rescale camera calibration to another resolution
   */
  void rescaleCameraInfo(int width, int height);

  /**
   * @brief node handle for advertise.
   */
  ros::NodeHandle node_;

  /**
   * @brief ROS image transport utility.
   */
  image_transport::ImageTransport it_;

  /**
   * @brief name of topic without namespace (usually "image_raw").
   */

  std::string topic_name_left_;
  std::string topic_name_right_;

  /**
   * @brief header.frame_id for publishing images.
   */

  std::string frame_id_left_;
  std::string frame_id_right_;
  /**
   * @brief size of publisher buffer
   */
  int32_t buffer_size_;

  /**
   * @brief image publisher created by image_transport::ImageTransport.
   */
  image_transport::CameraPublisher pub_left_;
  image_transport::CameraPublisher pub_right_;

  /**
   * @brief capture device.
   */
  cv::VideoCapture cap_;

  /**
   * @brief this stores last captured image.
   */
  cv_bridge::CvImage bridge_left_;
  cv_bridge::CvImage bridge_right_;

  /**
   * @brief this stores last captured image info.
   *
   * currently this has image size (width/height) only.
   */

  sensor_msgs::CameraInfo info_left_;
  sensor_msgs::CameraInfo info_right_;

  /**
   * @brief camera info manager
   */
  camera_info_manager::CameraInfoManager info_manager_left_;
  camera_info_manager::CameraInfoManager info_manager_right_;

  /**
   * @brief rescale_camera_info param value
   */
  bool rescale_camera_info_;

  /**
   * @brief capture_delay param value
   */
  ros::Duration capture_delay_;
};

} // namespace cv_camera

#endif // CV_CAMERA_CAPTURE_H
