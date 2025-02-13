// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver_zed.h"

#include <nodelet/nodelet.h>
#include <boost/thread.hpp>

namespace cv_camera
{

/**
 * @brief Nodelet version of cv_camera.
 */
  class CvCameraZedNodelet : public nodelet::Nodelet
{
public:
  CvCameraZedNodelet() : is_running_(false)
  {
  }
  ~CvCameraZedNodelet()
  {
    if (is_running_)
    {
      is_running_ = false;
      thread_->join();
    }
  }

private:
  /**
   * @brief Start capture/publish thread.
   */
  virtual void onInit()
  {
    driver_.reset(new DriverZed(getPrivateNodeHandle(),
                             getPrivateNodeHandle()));
    try
    {
      driver_->setup();
      is_running_ = true;
      thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CvCameraZedNodelet::main, this)));
    }
    catch (cv_camera::DeviceError &e)
    {
      NODELET_ERROR_STREAM("failed to open device... do nothing: " << e.what());
    }
  }

  /**
   * @brief capture and publish.
   */
  void main()
  {
    while (is_running_)
    {
      driver_->proceed();
    }
  }

  /**
   * @brief true is thread is running.
   */
  bool is_running_;

  /**
   * @brief ROS cv camera driver.
   */
  boost::shared_ptr<DriverZed> driver_;

  /**
   * @brief thread object for main loop.
   */
  boost::shared_ptr<boost::thread> thread_;
};

} // end namespace cv_camera

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cv_camera::CvCameraZedNodelet, nodelet::Nodelet)
