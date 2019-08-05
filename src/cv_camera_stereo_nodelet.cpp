// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver_stereo.h"

#include <nodelet/nodelet.h>
#include <boost/thread.hpp>

namespace cv_camera
{

/**
 * @brief Nodelet version of cv_camera.
 */
class CvCameraStereoNodelet : public nodelet::Nodelet
{
public:
  CvCameraStereoNodelet() : is_running_(false)
  {
  }
  ~CvCameraStereoNodelet()
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
    driver_stereo_.reset(new DriverStereo(getPrivateNodeHandle(),
                             getPrivateNodeHandle()));
    try
    {
      driver_stereo_->setup();
      is_running_ = true;
      thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CvCameraStereoNodelet::main, this)));
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
      driver_stereo_->proceed();
    }
  }

  /**
   * @brief true is thread is running.
   */
  bool is_running_;

  /**
   * @brief ROS cv camera driver.
   */
  boost::shared_ptr<DriverStereo> driver_stereo_;

  /**
   * @brief thread object for main loop.
   */
  boost::shared_ptr<boost::thread> thread_;
};

} // end namespace cv_camera

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cv_camera::CvCameraStereoNodelet, nodelet::Nodelet)
