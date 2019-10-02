// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>
#include <ros/ros.h>
#include <cv_camera/driver.h>
#include <dynamic_reconfigure/server.h>
#include <cv_camera/CvCameraConfig.h>
#include <cv_camera/TutorialsConfig.h>


void callback(cv_camera::CvCameraConfig & config, uint32_t level) {
	ROS_INFO("%f", config.cv_cap_prop_gain);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_camera");
  ros::NodeHandle private_node("~");
  cv_camera::Driver driver(private_node, private_node);

  dynamic_reconfigure::Server<cv_camera::CvCameraConfig> server;
 dynamic_reconfigure::Server<cv_camera::CvCameraConfig>::CallbackType f;
 f = boost::bind(&callback, _1, _2);
 server.setCallback(f);


  try
  {
    driver.setup();
    while (ros::ok())
    {
      driver.proceed();
      ros::spinOnce();
    }
  }
  catch (cv_camera::DeviceError &e)
  {
    ROS_ERROR_STREAM("cv camera open failed: " << e.what());
    return 1;
  }

  return 0;
}
