#include "cv_camera/driver_stereo.h"
#include <string>

namespace
{
	const double DEFAULT_RATE = 30.0;
	const int32_t PUBLISHER_BUFFER_SIZE = 1;
}

namespace cv_camera
{
	DriverStereo::DriverStereo(ros::NodeHandle &private_node, ros::NodeHandle &camera_node)
		: private_node_(private_node),
		camera_node_(camera_node)
	{

  		//dynamic_reconfigure::Server<cv_camera::CvCameraConfig>::CallbackType f;
		//f = boost::bind(&DriverStereo::callbackDR, this, _1, _2);
		//dynRecServer.setCallback(f);
	}

	void DriverStereo::callbackDR(const cv_camera::CvCameraConfig & config, uint32_t level) {
		ROS_INFO("Level: %d, value: %f", (int) level, config.cv_cap_prop_gain);
	}

	void DriverStereo::setup()
	{
		
		double hz(DEFAULT_RATE);
		int32_t device_id_left(0);
		std::string device_path_left("");
		std::string frame_id_left("camera_left_link");
		std::string file_path_left("");


		int32_t device_id_right(1);
		std::string device_path_right("");
		std::string frame_id_right("camera_right_link");
		std::string file_path_right("");

		private_node_.getParam("device_id_left", device_id_left);
		private_node_.getParam("frame_id_left", frame_id_left);

		private_node_.getParam("device_id_right", device_id_right);
		private_node_.getParam("frame_id_right", frame_id_right);
		private_node_.getParam("rate", hz);

		int32_t image_width(640);
		int32_t image_height(480);

		camera_left_.reset(new Capture(camera_node_, "left", PUBLISHER_BUFFER_SIZE, frame_id_left));

		if (private_node_.getParam("file_left", file_path_left) && file_path_left != "")
		{
			camera_left_->openFile(file_path_left);
		}
		else if (private_node_.getParam("device_path_left", device_path_left) && device_path_left != "")
		{
			camera_left_->open(device_path_left);
		}
		else
		{
			camera_left_->open(device_id_left);
		}

		camera_right_.reset(new Capture(camera_node_, "right", PUBLISHER_BUFFER_SIZE, frame_id_right));

		if (private_node_.getParam("file_right", file_path_right) && file_path_right != "")
		{
			camera_right_->openFile(file_path_right);
		}
		else if (private_node_.getParam("device_path_right", device_path_right) && device_path_right != "")
		{
			camera_right_->open(device_path_right);
		}
		else
		{
			camera_right_->open(device_id_right);
		}

		if (private_node_.getParam("image_width", image_width))
		{
			if (!camera_left_->setWidth(image_width))
			{
				ROS_WARN("fail to set image_width");
			}
			if (!camera_right_->setWidth(image_width))
			{
				ROS_WARN("fail to set image_width");
			}
		}
		if (private_node_.getParam("image_height", image_height))
		{
			if (!camera_left_->setHeight(image_height))
			{
				ROS_WARN("fail to set image_height");
			}
			if (!camera_right_->setHeight(image_height))
			{
				ROS_WARN("fail to set image_height");
			}
		}

		camera_left_->setPropertyFromParam(CV_CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_FPS, "cv_cap_prop_fps");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_FORMAT, "cv_cap_prop_format");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_MODE, "cv_cap_prop_mode");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_SATURATION, "cv_cap_prop_saturation");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_HUE, "cv_cap_prop_hue");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_GAIN, "cv_cap_prop_gain");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");

		camera_left_->setPropertyFromParam(CV_CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
		camera_left_->setPropertyFromParam(CV_CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
		camera_left_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
		camera_left_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
		camera_left_->setPropertyFromParam(CV_CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif // CV_CAP_PROP_BUFFERSIZE


		camera_right_->setPropertyFromParam(CV_CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_FPS, "cv_cap_prop_fps");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_FORMAT, "cv_cap_prop_format");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_MODE, "cv_cap_prop_mode");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_SATURATION, "cv_cap_prop_saturation");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_HUE, "cv_cap_prop_hue");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_GAIN, "cv_cap_prop_gain");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");

		camera_right_->setPropertyFromParam(CV_CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
		camera_right_->setPropertyFromParam(CV_CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
		camera_right_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
		camera_right_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
		camera_right_->setPropertyFromParam(CV_CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif // CV_CAP_PROP_BUFFERSIZE


		rate_.reset(new ros::Rate(hz));

	}

	void DriverStereo::proceed()
	{
		ros::Time stamp = ros::Time::now();
		if(camera_left_->capture(stamp) && camera_right_->capture())
		{
			camera_left_->publish();
			camera_right_->publish();
		}
	}
	DriverStereo::~DriverStereo()
	{
	}
} // namespace cv_camera

