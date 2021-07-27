#include "RGBDExtractorNode.h"

RGBDExtractorNode::RGBDExtractorNode(const ros::NodeHandle &nh) : nh_{nh}, n_synchro_{0}
{

	ROS_INFO("Starting cpp ros node RGBD extractor node");

	// check parameters are valid
	std::string errmsg{""};
	if (!paramValidation(errmsg))
	{
		ROS_ERROR("%s", errmsg.c_str());
		ros::shutdown();
		exit(1);
	}

	ROS_INFO("output folder: %s", output_folder_.c_str());
	ROS_INFO("RGB topic: [%s]", rgb_sub_str_.c_str());
	ROS_INFO("depth topic: [%s]", depth_sub_str_.c_str());

	// topic subscribed to
	rgb_sub_.subscribe(nh_, rgb_sub_str_, 5);
	depth_sub_.subscribe(nh_, depth_sub_str_, 5);

	sync_.reset(new Sync(sync_policy(20), rgb_sub_, depth_sub_));
	sync_->registerCallback(boost::bind(&RGBDExtractorNode::callback, this, _1, _2));
}

RGBDExtractorNode::~RGBDExtractorNode()
{
	ROS_INFO("Extracted [%d] RGBD images in folder: [%s]", n_synchro_, output_folder_.c_str());
}

void RGBDExtractorNode::callback(const sensor_msgs::ImageConstPtr &msg_rgb, const sensor_msgs::ImageConstPtr &msg_depth)
{
	cv_bridge::CvImagePtr rgb_ptr;
	cv_bridge::CvImagePtr depth_ptr;
	// convert image to opencv
	try
	{
		if (msg_depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
		{
			rgb_ptr = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
			depth_ptr = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);

			// increment synchronization number;
			++n_synchro_;
			std::string rgb_file = output_folder_ + std::to_string(n_synchro_) + "_rgb.png";
			std::string depth_file = output_folder_ + std::to_string(n_synchro_) + "_depth.png";

			cv::imwrite(rgb_file, rgb_ptr->image);
			cv::imwrite(depth_file, depth_ptr->image);
			ROS_INFO("RGB-D image [%d] saved in [%s]", n_synchro_, output_folder_.c_str());
		}
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}
}

bool RGBDExtractorNode::paramValidation(std::string &errmsg)
{
	bool status = true;

	// check parameters output_folder
	if (!nh_.getParam("output_folder", output_folder_) || output_folder_.empty())
	{
		errmsg += "Could not find valid parameter for [output_folder]\n";
		status = false;
	}
	// check parameters rgb_sub_topic
	if (!nh_.getParam("rgb_sub_topic", rgb_sub_str_))
	{
		errmsg += "Could not find parameter [rgb_topic]\n";
		status = false;
	}
	// check parameters depth_sub_topic
	if (!nh_.getParam("depth_sub_topic", depth_sub_str_))
	{
		errmsg += "Could not find parameter [depth_topic]\n";
		status = false;
	}

	// complete error message if required
	if (!status)
	{
		errmsg = "Some parameters are missing RGBD extract node will stop!!\n" + errmsg;
	}
	return status;
}
