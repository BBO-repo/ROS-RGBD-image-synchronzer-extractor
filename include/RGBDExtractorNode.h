// Headers
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <err.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <ctime>
#include <cmath>


class RGBDExtractorNode
{

private:
	ros::NodeHandle nh_;

	message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
    typedef message_filters::Synchronizer<sync_policy> Sync;
    boost::shared_ptr<Sync> sync_;

public:
	RGBDExtractorNode(const ros::NodeHandle &nh);
	~RGBDExtractorNode();

	void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth );

private:
	int n_synchro_;
	std::string output_folder_;
	std::string rgb_sub_str_;
	std::string depth_sub_str_;

	bool paramValidation(std::string &errmsg);

};
