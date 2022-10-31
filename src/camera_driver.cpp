/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_calibration_parsers/parse.h>

#include <jetson-utils/gstCamera.h>

#include <jetson-utils/videoOptions.h>

#include "image_converter.h"



// globals	
gstCamera* camera = NULL;

imageConverter* camera_cvt = NULL;
ros::Publisher* camera_pub = NULL;
ros::Publisher* camera_info_pub = NULL;

sensor_msgs::CameraInfo camera_info_msg;


// aquire and publish camera frame
bool aquireFrame()
{
	float4* imgRGBA = NULL;

	// get the latest frame
	if( !camera->CaptureRGBA((float**)&imgRGBA, 1000) )
	{
		ROS_ERROR("failed to capture camera frame");
		return false;
	}

	ros::Time stamp = ros::Time::now();

	// assure correct image size
	if( !camera_cvt->Resize(camera->GetWidth(), camera->GetHeight(), IMAGE_RGBA32F) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;
	msg.header.frame_id = "jetbot_camera";
	msg.header.stamp = stamp;

	camera_info_msg.header = msg.header;

	if( !camera_cvt->Convert(msg, imageConverter::ROSOutputFormat, imgRGBA, camera_info_msg) )
	{
		ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
		return false;
	}

	// publish the message
	camera_pub->publish(msg);
	camera_info_pub->publish(camera_info_msg);
	ROS_INFO("published camera frame");
	return true;
}


// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "jetbot_camera");
 
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */
	std::string camera_device = "0";	// MIPI CSI camera by default
	int width = 1280;
	int height = 720;
	int frame_rate = 10;
	std::string camera_info_url = "";
	bool preflip = false;
	bool undistort = false;

	private_nh.param<std::string>("device", camera_device, camera_device);
	private_nh.param<int>("frame_rate", frame_rate, frame_rate);
	private_nh.param<int>("width", width, width);
	private_nh.param<int>("height", height, height);
	private_nh.param<std::string>("camera_info_url", camera_info_url, camera_info_url);
	private_nh.param<bool>("preflip", preflip, preflip);
	private_nh.param<bool>("undistort", undistort, undistort);
	
	ROS_INFO("opening camera device %s", camera_device.c_str());

	videoOptions opt;
	opt.resource = camera_device;
	opt.width = width;
	opt.height = height;
	opt.frameRate = frame_rate;
	opt.ioType = videoOptions::INPUT;

	/*
	 * open camera device
	 */
	camera = gstCamera::Create(opt);

	if( !camera )
	{
		ROS_ERROR("failed to open camera device %s", camera_device.c_str());
		return 0;
	}

	/*
	 * advertise publisher topics
	 */
	ros::Publisher camera_publisher = nh.advertise<sensor_msgs::Image>("image_raw", 2);
	camera_pub = &camera_publisher;

	ros::Publisher camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 2);
	camera_info_pub = &camera_info_publisher;

	/*
	 * start the camera streaming
	 */
	if( !camera->Open() )
	{
		ROS_ERROR("failed to start camera streaming");
		return 0;
	}

	/*
	 * load camera parameters, if calibrated
	 */
	if ( !camera_info_url.empty() )
	{
		std::string camera_name;
		bool loaded = camera_calibration_parsers::readCalibration(camera_info_url, camera_name, camera_info_msg);
		if ( !loaded )
		{
			ROS_ERROR("Error reading calibraion file: %s", camera_info_url.c_str());
			if ( undistort )
			{
				ROS_WARN("No calibration data, disable rectification");
				undistort = false;
			}
		}
		else
		{
			ROS_INFO("Camera parameters loaded from %s:", camera_info_url.c_str());
			std::cout << camera_info_msg;
		}
	}

	/*
	 * create image converter
	 */
	camera_cvt = new imageConverter(preflip, undistort);

	if( !camera_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}

	/*
	 * start publishing video frames
	 */
	ros::Rate rate(frame_rate);
	while( ros::ok() )
	{
		if( camera_pub->getNumSubscribers() > 0 )
			aquireFrame();

		ros::spinOnce();
		rate.sleep();
	}

	delete camera;
	return 0;
}
