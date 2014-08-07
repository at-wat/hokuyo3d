/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "vssp.hpp"


class hokuyo3d_node
{
	public:
		void cbPoint(
				const vssp::header &header, 
				const vssp::range_header &range_header, 
				const vssp::range_index &range_index,
				const boost::shared_array<vssp::xyzi> &points)
		{
			if(cloud.points.size() == 0)
			{
				cloud.header.frame_id = "hokuyo3d";
				cloud.header.stamp = ros::Time::now();
				// TODO: Use timestamp from sensor
			}
			// Pack scan data
			for(int i = 0; i < range_index.nspots; i ++)
			{
				geometry_msgs::Point32 point;
				point.x = points[i].x;
				point.y = points[i].y;
				point.z = points[i].z;
				cloud.points.push_back(point);
				cloud.channels[0].values.push_back(points[i].i);
			}
			// Publish frame
			if(range_header.field != field ||
				range_header.frame != frame)
			{
				pubPc.publish(cloud);
				field = range_header.field;
				frame = range_header.frame;
				cloud.points.clear();
				cloud.channels[0].values.clear();
			}
		};
		void cbConnect(bool success)
		{
			if(success)
			{
				driver.setInterlace(interlace);
				driver.requestHorizontalTable();
				driver.requestVerticalTable();
				driver.requestData(true, true);
				driver.receivePackets();
			}
			else
			{
				ROS_ERROR("Connection failure");
			}
		};
		hokuyo3d_node() :
			nh("~")
		{
			nh.param("interlace", interlace, 4);
			nh.param("ip", ip, std::string("192.168.0.10"));
			nh.param("port", port, 10940);
			pubPc = nh.advertise<sensor_msgs::PointCloud>("hokuyo_cloud", 5);

			driver.setTimeout(2.0);
			driver.connect(ip.c_str(), port, 
					boost::bind(&hokuyo3d_node::cbConnect, this, _1));
			driver.registerCallback(
					boost::bind(&hokuyo3d_node::cbPoint, this, _1, _2, _3, _4));
			field = 0;
			frame = 0;

			sensor_msgs::ChannelFloat32 channel;
			channel.name = std::string("intensity");
			cloud.channels.push_back(channel);
		};
		~hokuyo3d_node()
		{
			driver.requestData(true, false);
		};
		bool poll()
		{
			if(driver.poll())
			{
				return true;
			}
			ROS_ERROR("Connection closed");
			return false;
		};
		void ping()
		{
			driver.requestPing();
		};
	private:
		ros::NodeHandle nh;
		ros::Publisher pubPc;
		vssp::vsspDriver driver;
		sensor_msgs::PointCloud cloud;

		int field;
		int frame;

		std::string ip;
		int port;
		int interlace;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hokuyo3d");
	hokuyo3d_node node;

	ros::Rate wait(200);
	
	while (ros::ok())
	{
		if(!node.poll()) break;
		ros::spinOnce();
		wait.sleep();
	}

	return 1;
}

