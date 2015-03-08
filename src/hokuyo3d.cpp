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
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "vssp.hpp"


class hokuyo3d_node
{
	public:
		void cbPoint(
				const vssp::header &header, 
				const vssp::range_header &range_header, 
				const vssp::range_index &range_index,
				const boost::shared_array<uint16_t> &index,
				const boost::shared_array<vssp::xyzi> &points,
				const std::chrono::microseconds &delayRead)
		{
			if(timestampBase == ros::Time(0)) return;
			// Pack scan data
			if(enablePc)
			{
				if(cloud.points.size() == 0)
				{
					// Start packing PointCloud message
					cloud.header.frame_id = frame_id;
					cloud.header.stamp = timestampBase
						+ ros::Duration(range_header.line_head_timestamp_ms * 0.001);
				}
				// Pack PointCloud message
				for(int i = 0; i < index[range_index.nspots]; i ++)
				{
					if(points[i].r < range_min)
					{
						continue;
					}
					geometry_msgs::Point32 point;
					point.x = points[i].x;
					point.y = points[i].y;
					point.z = points[i].z;
					cloud.points.push_back(point);
					cloud.channels[0].values.push_back(points[i].i);
				}
			}
			if(enablePc2)
			{
				if(cloud2.data.size() == 0)
				{
					// Start packing PointCloud2 message
					cloud2.header.frame_id = frame_id;
					cloud2.header.stamp = timestampBase
						+ ros::Duration(range_header.line_head_timestamp_ms * 0.001);
					cloud2.row_step = 0;
					cloud2.width = 0;
				}
				// Pack PointCloud2 message
				cloud2.data.resize((cloud2.width + index[range_index.nspots])
					   	* cloud2.point_step);

				float *data = reinterpret_cast<float*>(&cloud2.data[0]);
				data += cloud2.width * cloud2.point_step / sizeof(float);
				for(int i = 0; i < index[range_index.nspots]; i ++)
				{
					if(points[i].r < range_min)
					{
						continue;
					}
					*(data++) = points[i].x;
					*(data++) = points[i].y;
					*(data++) = points[i].z;
					*(data++) = points[i].i;
					cloud2.width ++;
				}
				cloud2.row_step = cloud2.width * cloud2.point_step;
			}
			// Publish points
			if((cycle == CYCLE_FIELD &&
						(range_header.field != field ||
						 range_header.frame != frame)) ||
					(cycle == CYCLE_FRAME &&
						(range_header.frame != frame)) ||
					(cycle == CYCLE_LINE))
			{
				if(enablePc)
				{
					pubPc.publish(cloud);
					cloud.points.clear();
					cloud.channels[0].values.clear();
				}
				if(enablePc2)
				{
					cloud2.data.resize(cloud2.width * cloud2.point_step);
					pubPc2.publish(cloud2);
					cloud2.data.clear();
				}
				if(range_header.frame != frame) ping();
				frame = range_header.frame;
				field = range_header.field;
				line = range_header.line;
			}
		};
		void cbError(const vssp::header &header, const std::string &message)
		{
			ROS_ERROR("%s", message.c_str());
		}
		void cbPing(const vssp::header &header, const std::chrono::microseconds &delayRead)
		{
			ros::Time now = ros::Time::now() - ros::Duration(delayRead.count() * 0.001 * 0.001);
			ros::Duration delay = ((now - timePing)
					- ros::Duration(header.send_time_ms * 0.001 - header.received_time_ms * 0.001)) * 0.5;
			ros::Time base = timePing + delay - ros::Duration(header.received_time_ms * 0.001);
			if(timestampBase == ros::Time(0)) timestampBase = base;
			else timestampBase += (base - timestampBase) * 0.01;
		}
		void cbAux(
				const vssp::header &header, 
				const vssp::aux_header &aux_header, 
				const boost::shared_array<vssp::aux> &auxs,
				const std::chrono::microseconds &delayRead)
		{
			if(timestampBase == ros::Time(0)) return;
			ros::Time stamp = timestampBase + ros::Duration(aux_header.timestamp_ms * 0.001);

			if((aux_header.data_bitfield & (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC))
					== (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC))
			{
				imu.header.frame_id = frame_id;
				imu.header.stamp = stamp;
				for(int i = 0; i < aux_header.data_count; i ++)
				{
					imu.orientation_covariance[0] = -1.0;
					imu.angular_velocity.x = auxs[i].ang_vel.x;
					imu.angular_velocity.y = auxs[i].ang_vel.y;
					imu.angular_velocity.z = auxs[i].ang_vel.z;
					imu.linear_acceleration.x = auxs[i].lin_acc.x;
					imu.linear_acceleration.y = auxs[i].lin_acc.y;
					imu.linear_acceleration.z = auxs[i].lin_acc.z;
					pubImu.publish(imu);
					imu.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
				}
			}
			if((aux_header.data_bitfield & vssp::AX_MASK_MAG) == vssp::AX_MASK_MAG )
			{
				mag.header.frame_id = frame_id;
				mag.header.stamp = stamp;
				for(int i = 0; i < aux_header.data_count; i ++)
				{
					mag.magnetic_field.x = auxs[i].mag.x;
					mag.magnetic_field.y = auxs[i].mag.y;
					mag.magnetic_field.z = auxs[i].mag.z;
					pubMag.publish(mag);
					mag.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
				}
			}
		};
		void cbConnect(bool success)
		{
			if(success)
			{
				ROS_INFO("Connection established");
				ping();
				driver.setInterlace(interlace);
				driver.requestHorizontalTable();
				driver.requestVerticalTable();
				driver.requestData(true, true);
				driver.requestAuxData();
				driver.receivePackets();
				ROS_INFO("Communication started");
			}
			else
			{
				ROS_ERROR("Connection failed");
			}
		};
		hokuyo3d_node() :
			nh("~"),
			timestampBase(0)
		{
			nh.param("interlace", interlace, 4);
			nh.param("ip", ip, std::string("192.168.0.10"));
			nh.param("port", port, 10940);
			nh.param("frame_id", frame_id, std::string("hokuyo3d"));
			nh.param("range_min", range_min, 0.0);

			std::string output_cycle;
			nh.param("output_cycle", output_cycle, std::string("field"));

			if(output_cycle.compare("frame") == 0)
				cycle = CYCLE_FRAME;
			else if(output_cycle.compare("field") == 0)
				cycle = CYCLE_FIELD;
			else if(output_cycle.compare("line") == 0)
				cycle = CYCLE_LINE;
			else
			{
				ROS_ERROR("Unknown output_cycle value %s", output_cycle.c_str());
				ros::shutdown();
			}

			driver.setTimeout(2.0);
			ROS_INFO("Connecting to %s", ip.c_str());
			driver.connect(ip.c_str(), port, 
					boost::bind(&hokuyo3d_node::cbConnect, this, _1));
			driver.registerCallback(
					boost::bind(&hokuyo3d_node::cbPoint, this, _1, _2, _3, _4, _5, _6));
			driver.registerAuxCallback(
					boost::bind(&hokuyo3d_node::cbAux, this, _1, _2, _3, _4));
			driver.registerPingCallback(
					boost::bind(&hokuyo3d_node::cbPing, this, _1, _2));
			driver.registerErrorCallback(
					boost::bind(&hokuyo3d_node::cbError, this, _1, _2));
			field = 0;
			frame = 0;
			line = 0;

			sensor_msgs::ChannelFloat32 channel;
			channel.name = std::string("intensity");
			cloud.channels.push_back(channel);

			cloud2.height = 1;
			cloud2.is_bigendian = false;
			cloud2.is_dense = false;
			sensor_msgs::PointCloud2Modifier pc2_modifier(cloud2);
			pc2_modifier.setPointCloud2Fields(4,
					"x", 1, sensor_msgs::PointField::FLOAT32,
					"y", 1, sensor_msgs::PointField::FLOAT32,
					"z", 1, sensor_msgs::PointField::FLOAT32,
					"intensity", 1, sensor_msgs::PointField::FLOAT32);

			pubImu = nh.advertise<sensor_msgs::Imu>("imu", 5);
			pubMag = nh.advertise<sensor_msgs::MagneticField>("mag", 5);

			enablePc = enablePc2 = false;
			ros::SubscriberStatusCallback cbCon =
				boost::bind(&hokuyo3d_node::cbSubscriber, this);

			std::lock_guard<std::mutex> lock(connect_mutex);
			pubPc = nh.advertise<sensor_msgs::PointCloud>("hokuyo_cloud", 5, cbCon, cbCon);
			pubPc2 = nh.advertise<sensor_msgs::PointCloud2>("hokuyo_cloud2", 5, cbCon, cbCon);
		};
		~hokuyo3d_node()
		{
			driver.requestAuxData(false);
			driver.requestData(true, false);
			driver.requestData(false, false);
			driver.poll();
			ROS_INFO("Communication stoped");
		};
		void cbSubscriber()
		{
			std::lock_guard<std::mutex> lock(connect_mutex);
			if(pubPc.getNumSubscribers() > 0)
			{
				enablePc = true;
				ROS_DEBUG("PointCloud output enabled");
			}
			else
			{
				enablePc = false;
				ROS_DEBUG("PointCloud output disabled");
			}
			if(pubPc2.getNumSubscribers() > 0)
			{
				enablePc2 = true;
				ROS_DEBUG("PointCloud2 output enabled");
			}
			else
			{
				enablePc2 = false;
				ROS_DEBUG("PointCloud2 output disabled");
			}
		}
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
			timePing = ros::Time::now();
		};
	private:
		ros::NodeHandle nh;
		ros::Publisher pubPc;
		ros::Publisher pubPc2;
		ros::Publisher pubImu;
		ros::Publisher pubMag;
		vssp::vsspDriver driver;
		sensor_msgs::PointCloud cloud;
		sensor_msgs::PointCloud2 cloud2;
		sensor_msgs::Imu imu;
		sensor_msgs::MagneticField mag;
				
		bool enablePc;
		bool enablePc2;
		std::mutex connect_mutex;

		ros::Time timePing;
		ros::Time timestampBase;

		int field;
		int frame;
		int line;

		enum
		{
			CYCLE_FIELD,
			CYCLE_FRAME,
			CYCLE_LINE
		} cycle;
		std::string ip;
		int port;
		int interlace;
		double range_min;
		std::string frame_id;
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

