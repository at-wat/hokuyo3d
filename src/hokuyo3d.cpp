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
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>

#include <algorithm>
#include <deque>
#include <string>
#include <thread>

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <vssp.h>

class Hokuyo3dNode : public rclcpp::Node
{
public:
  void cbPoint(
      const vssp::Header &header,
      const vssp::RangeHeader &range_header,
      const vssp::RangeIndex &range_index,
      const boost::shared_array<uint16_t> &index,
      const boost::shared_array<vssp::XYZI> &points,
      const boost::posix_time::ptime &time_read)
  {
    if (timestamp_base_.seconds() == rclcpp::Time(0).seconds())
      return;
    // Pack scan data
    if (enable_pc_)
    {
      if (cloud_.points.size() == 0)
      {
        // Start packing PointCloud message
        cloud_.header.frame_id = frame_id_;
        cloud_.header.stamp = timestamp_base_ + rclcpp::Duration(range_header.line_head_timestamp_ms * 0.001, 0);
      }
      // Pack PointCloud message
      for (int i = 0; i < index[range_index.nspots]; i++)
      {
        if (points[i].r < range_min_)
        {
          continue;
        }
        geometry_msgs::msg::Point32 point;
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = points[i].z;
        cloud_.points.push_back(point);
        cloud_.channels[0].values.push_back(points[i].i);
      }
    }
    if (enable_pc2_)
    {
      if (cloud2_.data.size() == 0)
      {
        // Start packing PointCloud2 message
        cloud2_.header.frame_id = frame_id_;
        cloud2_.header.stamp = timestamp_base_ + rclcpp::Duration(range_header.line_head_timestamp_ms * 0.001, 0);
        cloud2_.row_step = 0;
        cloud2_.width = 0;
      }
      // Pack PointCloud2 message
      cloud2_.data.resize((cloud2_.width + index[range_index.nspots]) * cloud2_.point_step);

      float *data = reinterpret_cast<float *>(&cloud2_.data[0]);
      data += cloud2_.width * cloud2_.point_step / sizeof(float);
      for (int i = 0; i < index[range_index.nspots]; i++)
      {
        if (points[i].r < range_min_)
        {
          continue;
        }
        *(data++) = points[i].x;
        *(data++) = points[i].y;
        *(data++) = points[i].z;
        *(data++) = points[i].i;
        cloud2_.width++;
      }
      cloud2_.row_step = cloud2_.width * cloud2_.point_step;
    }
    // Publish points
    if ((cycle_ == CYCLE_FIELD && (range_header.field != field_ || range_header.frame != frame_)) ||
        (cycle_ == CYCLE_FRAME && (range_header.frame != frame_)) || (cycle_ == CYCLE_LINE))
    {
      if (enable_pc_)
      {
        if (cloud_.header.stamp.sec < cloud_stamp_last_.seconds() && !allow_jump_back_)
        {
          RCLCPP_INFO(this->get_logger(), "Dropping timestamp jump backed cloud");
        }
        else
        {
          pub_pc_->publish(cloud_);
        }
        cloud_stamp_last_ = cloud_.header.stamp;
        cloud_.points.clear();
        cloud_.channels[0].values.clear();
      }
      if (enable_pc2_)
      {
        cloud2_.data.resize(cloud2_.width * cloud2_.point_step);
        if (cloud2_.header.stamp.sec < cloud_stamp_last_.seconds() && !allow_jump_back_)
        {
          RCLCPP_INFO(this->get_logger(), "Dropping timestamp jump backed cloud2");
        }
        else
        {
          pub_pc2_->publish(cloud2_);
        }
        cloud_stamp_last_ = cloud2_.header.stamp;
        cloud2_.data.clear();
      }
      if (range_header.frame != frame_)
        ping();
      frame_ = range_header.frame;
      field_ = range_header.field;
      line_ = range_header.line;
    }
  }
  void cbError(
      const vssp::Header &header,
      const std::string &message,
      const boost::posix_time::ptime &time_read)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  }
  void cbPing(
      const vssp::Header &header,
      const boost::posix_time::ptime &time_read)
  {
    /*
    const ros::Time now = ros::Time::fromBoost(time_read);
    const ros::Duration delay =
        ((now - time_ping_) - ros::Duration(header.send_time_ms * 0.001 - header.received_time_ms * 0.001)) * 0.5;
    const ros::Time base = time_ping_ + delay - ros::Duration(header.received_time_ms * 0.001);
    */
    const rclcpp::Time now = this->now();
    rclcpp::Time delay =
        rclcpp::Time(((now - time_ping_) - rclcpp::Duration(header.send_time_ms * 0.001 - header.received_time_ms * 0.001, 0)).seconds() * 0.5, 0);
        
    const rclcpp::Time base = rclcpp::Time(time_ping_.seconds() + delay.seconds() - header.received_time_ms * 0.001, 0);
    timestamp_base_buffer_.push_back(base);
    if (timestamp_base_buffer_.size() > 5)
      timestamp_base_buffer_.pop_front();

    auto sorted_timstamp_base = timestamp_base_buffer_;
    std::sort(sorted_timstamp_base.begin(), sorted_timstamp_base.end());
    
    if (timestamp_base_.seconds() == rclcpp::Time(0).seconds())
      timestamp_base_ = sorted_timstamp_base[sorted_timstamp_base.size() / 2];
    else
      timestamp_base_ += (sorted_timstamp_base[sorted_timstamp_base.size() / 2] - timestamp_base_) * 0.1;

    RCLCPP_DEBUG(this->get_logger(), "timestamp_base: %lf", timestamp_base_.seconds());
  }
  void cbAux(
      const vssp::Header &header,
      const vssp::AuxHeader &aux_header,
      const boost::shared_array<vssp::Aux> &auxs,
      const boost::posix_time::ptime &time_read)
  {
    if (timestamp_base_.seconds() == rclcpp::Time(0).seconds())
      return;
    
    rclcpp::Time stamp = timestamp_base_ + rclcpp::Duration(aux_header.timestamp_ms * 0.001, 0);

    if ((aux_header.data_bitfield & (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC)) ==
        (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC))
    {
      imu_.header.frame_id = imu_frame_id_;
      imu_.header.stamp = stamp;
      for (int i = 0; i < aux_header.data_count; i++)
      {
        imu_.orientation_covariance[0] = -1.0;
        imu_.angular_velocity.x = auxs[i].ang_vel.x;
        imu_.angular_velocity.y = auxs[i].ang_vel.y;
        imu_.angular_velocity.z = auxs[i].ang_vel.z;
        imu_.linear_acceleration.x = auxs[i].lin_acc.x;
        imu_.linear_acceleration.y = auxs[i].lin_acc.y;
        imu_.linear_acceleration.z = auxs[i].lin_acc.z;
        if (imu_stamp_last_.seconds() > imu_.header.stamp.sec && !allow_jump_back_)
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dropping timestamp jump backed imu");
        }
        else
        {
          pub_imu_->publish(imu_);
        }
        imu_stamp_last_ = imu_.header.stamp;
        imu_.header.stamp.sec += rclcpp::Duration(aux_header.data_ms * 0.001, 0).seconds();
      }
    }
    if ((aux_header.data_bitfield & vssp::AX_MASK_MAG) == vssp::AX_MASK_MAG)
    {
      mag_.header.frame_id = mag_frame_id_;
      mag_.header.stamp = stamp;
      for (int i = 0; i < aux_header.data_count; i++)
      {
        mag_.magnetic_field.x = auxs[i].mag.x;
        mag_.magnetic_field.y = auxs[i].mag.y;
        mag_.magnetic_field.z = auxs[i].mag.z;
        if (mag_stamp_last_.seconds() > imu_.header.stamp.sec && !allow_jump_back_)
        {
          RCLCPP_INFO(this->get_logger(), "Dropping timestamp jump backed mag");
        }
        else
        {
          pub_mag_->publish(mag_);
        }
        mag_stamp_last_ = imu_.header.stamp;
        mag_.header.stamp.sec += rclcpp::Duration(aux_header.data_ms * 0.001, 0).seconds();
      }
    }
  }
  void cbConnect(bool success)
  {
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Connection established");
      ping();
      if (set_auto_reset_)
        driver_.setAutoReset(auto_reset_);
      driver_.setHorizontalInterlace(horizontal_interlace_);
      driver_.requestHorizontalTable();
      driver_.setVerticalInterlace(vertical_interlace_);
      driver_.requestVerticalTable(vertical_interlace_);
      driver_.requestData(true, true);
      driver_.requestAuxData();
      driver_.receivePackets();
      RCLCPP_INFO(this->get_logger(), "Communication started");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Connection failed");
    }
  }
  Hokuyo3dNode()
      : Node("hokuyo3d_node"), timestamp_base_(rclcpp::Time(0)), timer_(io_, boost::posix_time::milliseconds(500))
  {
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);

    parameter_subscription_ = parameter_client_->on_parameter_event(std::bind(&Hokuyo3dNode::onParameterEvent, this, std::placeholders::_1));

    // Setup parameter client
    while (!parameter_client_->wait_for_service(std::chrono::seconds(5))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(
        this->get_logger(),
        "service not available, waiting again...");
    }

    if (this->get_parameter("horizontal_interlace", horizontal_interlace_) || !this->get_parameter("interlace", vertical_interlace_))
    {
      horizontal_interlace_ = this->declare_parameter<int>("horizontal_interlace", 4);
    }
    else if (this->has_parameter("interlace"))
    {
      RCLCPP_WARN(this->get_logger(), "'interlace' parameter is deprecated. Use horizontal_interlace instead.");
      horizontal_interlace_ = this->declare_parameter("interlace", 4);
    }
    vertical_interlace_ = this->declare_parameter("vertical_interlace", 1);
    ip_ = this->declare_parameter("ip", std::string("192.168.0.10"));
    port_ = this->declare_parameter("port", 10940);
    frame_id_ = this->declare_parameter("frame_id", std::string("hokuyo3d"));
    imu_frame_id_ = this->declare_parameter("imu_frame_id", frame_id_ + "_imu");
    mag_frame_id_ = this->declare_parameter("mag_frame_id", frame_id_ + "_mag");
    range_min_ = this->declare_parameter("range_min", 0.0);
    set_auto_reset_ = this->has_parameter("auto_reset");
    auto_reset_ = this->declare_parameter("auto_reset", false);

    allow_jump_back_ = this->declare_parameter("allow_jump_back", false);

    std::string output_cycle;
    output_cycle = this->declare_parameter("output_cycle", std::string("field"));

    if (output_cycle.compare("frame") == 0)
      cycle_ = CYCLE_FRAME;
    else if (output_cycle.compare("field") == 0)
      cycle_ = CYCLE_FIELD;
    else if (output_cycle.compare("line") == 0)
      cycle_ = CYCLE_LINE;
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown output_cycle value %s", output_cycle.c_str());
      rclcpp::shutdown();
    }

    driver_.setTimeout(2.0);
    RCLCPP_INFO(this->get_logger(), "Connecting to %s", ip_.c_str());
    driver_.registerCallback(boost::bind(&Hokuyo3dNode::cbPoint, this, _1, _2, _3, _4, _5, _6));
    driver_.registerAuxCallback(boost::bind(&Hokuyo3dNode::cbAux, this, _1, _2, _3, _4));
    driver_.registerPingCallback(boost::bind(&Hokuyo3dNode::cbPing, this, _1, _2));
    driver_.registerErrorCallback(boost::bind(&Hokuyo3dNode::cbError, this, _1, _2, _3));
    field_ = 0;
    frame_ = 0;
    line_ = 0;

    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = std::string("intensity");
    cloud_.channels.push_back(channel);

    cloud2_.height = 1;
    cloud2_.is_bigendian = false;
    cloud2_.is_dense = false;
    sensor_msgs::PointCloud2Modifier pc2_modifier(cloud2_);
    pc2_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 5);
    pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 5);

    enable_pc_ = enable_pc2_ = false;
    cloud_publish_timer_callback_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                                        std::bind(&Hokuyo3dNode::cbSubscriber, this));

    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud>("hokuyo_cloud", 5);
    pub_pc2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("hokuyo_cloud2", 5);

    // Start communication with the sensor
    driver_.connect(ip_.c_str(), port_, boost::bind(&Hokuyo3dNode::cbConnect, this, _1));
  }
  ~Hokuyo3dNode()
  {
    driver_.requestAuxData(false);
    driver_.requestData(true, false);
    driver_.requestData(false, false);
    driver_.poll();
    RCLCPP_INFO(this->get_logger(), "Communication stoped");
  }
  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    for (auto & changed_parameter : event->changed_parameters) {
      updateParameterValue(changed_parameter);
    }
  }
  void updateParameterValue(const rcl_interfaces::msg::Parameter param)
  {
    if(param.name == "allow_jump_back")
    {
      allow_jump_back_ = param.value.bool_value;
    } else if(param.name == "auto_reset")
    {
      auto_reset_ = param.value.bool_value;
    } else if(param.name == "frame_id")
    {
      frame_id_ = param.value.string_value;
    } else if(param.name == "horizontal_interlace")
    {
      horizontal_interlace_ = param.value.integer_value;
    } else if(param.name == "imu_frame_id")
    {
      imu_frame_id_ = param.value.string_value;
    } else if(param.name == "ip")
    {
      ip_ = param.value.string_value;
    } else if(param.name == "mag_frame_id")
    {
      mag_frame_id_ = param.value.string_value;
    } else if(param.name == "output_cycle")
    {
      std::string output_cycle = param.value.string_value;
      if (output_cycle.compare("frame") == 0)
        cycle_ = CYCLE_FRAME;
      else if (output_cycle.compare("field") == 0)
        cycle_ = CYCLE_FIELD;
      else if (output_cycle.compare("line") == 0)
        cycle_ = CYCLE_LINE;
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown output_cycle value %s", output_cycle.c_str());
        rclcpp::shutdown();
      }
    } else if(param.name == "port")
    {
      port_ = param.value.integer_value;
    } else if(param.name == "range_min")
    {
      range_min_ = param.value.double_value;
    } else if(param.name == "use_sim_time")
    {
    } else if(param.name == "vertical_interlace")
    {
      vertical_interlace_ = param.value.integer_value;
    }
  }

  void cbSubscriber()
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (pub_pc_->get_subscription_count() > 0)
    {
      enable_pc_ = true;
      RCLCPP_DEBUG(this->get_logger(), "PointCloud output enabled");
    }
    else
    {
      enable_pc_ = false;
      RCLCPP_DEBUG(this->get_logger(), "PointCloud output disabled");
    }
    if (pub_pc2_->get_subscription_count() > 0)
    {
      enable_pc2_ = true;
      RCLCPP_DEBUG(this->get_logger(), "PointCloud2 output enabled");
    }
    else
    {
      enable_pc2_ = false;
      RCLCPP_DEBUG(this->get_logger(), "PointCloud2 output disabled");
    }
  }
  bool poll()
  {
    if (driver_.poll())
    {
      return true;
    }
    RCLCPP_INFO(this->get_logger(), "Connection closed");
    return false;
  }
  void cbTimer(const boost::system::error_code &error)
  {
    if (error)
      return;

    if (!rclcpp::ok())
    {
      driver_.stop();
    }
    else
    {
      timer_.expires_at(
          timer_.expires_at() +
          boost::posix_time::milliseconds(500));
      timer_.async_wait(boost::bind(&Hokuyo3dNode::cbTimer, this, _1));
    }
  }
  void spin()
  {
    timer_.async_wait(boost::bind(&Hokuyo3dNode::cbTimer, this, _1));
    boost::thread thread(
        boost::bind(&boost::asio::io_service::run, &io_));

    rclcpp::executors::MultiThreadedExecutor executor;
    std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));
    executor.add_node(this->get_node_base_interface());
    // spinner.start();
    driver_.spin();
    // spinner.stop();
    timer_.cancel();
    RCLCPP_INFO(this->get_logger(), "Connection closed");
  }
  void ping()
  {
    driver_.requestPing();
    time_ping_ = this->now();
  }

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_pc_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc2_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  vssp::VsspDriver driver_;
  sensor_msgs::msg::PointCloud cloud_;
  sensor_msgs::msg::PointCloud2 cloud2_;
  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::MagneticField mag_;
  rclcpp::TimerBase::SharedPtr cloud_publish_timer_callback_;
  std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent,
    std::allocator<void>>> parameter_subscription_;

  bool enable_pc_;
  bool enable_pc2_;
  bool allow_jump_back_;
  boost::mutex connect_mutex_;

  rclcpp::Time time_ping_;
  rclcpp::Time timestamp_base_;
  std::deque<rclcpp::Time> timestamp_base_buffer_;
  rclcpp::Time imu_stamp_last_;
  rclcpp::Time mag_stamp_last_;
  rclcpp::Time cloud_stamp_last_;

  boost::asio::io_service io_;
  boost::asio::deadline_timer timer_;

  int field_;
  int frame_;
  int line_;

  enum PublishCycle
  {
    CYCLE_FIELD,
    CYCLE_FRAME,
    CYCLE_LINE
  };
  PublishCycle cycle_;
  std::string ip_;
  int port_;
  int horizontal_interlace_;
  int vertical_interlace_;
  double range_min_;
  std::string frame_id_;
  std::string imu_frame_id_;
  std::string mag_frame_id_;
  bool auto_reset_;
  bool set_auto_reset_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Hokuyo3dNode>();
  // rclcpp::spin_some(node);
  node->spin();
  
  // while(rclcpp::ok())
  // {
  //   rclcpp::spin_some(node);
  // }

  return 1;
}
