// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // Include for PointCloud2 message type

#include "rclcpp/rclcpp.hpp"

void draw_pcl(cv::Mat &frame){
  cv::circle(frame, cv::Point(100, 100), 100, CV_RGB(255,0,0), -1);
  return;
}

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber() : Node("image_subscriber"), it_(nullptr)
  {
  }

  void init()
  {
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    image_subscription_ = it_->subscribe(
      "/gst_image_pub", 10, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

    // Subscribe to PointCloud2 messages from mmwave_0/points
    pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/mmwave_0/points", 10, std::bind(&ImageSubscriber::pcl_callback, this, std::placeholders::_1));

    // Initialize the publisher
    publisher_ = it_->advertise("/overlay_radar_camera", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      draw_pcl(frame);

      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
      publisher_.publish(out_msg);
      
      cv::imshow("Modified Image", frame);
      cv::waitKey(10);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  // Callback function for handling PointCloud2 messages
  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Process PointCloud2 data here
    RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message with %u points", msg->width);
  }

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription_;
  image_transport::Publisher publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
