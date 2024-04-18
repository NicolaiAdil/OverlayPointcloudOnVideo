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

#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl/impl/point_types.hpp"
#include "rclcpp/rclcpp.hpp"

struct CameraIntrinsics {
  float fx, fy; // Focal lengths in x and y
  float cx, cy; // Optical center coordinates
};


class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber() : Node("image_subscriber"), it_(nullptr)
  {
    pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  void init()
  {
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    image_subscription_ = it_->subscribe(
      "/gst_image_pub", 10, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

    pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/mmwave_0/points", 10, std::bind(&ImageSubscriber::pcl_callback, this, std::placeholders::_1));

    publisher_ = it_->advertise("/overlay_radar_camera", 10);

    intrinsics = {1288.56, 1293.73, 1179.74, 802.44};
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      // Transform the ros msg to cv::Mat, and draw the point cloud on it
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      draw_pcl(frame);

      // Transform the cv::Mat back to ros msg and publish it
      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
      publisher_.publish(out_msg);
      
      // Display the modified image for debugging
      cv::imshow("Modified Image", frame);
      cv::waitKey(10);
    } 
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->data.empty()) {
      pcl_cloud_->points.clear();
      return;
    }
    // Convert the point cloud to pcl format so that we can draw it on the image
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
    pcl::fromROSMsg(*msg, *pcl_cloud_);
  }

  void draw_pcl(cv::Mat &frame)
  {
    if (pcl_cloud_->points.size() == 0) {
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "Drawing point cloud");
    for (const auto& pt : pcl_cloud_->points) {
      // y is straight ahead, z is up, and x is to the right. Convert to make sense.
      float logical_x = static_cast<float>(pt.y);
      float logical_y = static_cast<float>(-pt.x);
      float logical_z = static_cast<float>(pt.z);

      if (logical_x <= 0) continue;  // Ensure the point is in front of the camera

      // Formula from: https://towardsdatascience.com/what-are-intrinsic-and-extrinsic-camera-parameters-in-computer-vision-7071b72fb8ec
      int pixel_x = static_cast<int>(logical_x * intrinsics.fx / logical_z + intrinsics.cx);
      int pixel_y = static_cast<int>(logical_y * intrinsics.fy / logical_z + intrinsics.cy);

      if (pixel_x < 0 || pixel_x > frame.cols || pixel_y < 0 || pixel_y > frame.rows) {
          continue;
      }

      float depth = sqrt(pow(logical_x, 2) + pow(logical_y, 2) + pow(logical_z, 2));
      RCLCPP_INFO(this->get_logger(), "Depth: %f, (%f, %f, %f)", depth, logical_x, logical_y, logical_z);
      cv::Scalar color = depth_to_color(depth);
      
      cv::circle(frame, cv::Point(pixel_x, pixel_y), 10, color, -1);
    }
  }

  cv::Scalar depth_to_color(float depth)
  {
    // Adjust these ranges and colors as necessary
    if (depth < 1.0) return CV_RGB(255, 0, 0);     // Red for very close
    else if (depth < 3.0) return CV_RGB(0, 255, 0); // Green for close
    else if (depth < 5.0) return CV_RGB(0, 0, 255); // Blue for moderate
    else return CV_RGB(255, 255, 255);          // White for far
  }

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription_;
  image_transport::Publisher publisher_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_;
  CameraIntrinsics intrinsics;
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