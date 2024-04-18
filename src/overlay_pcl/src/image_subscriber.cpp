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
#include <map>
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
  cv::Mat cameraMatrix; // Camera matrix (K)
  cv::Mat distCoeffs;   // Distortion coefficients (D)
  float fx, fy;         // Focal lengths in x and y
  float cx, cy;         // Optical center coordinates
};

struct TimedPoint {
  pcl::PointXYZ point;
  int age; // Age or time since this point was added, used for fading
};

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber() : Node("image_subscriber"), it_(nullptr)
  {
    pcl_cloud_ = std::make_shared<std::vector<TimedPoint>>();
  }

  void init()
  {
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    image_subscription_ = it_->subscribe(
      "/gst_image_pub", 10, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

    pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/mmwave_0/points", 10, std::bind(&ImageSubscriber::pcl_callback, this, std::placeholders::_1));

    publisher_ = it_->advertise("/overlay_radar_camera", 10);

    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) << 1288.56, 0, 1179.74, 0, 1293.73, 802.44, 0, 0, 1);
    intrinsics.distCoeffs = (cv::Mat_<double>(5, 1) << -0.24541651244031235, 0.05791238049933166, 0.0009060251209707499, -0.006485038511542, 0.0);
    intrinsics.fx = 1288.56;
    intrinsics.fy = 1293.73;
    intrinsics.cx = 1179.74;
    intrinsics.cy = 802.44;
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat undistorted_frame;
      cv::undistort(frame, undistorted_frame, intrinsics.cameraMatrix, intrinsics.distCoeffs);
      draw_pcl(undistorted_frame);
      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", undistorted_frame).toImageMsg();
      publisher_.publish(out_msg);
    } 
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    pcl::fromROSMsg(*msg, new_cloud);
    for (const auto& pt : new_cloud.points) {
      pcl_cloud_->emplace_back(TimedPoint{pt, 0});
    }
    // Increment ages and remove old points
    auto it = pcl_cloud_->begin();
    while (it != pcl_cloud_->end()) {
      it->age++;
      if (it->age > 30) { // Points fade out after 30 frames
        it = pcl_cloud_->erase(it);
      } else {
        ++it;
      }
    }
  }

  void draw_pcl(cv::Mat &frame)
  {
    for (const auto& timed_point : *pcl_cloud_) {
      float logical_x = static_cast<float>(timed_point.point.y);
      float logical_y = static_cast<float>(-timed_point.point.x);
      float logical_z = static_cast<float>(timed_point.point.z);

      if (logical_x <= 0) continue;

      int pixel_x = static_cast<int>( - (logical_x * intrinsics.fx / logical_z + intrinsics.cx));
      int pixel_y = static_cast<int>(logical_y * intrinsics.fy / logical_z + intrinsics.cy);

      if (pixel_x < 0 || pixel_x >= frame.cols || pixel_y < 0 || pixel_y >= frame.rows) continue;
      
      int alpha = 255 - (timed_point.age * 8); // Decrease alpha to make points fade out
      alpha = std::max(alpha, 0);
      cv::circle(frame, cv::Point(pixel_x, pixel_y), 10, cv::Scalar(0, 0, 255, alpha), -1);
    }
  }

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription_;
  image_transport::Publisher publisher_;
  std::shared_ptr<std::vector<TimedPoint>> pcl_cloud_;
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
