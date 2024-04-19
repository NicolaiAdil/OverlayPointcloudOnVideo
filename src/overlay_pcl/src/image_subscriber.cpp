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
  cv::Mat cameraMatrix; // Camera matrix (K)
  cv::Mat distCoeffs;   // Distortion coefficients (D)
  float fx, fy;         // Focal lengths in x and y
  float cx, cy;         // Optical center coordinates
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

      // Undistort the image
      cv::Mat undistorted_frame;
      cv::undistort(frame, undistorted_frame, intrinsics.cameraMatrix, intrinsics.distCoeffs);

      // Now use the undistorted frame
      draw_pcl(undistorted_frame);

      // Transform the cv::Mat back to ros msg and publish it
      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", undistorted_frame).toImageMsg();
      publisher_.publish(out_msg);
      
      // Display the modified image for debugging
      // cv::imshow("Modified Image", undistorted_frame);
      // cv::waitKey(10);
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
    // RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
    pcl::fromROSMsg(*msg, *pcl_cloud_);
  }

  void draw_pcl(cv::Mat &frame)
{
    // Create a deep copy of pcl_cloud_ to avoid race conditions
    auto pcl_cloud_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*pcl_cloud_);

    if (pcl_cloud_copy->points.empty()) {
        return;
    }

    for (const auto& pt : pcl_cloud_copy->points) {
        // y is straight ahead, z is up, and x is to the right. Convert to make sense.
        float logical_x = static_cast<float>(pt.x);
        float logical_y = -static_cast<float>(pt.z);
        float logical_z = static_cast<float>(pt.y);

        if (logical_x <= 0) continue;  // Ensure the point is in front of the camera

        // Formula from: https://towardsdatascience.com/what-are-intrinsic-and-extrinsic-camera-parameters-in-computer-vision-7071b72fb8ec
        int pixel_x = static_cast<int>(logical_x * intrinsics.fx / logical_z + intrinsics.cx);
        int pixel_y = static_cast<int>(logical_y * intrinsics.fy / logical_z + intrinsics.cy);

        if (pixel_x < 0 || pixel_x >= frame.cols || pixel_y < 0 || pixel_y >= frame.rows) {
            RCLCPP_INFO(this->get_logger(), "Calculated pixel point is outside the image. (%d, %d)", pixel_x, pixel_y);
            continue;
        }

        float depth = sqrt(pow(logical_x, 2) + pow(logical_y, 2) + pow(logical_z, 2));
        RCLCPP_INFO(this->get_logger(), "Publishing point with depth: %f, (%d, %d)", depth, pixel_x, pixel_y);
        cv::Scalar color = depth_to_color(depth);
        
        // Draw the point on the image
        cv::circle(frame, cv::Point(pixel_x, pixel_y), 10, color, -1);
    }
}


  cv::Scalar depth_to_color(float depth)
  {
    // Normalize the depth to a 0.0 to 1.0 scale based on expected depth range
    // Adjust max_depth as necessary to fit the expected range of your data
    float max_depth = 10.0;  // maximum depth to normalize the color scale
    float normalized_depth = std::min(depth / max_depth, 1.0f);

    // Use a color gradient from blue (close) to red (far)
    int red = static_cast<int>(255 * normalized_depth);        // Increases as depth increases
    int blue = static_cast<int>(255 * (1 - normalized_depth)); // Decreases as depth increases
    int green = 0;  // Green channel is not used for simplicity, but you can adjust it for different colors

    return cv::Scalar(blue, green, red);
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