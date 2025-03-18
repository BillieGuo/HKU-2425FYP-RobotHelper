#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarToLaserScan : public rclcpp::Node
{
public:
  LidarToLaserScan() : Node("lidar_to_laserscan")
  {
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", 10, std::bind(&LidarToLaserScan::lidar_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  }

private:
  float custom_angle_min = -M_PI / 2 - M_PI / 4;
  float custom_angle_max = M_PI / 2 + M_PI / 4;

  float box_y_max = 0.3;// 2.28 edit + cut box at the back
  float box_y_min = -0.3;
  float box_x_min = 0.05;
  void lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received lidar message");

    std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
    std::vector<float> intensities(360, 0.0);
    std::vector<float> angle_range(360);
    for (int i = 0; i < 360; ++i)
    {
      angle_range[i] = -M_PI + i * (2 * M_PI / 360);
    }

    for (const auto & point : msg->points)
    {
      float x = point.x;
      float y = point.y;
      float z = point.z;
      // 2.28 edit + cut box at the back (y <= box_y_min && y >= box_y_max) || (x >= box_x_min)
      if ((x != 0 && y != 0 && std::abs(z) < 0.3) && (y <= box_y_min || y >= box_y_max || x > box_x_min))
      {
        float range = std::sqrt(x * x + y * y);
        float intensity = static_cast<float>(point.reflectivity);
        float angle = std::atan2(y, x);
        // full sphere
        // auto closest_angle_it = std::min_element(angle_range.begin(), angle_range.end(),
        //                                          [angle](float a, float b) { return std::abs(a - angle) < std::abs(b - angle); });
        // int closest_angle_index = std::distance(angle_range.begin(), closest_angle_it);
        // if (range < ranges[closest_angle_index]) 
        // {
        //   ranges[closest_angle_index] = range;
        //   intensities[closest_angle_index] = intensity;
        // }

        // Only consider front half sphere
        if (angle >= custom_angle_min && angle <= custom_angle_max) 
        {
          auto closest_angle_it = std::min_element(angle_range.begin(), angle_range.end(),
                                                   [angle](float a, float b) { return std::abs(a - angle) < std::abs(b - angle); });
          int closest_angle_index = std::distance(angle_range.begin(), closest_angle_it);
          if (range < ranges[closest_angle_index])
          {
            ranges[closest_angle_index] = range;
            intensities[closest_angle_index] = intensity;
          }
        }
      }
    }

    auto laser_scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    // laser_scan_msg->header.stamp = this->get_clock()->now();
    laser_scan_msg->header.stamp = msg->header.stamp;
    laser_scan_msg->header.frame_id = "laser";
    laser_scan_msg->angle_min = -M_PI;
    laser_scan_msg->angle_max = M_PI;
    laser_scan_msg->angle_increment = (laser_scan_msg->angle_max - laser_scan_msg->angle_min) / ranges.size();
    laser_scan_msg->time_increment = 0.0;
    laser_scan_msg->scan_time = 0.1;
    laser_scan_msg->range_min = 0.0;
    laser_scan_msg->range_max = 100.0;
    laser_scan_msg->ranges = ranges;
    laser_scan_msg->intensities = intensities;

    publisher_->publish(*laser_scan_msg);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarToLaserScan>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}