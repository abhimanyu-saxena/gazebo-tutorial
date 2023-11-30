/**
 * @file roomba.cpp
 * @author Abhimanyu Saxena (asaxena4@umd.edu)
 * @brief Roomba functionality for turtlebot3
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Class declaration for roomba implemenation in gazebo
 * 
 */
class RoombaAlgo : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new roomba object
   * 
   */
  RoombaAlgo() : Node("roomba") {
    laser_data_subscriber =
      this->create_subscription<sensor_msgs::msg::LaserScan>
        ("scan",
        10,
        std::bind(&RoombaAlgo::laser_scan_cb, this, std::placeholders::_1));

    // create a new velocity publisher
    velcoity_publisher =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

 private:
  /**
   * @brief cbk function for laser scan 
   * 
   * @param laser_data 
   */
  void laser_scan_cb(const sensor_msgs::msg::LaserScan& laser_data) {
    if (laser_data.header.stamp.sec == 0) {
      return;
    }

    auto laser_scan = laser_data.ranges;
    for (unsigned int scan_angle = 330; scan_angle < 330 + 60; scan_angle++) {
      if (laser_scan[scan_angle % 360] < 0.9) {
        move_roomba(0.0, 0.1);
      } else {
        move_roomba(0.1, 0.0);
      }
    }
  }

  /**
   * @brief Method to move roomba robot
   * 
   * @param x velocity in x 
   * @param z velocity in z
   */
  void move_roomba(float x, float z) {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = x;
    twist.angular.z = -z;
    velcoity_publisher->publish(twist);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_data_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velcoity_publisher;
  rclcpp::TimerBase::SharedPtr timer;
};

/**
 * @brief Main function to execute roomba functionality
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoombaAlgo>());
  rclcpp::shutdown();
  return 0;
}