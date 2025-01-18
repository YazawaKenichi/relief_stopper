// SPDX-FileCopyrightText: 2024 YAZAWA Kenichi <s21c1036hn@gmail.com>
// SPDX-License-Identifier: MIT-LICENSE

#ifndef CORE_HPP_
#define CORE_HPP_

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "FileRW.hpp"
#include "TimeObserver.hpp"

namespace ReliefStoppers
{
class ReliefStopper : public rclcpp::Node
{
public:
    ReliefStopper();

private:
    void raw_cmd_vel_callback_(const geometry_msgs::msg::Twist & raw_cmd_vel);
    void bar_angle_callback_(const std_msgs::msg::Float32 & angle);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr processed_cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bar_angle_subscriber_;
    std::string processed_cmd_vel_name_;
    std::string raw_cmd_vel_name_;
    std::string bar_angle_name_;
    geometry_msgs::msg::Twist cmd_vel_zeros_;

    float relief_threshold_;
    bool reverse_;
    bool stopping_;

    std::string saving_raw_data_path_;
    std::string saving_stopping_data_path_;
    FileRW::FileRW raw_data_file_;
    FileRW::FileRW stopping_data_file_;
    TimeObserver::TimeObserver tm_observer_;
};
}

#endif

