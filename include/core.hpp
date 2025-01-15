// SPDX-FileCopyrightText: 2024 YAZAWA Kenichi <s21c1036hn@gmail.com>
// SPDX-License-Identifier: MIT-LICENSE

#ifndef CORE_HPP_
#define CORE_HPP_

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "FileRW.hpp"
#include "TimeObserver.hpp"

namespace ReliefStoppers
{
class ReliefStopper : public rclcpp::Node
{
public:
    ReliefStopper();

private:
    void timer_cb();
    void bar_callback(const std_msgs::msg::Float32 &) const;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stopping_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bar_value_subscriber_;
    std::string stop_lifecycle_name_;
    std::string sub_topic_name_;
    float relief_threshold_;
    bool reverse_;

    std::string saving_raw_data_path_;
    std::string saving_stopping_data_path_;
    FileRW::FileRW raw_data_file_;
    FileRW::FileRW stopping_data_file_;
    TimeObserver::TimeObserver tm_observer_;
};
}

#endif

