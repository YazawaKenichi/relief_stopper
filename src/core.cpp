//! SPDX-FileCopyrightText: 2024 YAZAWA Kenichi <s21c1036hn@gmail.com>
//! SPDX-License-Identifier: MIT-LICENSE

#include "core.hpp"
#include "FileRW.hpp"
#include "TimeObserver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace ReliefStoppers
{
//! コンストラクタ
ReliefStopper::ReliefStopper()
    : Node("relief_stopper"),
    processed_cmd_vel_name_("/cmd_vel"),
    raw_cmd_vel_name_("/cmd_vel_raw"),
    bar_angle_name_("/bar/angle"),
    relief_threshold_(1.0f),
    reverse_(false),
    stopping_(false),
    saving_raw_data_path_("/dev/null"),
    saving_stopping_data_path_("/dev/null"),
    raw_data_file_(this->saving_raw_data_path_),
    stopping_data_file_(this->saving_stopping_data_path_)
{
    ////////// 記録 //////////
    this->tm_observer_ = TimeObserver::TimeObserver();
    std::string savingfilename = tm_observer_.now();
    std::string saving_raw_data_directory_;
    std::string saving_stopping_data_directory_;

    ////////// パラメータ //////////
    //! 定義
    this->declare_parameter("processed_cmd_vel_topic_name", this->processed_cmd_vel_name_);
    this->declare_parameter("raw_cmd_vel_topic_name", this->raw_cmd_vel_name_);
    this->declare_parameter("bar_angle_topic_name", this->bar_angle_name_);
    this->declare_parameter("relief_threshold", this->relief_threshold_);
    this->declare_parameter("reverse", this->reverse_);
    //! 読み込み
    this->processed_cmd_vel_name_ = this->get_parameter("processed_cmd_vel_topic_name").as_string();
    this->raw_cmd_vel_name_ = this->get_parameter("raw_cmd_vel_topic_name").as_string();
    this->bar_angle_name_ = this->get_parameter("bar_angle_topic_name").as_string();
    this->relief_threshold_ = this->get_parameter("relief_threshold").as_double();
    this->reverse_ = this->get_parameter("reverse").as_double();

    ////////// 記録 //////////
    //! ファイル記録用の処理
    this->declare_parameter("saving_raw_data_directory", saving_raw_data_directory_);
    this->declare_parameter("saving_stopping_data_directory", saving_stopping_data_directory_);
    this->saving_raw_data_path_ = this->get_parameter("saving_raw_data_directory").as_string() + "/" + savingfilename;
    this->saving_stopping_data_path_ = this->get_parameter("saving_stopping_data_directory").as_string() + "/" + savingfilename;
    this->raw_data_file_ = FileRW::FileRW(this->saving_raw_data_path_);
    this->stopping_data_file_ = FileRW::FileRW(this->saving_stopping_data_path_);

    ////////// 初期値の代入 //////////
    this->cmd_vel_zeros_.linear.x = 0.0f;
    this->cmd_vel_zeros_.linear.y = 0.0f;
    this->cmd_vel_zeros_.linear.z = 0.0f;
    this->cmd_vel_zeros_.angular.x = 0.0f;
    this->cmd_vel_zeros_.angular.y = 0.0f;
    this->cmd_vel_zeros_.angular.z = 0.0f;

    ////////// パブリッシャ //////////
    this->processed_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(this->processed_cmd_vel_name_, 10);

    ////////// サブスクライバ //////////
    this->raw_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            this->raw_cmd_vel_name_, 10, std::bind(
                &ReliefStopper::raw_cmd_vel_callback_, this,
                std::placeholders::_1));
    this->bar_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            this->bar_angle_name_, 10, std::bind(
                &ReliefStopper::bar_angle_callback_, this,
                std::placeholders::_1));
}

void ReliefStopper::raw_cmd_vel_callback_(const geometry_msgs::msg::Twist & raw_cmd_vel)
{
    if(this->stopping_)
    {
        this->processed_cmd_vel_publisher_->publish(this->cmd_vel_zeros_);
    }
    else
    {
        this->processed_cmd_vel_publisher_->publish(raw_cmd_vel);
    }
}

//! サブスクライブ コールバック
void ReliefStopper::bar_angle_callback_(const std_msgs::msg::Float32 & angle)
{
    this->stopping_ = (!this->reverse_) ? (angle.data > this->relief_threshold_ ? true : false) : (angle.data > this->relief_threshold_ ? false : true);

    RCLCPP_INFO(this->get_logger(), "    raw  : %8.3f", angle.data);
    RCLCPP_INFO(this->get_logger(), "stopping : %8d", this->stopping_ ? 1 : 0);
}
}

