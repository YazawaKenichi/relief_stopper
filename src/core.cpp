//! SPDX-FileCopyrightText: 2024 YAZAWA Kenichi <s21c1036hn@gmail.com>
//! SPDX-License-Identifier: MIT-LICENSE

#include "core.hpp"
#include "FileRW.hpp"
#include "TimeObserver.hpp"

namespace ReliefStoppers
{
//! コンストラクタ
ReliefStopper::ReliefStopper()
    : Node("relief_stopper"),
    stop_lifecycle_name_("/relief_stopper/bar/stopping"),
    sub_topic_name_("/relief_detector/bar/raw"),
    relief_threshold_(1.0f),
    reverse_(false),
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
    this->declare_parameter("stopping_lifecycle_name", this->stop_lifecycle_name_);
    this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->declare_parameter("relief_threshold", this->relief_threshold_);
    //! 読み込み
    this->stop_lifecycle_name_ = this->get_parameter("stopping_lifecycle_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();
    this->relief_threshold_ = this->get_parameter("relief_threshold").as_double();

    ////////// 記録 //////////
    //! ファイル記録用の処理
    this->declare_parameter("saving_raw_data_directory", saving_raw_data_directory_);
    this->declare_parameter("saving_stopping_data_directory", saving_stopping_data_directory_);
    this->saving_raw_data_path_ = this->get_parameter("saving_raw_data_directory").as_string() + "/" + savingfilename;
    this->saving_stopping_data_path_ = this->get_parameter("saving_stopping_data_directory").as_string() + "/" + savingfilename;
    this->raw_data_file_ = FileRW::FileRW(this->saving_raw_data_path_);
    this->stopping_data_file_ = FileRW::FileRW(this->saving_stopping_data_path_);

    ////////// パブリッシャ //////////
    this->stopping_publisher_ = this->create_publisher<std_msgs::msg::Bool>(this->stop_lifecycle_name_, 10);

    ////////// サブスクライバ //////////
    this->bar_value_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            this->sub_topic_name_, 10, std::bind(
                &ReliefStopper::bar_callback, this,
                std::placeholders::_1));
}

//! サブスクライブ コールバック
void ReliefStopper::bar_callback(const std_msgs::msg::Float32 & raw) const
{
    //! サブスクライブ値の範囲
    // const int max = 256;
    // const int min = 0;

    //! 変換後の値
    std_msgs::msg::Bool stopping;
    stopping.data = (!this->reverse_) ? (raw.data > this->relief_threshold_ ? true : false) : (raw.data > this->relief_threshold_ ? false : true);

    RCLCPP_INFO(this->get_logger(), "    raw  : %8.3f", raw.data);
    RCLCPP_INFO(this->get_logger(), "stopping : %8d", stopping.data ? 1 : 0);

    this->raw_data_file_.writef(std::to_string(raw.data) + "\r\n");
    this->stopping_data_file_.writef(std::to_string(stopping.data) + "\r\n");

    this->stopping_publisher_->publish(stopping);
}
}

