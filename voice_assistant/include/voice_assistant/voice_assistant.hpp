#pragma once
#include <sys/stat.h>

#include <chrono>
#include <execution>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// extention node
#include "extension_node/extension_node.hpp"
// common_utils
#define USE_ROS2
#include "common_utils/common_utils.hpp"
// ROS2
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <voicevox_ros2_msgs/msg/talk.hpp>
// OpenMP
#include <omp.h>

using namespace std::chrono_literals;

class VoiceAssistant : public ext_rclcpp::ExtensionNode {
public:
  VoiceAssistant(const rclcpp::NodeOptions& options) : VoiceAssistant("", options) {}
  VoiceAssistant(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : ext_rclcpp::ExtensionNode("voice_assistant_node", name_space, options) {
    RCLCPP_INFO(this->get_logger(), "start voice_assistant_node");
    speaker_id_               = param<int>("voice_assistant.speaker_id", 0);
    INTERVAL                  = param<double>("voice_assistant.interval", 2.0);
    std::string LOG_LEVEL_STR = param<std::string>("voice_assistant.log_level", "DEBUG"); // DEBUG, INFO, WARN, ERROR, NONE
    logger.set_log_level(ext::get_log_level(LOG_LEVEL_STR));
    talk_pub_      = this->create_publisher<voicevox_ros2_msgs::msg::Talk>("/voicevox_ros2", rclcpp::QoS(10));
    listening_pub_ = this->create_publisher<std_msgs::msg::Bool>("whisper/listening", rclcpp::QoS(10));
    while (!talk_pub_->get_subscription_count()) {
      RCLCPP_INFO(this->get_logger(), "wait for subscriber");
      std::this_thread::sleep_for(1s);
    }
    talk_pub_->publish(make_talk("ボイスアシスタントを起動しました"));
    listening_pub_->publish(ros2_utils::make_bool(true));
    text_sub_ =
        this->create_subscription<std_msgs::msg::String>("chatgpt/output_text", rclcpp::QoS(10), [&](const std_msgs::msg::String::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "text: %s", msg->data.c_str());
          talk_pub_->publish(make_talk(msg->data));
          speaking_       = true;
          listening_time_ = this->get_now();
        });
    timer_          = this->create_wall_timer(20ms, [&]() {
      bool listening = false;
      if (!speaking_) listening = true;
      if (speaking_) {
        double elapsed = (this->get_now() - listening_time_).seconds();
        if (elapsed > INTERVAL) speaking_ = false;
      }
      listening_pub_->publish(ros2_utils::make_bool(listening));
    });
    listening_time_ = this->get_now();
  }

private:
  int speaker_id_;
  double INTERVAL = 1.0;
  bool speaking_  = false;
  // log
  ext::Logger logger;
  // subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
  // publisher
  rclcpp::Publisher<voicevox_ros2_msgs::msg::Talk>::SharedPtr talk_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr listening_pub_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time listening_time_;

  voicevox_ros2_msgs::msg::Talk make_talk(const std::string& text) {
    voicevox_ros2_msgs::msg::Talk talk;
    talk.text       = text;
    talk.speaker_id = speaker_id_;
    return talk;
  }
};