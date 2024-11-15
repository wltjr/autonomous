/** Copyright 2024 William L. Thomson Jr. <w@wltjr.com>
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include "autonomy.hpp"

using std::placeholders::_1;

namespace autonomous
{
    Autonomy::Autonomy(const rclcpp::NodeOptions &options)
        : Node("autonomy", options)
    {
        // load goal_poses from yaml config
        if (!this->has_parameter("goal_poses")) {
            this->declare_parameter("goal_poses", std::vector<double>(3));
        }
        goal_poses = this->get_parameter("goal_poses").as_double_array();

        // load frame_id from yaml config
        if (!this->has_parameter("frame_id")) {
            this->declare_parameter("frame_id", "map");
        }
        frame_id = this->get_parameter("frame_id").as_string();

        // publish to /goal_pose topic
        publisher_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

        // subscribe to /autonomy topic
        subscription_autonomy_ =
            this->create_subscription<std_msgs::msg::String>("autonomy",
                rclcpp::QoS(10), std::bind(&Autonomy::autonomyCallback, this, _1));
    }

    Autonomy::~Autonomy() = default;

    void Autonomy::autonomyCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "go")
        {
            // Initializes with zeros by default.
            auto goal_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

            goal_msg->header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
            goal_msg->header.frame_id = frame_id;
            goal_msg->pose.position.x = goal_poses[0];
            goal_msg->pose.position.y = goal_poses[1];
            goal_msg->pose.position.z = goal_poses[2];
            goal_msg->pose.orientation.w = 1.0;

            // publish goal
            publisher_goal_->publish(std::move(goal_msg));
        }

    }
}
