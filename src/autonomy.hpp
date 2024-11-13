/** Copyright 2024 William L. Thomson Jr. <w@wltjr.com>
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef AUTONOMOUS__AUTONOMY_HPP_
#define AUTONOMOUS__AUTONOMY_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "std_msgs/msg/string.hpp"

namespace autonomous
{
    /**
     * @brief Class for autonomous robot control
     */
    class Autonomy : public rclcpp::Node
    {
        public:
            /**
             * @brief Construct a new Autonomy object instance
             * 
             * @param options node options passed at startup
             */
            Autonomy(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief Destroy the Autonomy object instance
             */
            virtual ~Autonomy();

        private:
            // publisher - /goal_pose topic we publish commands to
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_;

            // subscription - the /autonomy topic we listen to
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_autonomy_;

            /**
             * @brief Callback function for subscription fired when messages
             *        on the /autonomy topic are heard
             * 
             * @param msg the message data that was heard
             */
            void autonomyCallback(const std_msgs::msg::String::SharedPtr msg);
    };
}

#endif
