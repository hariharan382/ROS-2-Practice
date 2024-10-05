#ifndef ROBOT_TF2_DETECTOR_OBSTACLEDETECTORNODE_HPP
#define ROBOT_TF2_DETECTOR_OBSTACLEDETECTORNODE_HPP

#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

//msgs
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rclcpp/rclcpp.hpp>

namespace robot_tf2_detector
{
    class ObstacleDetectorNode : public rclcpp::Node
    {
        public:
            ObstacleDetectorNode();

        private:
            void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    };
} //namespce robot_tf2_detector

#endif