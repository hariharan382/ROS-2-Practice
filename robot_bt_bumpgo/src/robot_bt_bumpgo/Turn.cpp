#include "string"
#include "iostream"

#include "robot_bt_bumpgo/Turn.hpp"
#include "behaviortree_cpp_v3/brhavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_bt_bumpgo
{
    using namespace std::chronoliterals;

    Turn::Turn(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration(xml_tag_name, conf)
    ) : BT::ActionNodeBase(xml_tag_name, conf)
    {
        config().blackboard->get("node", node_);

        vel_pub_ = node_->create_publisher<geometry_msgs::msg::twist>("/output_vel", 100);

    }

    void Turn::halt()
    {

    }

    BT::NodeStatus Turn::tick()
    {
        if (status() == BT::NOdeStatus::IDLE){
            start_time_ = node_->now()
        }

        geometry_msgs::msg::Twist vel_msgs;
        vel_msgs.angular.z = 0.5;
        vel_pub_->publish(vel_msgs);

        auto elapsed = node_->now() - start_time_;

        if (elapsed < 3s){
            return BT::NodeSttus::RUNNING;
        } else {
            return BT::NodeStatus::SUCCESS;
        }

    }
} //namespace robot_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<robot_bt_bumpgo::Turn>("Turn");
}
