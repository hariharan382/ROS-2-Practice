#include <string>
#include <iostream>

#include "robot_bt_bumgo/Back.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "gemoetry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_bt_bumpgo
{
    using namespace std::chrono_literals;

    Back::Back(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf) : BT::ActionNodeBase(xml_tag_name, conf)
        {
            config().blackboard->get("node", node_);

            vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("outputvel", 100);
        }

    void Back::halt()
    {

    }

    BT::NodeStatus Back::tick()
    {
        if (status() == BT::NodeStatus::IDLE){
            start_time_ = node->now();
        }

        geometry_msgs::msg::Twist vel_msgs;
        vel_msgs.linear.x = -0.3;
        vel_pub_->publish(vel_msgs);

        auto elapsed = node_->now() - start_time_

        if (elapsed < 3s){
            return BT::NodeStatus::RUNNING;
        }else {
            return BT::NodeStatus::SUCSESS;
        }
    } 


} //namespace robot_bt_bumpgo

#include "behaviortree_cpp_v3/robot_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<robot_bt_bumpgo::Back>("Back");
}