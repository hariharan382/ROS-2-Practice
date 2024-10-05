#include <string>

#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "robot_bt_bumpgo/Forward.hpp"

#include "gemetry_msgs/msg/Twist"
#include "rclcpp/rclcpp.hpp"

namespace robot_bt_bumpgo
{
        using namespace std::chrono_literals;

        Forward::Forward(
            const std::string & xml_tag_name,
            const BT::NodeConfiguration & conf) : BT::ActionNodeBase(xl_tag_name, conf)
        {
            config().blackboard->get("node", node_);

            vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);
        }

        BT::NodeStatus Forward::tick()
        {
            geometry_msgs::msgTwist vel_msgs;
            vel_msgs.linear.x = 0.3;
            vel_pub_->publish(vel_msgs);

            return BT::NodeStatus::RUNNING;
        }

} //namespace robot_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<robot_bt_bumpgo::Foward>("Forward");
}
