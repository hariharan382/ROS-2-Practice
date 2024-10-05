#ifndef ROBOT_BT_BUMPGO__BACK_HPP_
#define ROBOT_BT_BUMPGO__BACK_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcp/rclcpp.hpp"

namespace robot_bt_bumpgo
{
    class Back : public BT::ActionNodeBase
    {
        public:
            explicit Back(
                const std::string & xml_tag_name,
                const BT::NodeConfiguration & conf);

            void halt();
            BT::NodeStatus tick();

            static BT::PortList providedPots()
            {
                return BT::PortsList({});
            }

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
            rclcpp::Time start_time;
    };   
}// namespace robot_bt_bumpgo

#endif // ROBOT_BT_BUMPGO__BACK_HPP_