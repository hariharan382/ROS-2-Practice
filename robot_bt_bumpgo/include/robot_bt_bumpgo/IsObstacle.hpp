#ifndef ROBOT_BT_BUMPGO__ISOBSTACLE_HPP_
#define ROBOT_BT_BUMPGO__ISOBSTACLE_HPP_

#include <string>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace robot_bt_bumpgo
{
    class isObsacle : public BT::conditionNode
    {
        public:
            explicit IsObstacle(
                const std::string & xml_tag_name,
                const BT::NodeConfiguration & conf);
            
            BT::NodeStatus tick();

            static BT::PortsList providedPorts()
            {
                return BT::PortsList(
                    {BT::InputPort<double>("distance")}
                );
            }

            void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
        
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Time last_reading_time_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
            sensor_msgs::msg::LaserScn::UniquePtr last_scan_;
    };
} //namespace robot_bt_bumpgo

#endif //ROBOT_BT_BUMPGO__ISOBSTACLE_HPP_