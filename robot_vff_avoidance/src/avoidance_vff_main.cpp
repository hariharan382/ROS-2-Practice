#include <memory>

#include "robot_vff_avoidance/AvoidanceNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc ,char * argv[])
{
    rclcpp::init(argc, argv);

    auto avoidance_node = std::make_shared<robot_vff_avoidance::AvoidanceNode>();
    rclcpp::spin(avoidance_node);

    rclcpp::shutdown();

    return 0;

}