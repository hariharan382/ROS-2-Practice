#include <memory>

#include "robot_fsm_bumpgo_cpp/BumpGoNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc , char * argv[])
{
    rclcpp::init(argc, argv);

    auto bumpgo_node = std::make_shared<robot_fsm_bumpgo_cpp::BumpGoNode>();
    rclcpp::spin(bumpgo_node);

    rclcpp::shutdown();

    return 0;

}

