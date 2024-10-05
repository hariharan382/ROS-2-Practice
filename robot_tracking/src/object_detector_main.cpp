#include <memory>

#include "robot_tracking/ObjectDetector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int args, char * argv[])
{
    rclcpp::init(args , argv);

    auto node_detector = std::make_shared<robot_tracking::ObjectDetector>();

    rclcpp::spin(node_detector);
    rclcpp::shutdown();
    return 0;
}