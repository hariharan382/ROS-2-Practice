#ifndef ROBOT_TRACKING__HEADCONTROLLER_HPP_
#define ROBOT_TRACKING__HEADCONTROLLER_HPP_

#include <memory>

#include "robot_tracking_msgs/msg/pan_tilt_command.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_tracking/PIDController.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_tracking
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HeadController : public rclcpp_lifecycle::LifecycleNode
    {
        public:
            HeadController();

            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

            void control_cycle();

            void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);
    
            void command_callback(robot_tracking_msgs::msg::PanTiltCommand::UniquePtr msg);

        private:
            rclcpp::Subscription<robot_tracking_msgs::msg::PanTiltCommand>::SharedPtr command_sub_;
            rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_sub_;
            rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state_;
            robot_tracking_msgs::msg::PanTiltCommand::UniquePtr last_command_;
            rclcpp::Time last_command_ts_;

            PIDController pan_pid_, tilt_pid_;


    }; 
}//namespace robot_tracking

#endif //ROBOT_TRACKING__HEADCONTROLLER_HPP_