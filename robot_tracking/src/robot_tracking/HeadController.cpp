#include <algorithm>
#include <utility>

#include "robot_tracking/HeadController.hpp"
#include "robot_tracking/PIDController.hpp"

#include "robot_tracking_msgs/msg/pan_tilt_command.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_tracking
{
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    HeadController::HeadController() : LifecycleNode("head_tracker"),
    pan_pid_(0.0, 1.0, 0.0, 0.3),
    tilt_pid_(0.0, 1.0, 0.0, 0.3)
    {
        command_sub_ = create_subscription<robot_tracking_msgs::msg::PanTiltCommand>(
            "command", 10,
            std::bind(&HeadController::command_callback, this, _1));

        joint_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
                "joint_state", rclcpp::SensorDataQoS(),
                std::bind(&HeadController::joint_state_callback, this, _1));
            
        joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_command", 100);
    }

    CallbackReturn HeadController::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_logger(), "HeadController configured");
        pan_pid_.set_pid(0.4, 0.05, 0.55);
        tilt_pid_.set_pid(0.4, 0.05, 0.55);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeadController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_logger(), "HeadController activated");

        joint_pub_->on_activate();
        timer_ = create_wall_timer(100ms, std::bind(&HeadController::control_cycle, this));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeadController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_logger(), "HeadController deactivated");

        trajectory_msgs::msg::JointTrajectory command_msg;
        command_msg.header.stamp = now();
        command_msg.joint_names = last_state_->joint_names;
        command_msg.points.resize(1);
        // set values for point-1 (.points[0])
        command_msg.points[0].positions.resize(2);
        command_msg.points[0].velocities.resize(2);
        command_msg.points[0].accelerations.resize(2);
        command_msg.points[0].positions[0] = 0.0; // set X=0
        command_msg.points[0].positions[1] = 0.0; // set Y=0
        command_msg.points[0].velocities[0] = 0.1; //set X_dot = 0.1
        command_msg.points[0].velocities[1] = 0.1; //set Y_dot = 0.1
        command_msg.points[0].accelerations[0] = 0.1; //set X_ddot = 0.1
        command_msg.points[0].accelerations[1] = 0.1; //set Y_ddot = 0.1
        command_msg.points[0].time_from_start = rclcpp::Duration(1s);

        joint_pub_->publish(command_msg);

        joint_pub_->on_deactivate();
        timer_ = nullptr;

        return CallbackReturn::SUCCESS;

    }

    void HeadController::joint_state_callback(
        control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg
    )
    {
        last_state_ = std::move(msg);
    }

    void HeadController::command_callback(robot_tracking_msgs::msg::PanTiltCommand::UniquePtr msg)
    {
        last_command_ = std::move(msg);
        last_command_ts_ = now();
    }

    void HeadController::control_cycle()
    {
        if (last_state_ == NULL) {return;}

        trajectory_msgs::msg::JointTrajectory command_msg;
        command_msg.header.stamp = now();
        command_msg.joint_names = last_state_->joint_names;
        command_msg.points.resize(1);
        command_msg.points[0].positions.resize(2);
        command_msg.points[0].velocities.resize(2);
        command_msg.points[0].accelerations.resize(2);
        command_msg.points[0].time_from_start = rclcpp::Duration(200ms);

        if ((last_command_ == NULL) || ((now() - last_command_ts_) > 100ms))
        {
            //setting the values for point-1
            command_msg.points[0].positions[0] = 0.0 ; //set x = 0.0
            command_msg.points[0].positions[1] = 0.0 ; //set y = 0.0
            command_msg.points[0].velocities[0] = 0.1 ; //set x_dot =0.1
            command_msg.points[0].velocities[1] = 0.1 ; //set y_dot = 0.1
            command_msg.points[0].accelerations[0] = 0.1; // x_ddot = 0.1
            command_msg.points[0].accelerations[1] = 0.1; //set y_ddot = 0.1
            command_msg.points[0].time_from_start = rclcpp::Duration(1s);
        } else {
            double control_pan = pan_pid_.get_output(last_command_->pan);
            double control_tilt = tilt_pid_.get_output(last_command_ ->tilt);

            command_msg.points[0].positions[0] = last_state_->actual.positions[0] - control_pan;
            command_msg.points[0].positions[1] = last_state_->actual.positions[1] - control_tilt;

            command_msg.points[0].velocities[0] = 0.5;
            command_msg.points[0].velocities[1] = 0.5;
            command_msg.points[0].accelerations[0] = 0.5;
            command_msg.points[0].accelerations[1] = 0.5;
        }

        joint_pub_->publish(command_msg);

    }
} // namesace_robot_tracking