#include "memory"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "robot_tf2_detector/ObstacleDetectorImprovedNode.hpp"

#include "sensor_msgs/msg/laser_scan.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_tf2_detector
{
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    ObstacleDetectorImprovedNode::ObstacleDetectorImprovedNode() : Node("obstacle_detector_improved"), 
    tf_buffer_(),
    tf_listener_(tf_buffer_)
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("input_scan",
        rclcpp::SensorDataQoS(),
        std::bind(&ObstacleDetectorImprovedNode::scan_callback, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    void ObstacleDetectorImprovedNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
    {
        double dist = msg->ranges[msg->ranges.size()/2];

        if (!std::isinf(dist)) {
            tf2::Transform laser2object;
            laser2object.setOrigin(tf2::Vector3(dist, 0.0, 0.0));
            laser2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

            geometry_msgs::msg::TransformStamped odom2laser_msg;
            tf2::Stamped<tf2::Transform> odom2laser;

            try {
                odom2laser_msg = tf_buffer_.lookupTransform(
                    "odom","base_laser_link",
                    tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds() - 0.3)
                );
            } catch (tf2::TransformException & ex) {
                RCLCPP_WARN(get_logger(), "Obstacle transofrm was not found: %s",ex.what());
                return;
            }

            tf2::Transform odom2object = odom2laser * laser2object;

            geometry_msgs::msg::TransformStamped odom2object_msg;
            odom2object_msg.transform = tf2::toMsg(odom2object);

            odom2object_msg.header.stamp = msg->header.stamp;
            odom2object_msg.header.frame_id = "odom";
            odom2object_msg.child_frame_id = "detected_obstacle";

            tf_broadcaster_->sendTransform(odom2object_msg);

        }
    }


}
