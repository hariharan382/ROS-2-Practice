#include <memory>

#include "robot_tf2_detector/ObstacleDetectorNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
namespace robot_tf2_detector
{
    ObstacleDetectorNode::ObstacleDetectorNode() : Node("Obstacle_detector")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "input_scan",rclcpp::SensorDataQoS(),
            std::bind(&ObstacleDetectorNode::scan_callback, this, _1)
        );


        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    void  ObstacleDetectorNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
    {
        double dist = msg->ranges[msg->ranges.size()/2];

        if (!std::isinf(dist)){
            geometry_msgs::msg::TransformStamped detection_tf;

            detection_tf.header = msg->header;
            detection_tf.child_frame_id = "detected_obstacle";
            detection_tf.transform.translation.x = msg->ranges[msg->ranges.size()/2];

            tf_broadcaster_->sendTransform(detection_tf);

        }
    }
}


