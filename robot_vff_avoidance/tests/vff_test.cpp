#include <limits>
#include <vector>
#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_vff_avoidance/AvoidanceNode.hpp"

#include <gtest/gtest.h>

using namespace std::chrono_literals;

class AvoidanceNodeTest : public robot_vff_avoidance::AvoidanceNode
{
    public:
        robot_vff_avoidance::VFFVectors get_vff_test(const sensor_msgs::msg::LaserScan & scan)
        {
            return get_vff(scan);
        }

        visualization_msgs::msg::MarkerArray get_debug_vff_test(const robot_vff_avoidance::VFVectors & vff_vectors)
        {
            return get_debug_vff(vff_vectors)
        }
};

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
    // checkng the range array to infinity
    sensor_msgs::msg::LaserScan ret;
    ret.header.stamp = ts;
    ret.angle_min = -M_PI;
    ret.angle_max = M_PI;
    ret.angle_increment = 2.0*M_PI/16.0;
    //initializing the vector with infinity of 16 size
    ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

    return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
    // checking the range array when it was set to 0
    sensor_msgs::msg::LaserScan ret;
    ret.header.stamp = ts;
    ret.angle_min = -M_PI;
    ret.angle_max = M_PI;
    ret.angle_increment = 2.0*M_PI/16.0;
    ret.ranges = std::vector<float>(16, 0.0);

    return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
    //checking the range array , when the lowest values 0.3 is set to index 2
     // 360 is diveded by 16 and testing for 1st th quadrant
    sensor_msgs::msg::LaserScan ret;
    ret.header.stamp = ts;
    ret.angle_min = -M_PI;
    ret.angle_max = M_PI;
    ret.angle_increment = 2.0 * M_PI/16.0;
    ret.ranges = std::vector<float>(16, 5.0);
    //initializing the array and setting the lowest value to 0.3
    ret.ranges[2] = 0.3;

    return ret;
}

sensor_mgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
    //checking the range array , when the lowest values 0.3 is set to index 6
     // 360 is diveded by 16 and testing for 2 nd quadrant
    sensor_msgs::msg::LaserScan ret;
    ret.header.stamp = ts;
    ret.angle_min = -M_PI;
    ret.angle_max = M_PI;
    ret.angle_increment = 2.0*M_PI/16.0;
    ret.ranges = std::vector<float>(16, 5.0);
    ret.ranges[6] = 0.3;

    return ret
}

sensor_msgs:msg::LaserScan get_scan_test_5(rclcpp::Time ts)
{
    //checking the range array , when the lowest values 0.3 is set to index 10
    // 360 is diveded by 16 and testing for 3 rd quadrant
    sensor_msgs::msg::LaserScan ret;
    ret.header.stamp = ts;
    ret.angle_min = -M_PI;
    ret.angle_max = M_PI;
    ret.angle_increment = 2.0 * M_PI/16.0;
    ret.ranges = std::vector<float>(16, 5.0);
    ret.ranges[10] = 0.3;

    return ret;
}


sensor_msgs::msg::LaserScan get_scan_test_6(rclcpp::Time ts)
{
    //checking the range array , when the lowest values 0.3 is set to index 10
    // 360 is diveded by 16 and testing for 3 rd quadrant
    //Same as case-5
    sensor_msgs::msg::LaserScan ret;
    ret.header.stamp = ts;
    ret.angle_min = -M_PI;
    ret.angle_max = M_PI;
    ret.angle_increment = 2.0*M_PI/16.0;
    //Initializaing the vector with 5.0 for lenght of 16
    ret.ranges = std::vector<float>(16, 5.0);
    ret.ranges[10] = 0.3

    return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_7(rclcpp::Time ts)
{
  //checking the range array , when the lowest values 0.3 is set to index 14
  // 360 is diveded by 16 and testing for 4 th quadrant
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[14] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_8(rclcpp::Time ts)
{
  //checking the range array , when the lowest values 0.01 is set to index 8
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[8] = 0.01;

  return ret;
}

TEST(vff_tests, get_vff)
{
    auto node_avoidance = AvoidanceNodeTest();

    rclcpp::Time ts = node_avoidance.now();

    auto res1 = node_avoidance.get_vff_test(get_scan_test1(ts));
    ASSERT_EQ(res1.attarctive, std::vector<float>({1.0f, 0.0f}));
    ASSERT_EQ(res1.repulsive, std::vector<float>({0.0f, 0.0f}));
    ASSERT_EQ(res1.result, std::vector<float>({1.0f, 0.0f}));

    auto res2 = node_avoidance.get_vff_test(get_scan_test_2(ts));
    ASSERT_EQ(res2.attractive, std::vector<float>({.0f, 0.0f}));
    ASSERT_NEAR(res2.repulsive[0], 1.0f, 0.00001f);
    ASSERT_NEAR(res2.repulsive[1], 0.0f, 0.00001f);
    ASSERT_NEAR(res2.result[0], 2.0f, 0.00001f);
    ASSERT_NEAR(res2.result[1], 0.0f, 0.00001f);

    //checking for quadrant Q-1
    auto res3 = node_avoidance.get_vff_test(get_scan_test_3(ts));
    ASSERT_EQ(res3.attractive, std::vector<float>({1.0f, 0.0f}));
    ASSERT_GT(res3.repulsive[0], 0.0f);
    ASSERT_GT(res3.repulsive[1], 0.0f);
    ASSERT_GT(atan2(res3.result[1], res3.result[0]), 0.1);
    ASSERT_LT(atan2(res3.result[1], res3.result[0]), M_PI_2);
    ASSERT_GT(atan2(res3.result[1], res3.result[0]), 0.1);
    ASSERT_LT(atan2(res3.result[1], res3.result[0]), M_PI_2);

    //checking for quadrant Q-2
    auto res4 = node_avoidance.get_vff_test(get_scan_test_4(ts));
    ASSERT_EQ(res4.attractive, std::vector<float>({1.0f, 0.0f}));
    ASSERT_LT(res4.repulsive[0], 0.0f);
    ASSERT_GT(res4.repulsive[1], 0.0f);
    ASSERT_GT(atan2(res4.repulsive[1], res4.repulsive[0]), M_PI_2);
    ASSERT_LT(atan2(res4.repulsive[1], res4.repulsove[0]), M_PI);
    ASSERT_GT(atan2(res4.result[1], res4.result[0]),0.0);
    ASSERT_LT(atan2(res4.result[1], res4.result[0]), M_PI_2)

    //checking for quadrant Q-3
    auto res5 = node_avoidance.get_vff_test(get_scan_test_5(ts));
    ASSERT_EQ(res5.attractive, std::vector<float>({1.0f, 0.0f}));
    ASSERT_LT(res5.repulsive[0], 0.0f);
    ASSERT_LT(res5.repulsive[1], 0.0f);
    ASSERT_GT(atan2(res5.repulsive[1], res5.repulsive[0]), -M_PI);
    ASSERT_LT(atan2(res5.repulsive[1], res5.repulsive[0]), -M_PI_2);
    ASSERT_LT(atan2(res5.result[1], res5.result[0]), 0.0);
    ASSERT_GT(atan2(res5.result[1], res5.result[0]), -M_PI_2);

    //checking for quadrant Q-4
    auto res6 = node_avoidance.get_vff_test(get_scan_test_6(ts));
    ASSERT_EQ(res6.attractive, std::vector<float>({1.0f, 0.0f}));
    ASSERT_LT(res6.repulsive[0], 0.0f);
    ASSERT_LT(res6.repulsive[1], 0.0f);
    ASSERT_GT(atan2(res6.repulsive[1], res6.repulsive[0]), -M_PI);
    ASSERT_LT(atan2(res6.repulsive[1], res6.repulsive[0]), -M_PI_2);
    ASSERT_LT(atan2(res6.result[1], res6.result[0]), 0.0);
    ASSERT_GT(atan2(res6.result[1], res6.result[0]), -M_PI_2);

    auto res7 = node_avoidance.get_vff_test(get_scan_test_7(ts));
    ASSERT_EQ(res7.attractive, std::vector<float>({1.0f, 0.0f}));
    ASSERT_GT(res7.repulsive[0], 0.0f);
    ASSERT_LT(res7.repulsive[1], 0.0f);
    ASSERT_LT(atan2(res7.repulsive[1], res7.repulsive[0]), 0.0f);
    ASSERT_GT(atan2(res7.repulsive[1], res7.repulsive[0]), -M_PI_2);
    ASSERT_LT(atan2(res7.result[1], res7.result[0]), 0.0);
    ASSERT_GT(atan2(res7.result[1], res7.result[0]), -M_PI_2);

    auto res8 = node_avoidance.get_vff_test(get_scan_test_8(ts));
    ASSERT_EQ(res8.attractive, std::vector<float>({1.0f, 0.0f}));
    ASSERT_NEAR(res8.repulsive[0], -1.0f, 0.1f);
    ASSERT_NEAR(res8.repulsive[1], 0.0f, 0.0001f);
    ASSERT_NEAR(res8.result[0], 0.0f, 0.01f);
    ASSERT_NEAR(res8.result[1], 0.0f, 0.01f);

}

TEST(vff_tests, output_vels)
{
    auot node_avoidance = std::make_shared<AvoidanceNOdeTest>();

    // Create atesting node with a scan publisher and a speed publisher
    auto test_node = rclcpp::Node::make_shared("test_node");
    auto scan_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>("input_scan", 100);

    geometry_msgs::msg::Twist last_vel;
    auto vel_sub = test_node->create_subscription<geometry_msgs::msg::Twist>(
        "output_vel", 1, [&last_vel](geometry_msgs::msg::Twist::SharedPtr msg)
        {
            last_vel = *msg; //callback function is declared as lambda
        }
        );

    ASSERT_EQ(vel_sub->get_publihser_count(), 1); //checking the no. of subscribers is 1
    ASSERT_EQ(scan_pub->get_subscription_count(), 1) //checking the no. of publijsers is 1

    rclcpp::Rate rate(30);
    rclcpp::executors::SingleThreadExecutor executor;
    executor.add_node(node_avoidance);
    executor.add_node(test_node);

    //Test for scan test #1
    //runnning for 1 second
    auto start = node_avoidance->now();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 1s)
    {
        scan_pub->publish(get_scan_test_2(node_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_NEAR(last_vel.linear.x, 0.3f, 0.0001f);
    ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.0001f);

    // Test for scan test #2
    start =  avoidance_node->now();
    while(rclcpp::ok() && (node_avoidance->now() - start) < 1s)
    {
        scan_pub->publish(get_scan_test_2(noee_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_NEAR(last_vel.linear.x, 0.3f, 0.0001f);
    ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.0001f);

    // Test for scan test #3
    start = node_avoidance->now();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
        scan_pub->publish(get_scan_test_3(node_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_LT(last_vel.linear.x, 0.3f);
    ASSERT_GT(last_vel.linear.x, 0.0f);
    ASSERT_GT(last_vel.angular.z, 0.0f);
    ASSERT_LT(last_vel.angular.z, M_PI_2);

    // Test for scan test #4
    start = node_avoidance->now();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
        scan_pub->publish(get_scan_test_4(node_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_LT(last_vel.linear.x, 0.3f);
    ASSERT_GT(last_vel.linear.x, 0.0f);
    ASSERT_GT(last_vel.angular.z, 0.0f);
    ASSERT_LT(last_vel.angular.z, M_PI_2);

    // Test for scan test #5
    start = node_avoidance->now();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
        scan_pub->publish(get_scan_test_5(node_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_LT(last_vel.linear.x, 0.3f);
    ASSERT_GT(last_vel.linear.x, 0.0f);
    ASSERT_LT(last_vel.angular.z, 0.0f);
    ASSERT_GT(last_vel.angular.z, -M_PI_2);

    // Test for scan test #6
    start = node_avoidance->now();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
        scan_pub->publish(get_scan_test_6(node_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_LT(last_vel.linear.x, 0.3f);
    ASSERT_GT(last_vel.linear.x, 0.0f);
    ASSERT_LT(last_vel.angular.z, 0.0f);
    ASSERT_GT(last_vel.angular.z, -M_PI_2);

    // Test for scan test #7
    start = node_avoidance->now();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
        scan_pub->publish(get_scan_test_7(node_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_LT(last_vel.linear.x, 0.3f);
    ASSERT_GT(last_vel.linear.x, 0.0f);
    ASSERT_LT(last_vel.angular.z, 0.0f);
    ASSERT_GT(last_vel.angular.z, -M_PI_2);

    // Test for scan test #8
    start = node_avoidance->now();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 2s) {
        scan_pub->publish(get_scan_test_8(node_avoidance->now()));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_NEAR(last_vel.linear.x, 0.0f, 0.1f);
    ASSERT_LT(last_vel.angular.z, 0.0f);
    ASSERT_GT(last_vel.angular.z, -M_PI_2);

    // Test for stooping when scan is too old
    last_vel = geometry_msgs::msg::Twist();
    while (rclcpp::ok() && (node_avoidance->now() - start) < 3s) {
        scan_pub->publish(get_scan_test_6(start));
        executor.spin_some();
        rate.sleep();
    }
    ASSERT_NEAR(last_vel.linear.x, 0.0f, 0.01f);
    ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.01f);
}

int main(int agrc, char * argv[])
{
    rclcpp::init(argc, argv)

    testing::InitGoogleTest(&argc.argv)
    return RUN_ALL_TESTS();
}