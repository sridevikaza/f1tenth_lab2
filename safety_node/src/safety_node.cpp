#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
using std::placeholders::_1;
// using namespace std::chrono_literals;
using namespace std;
const float THRESH = 2.5;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        drive_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, _1));
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, _1));
    }

private:
    double speed = 0.0;
    // bool collision_detected = false;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr drive_msg)
    {
        /// TODO: update current speed
        speed = drive_msg->twist.twist.linear.x;
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drive_subscription_;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /// TODO: calculate TTC
        /// TODO: publish drive/brake message

        // save values from scan_msg
        auto angle_min = scan_msg->angle_min;
        auto angle_increment = scan_msg->angle_increment;
        auto range_min = scan_msg->range_min;
        auto range_max = scan_msg->range_max;
        auto ranges = scan_msg->ranges;

        // create stop message
        auto stop_msg = ackermann_msgs::msg::AckermannDriveStamped();
        stop_msg.drive.speed = 0.0;

        // loop through ranges
        int i = 0;
        for (auto range : ranges) {

            // check out of range or nan or inf values
            if( range > range_min && range < range_max && !isnan(range) && !isinf(range)){

                // calculate ttc
                auto beam_angle = angle_min + angle_increment*i;
                auto range_rate = speed*cos(beam_angle);
                auto ttc = range / max(-range_rate, 0.0);

                // publish stop message if below threshold
                if (ttc < THRESH && !isinf(ttc)){
                    cout << "COLLISION!!!" << endl;
                    drive_publisher_->publish(stop_msg);
                }
                
            }
            i++;
        }

    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}