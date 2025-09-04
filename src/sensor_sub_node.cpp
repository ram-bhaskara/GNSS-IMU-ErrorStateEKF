#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class sensorSubNode : public rclcpp::Node{
public:
    sensorSubNode() : Node("sensor_sub_node") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu0", 10,
            std::bind(&sensorSubNode::imuCallback, this, std::placeholders::_1)
        );  

        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ublox_driver/receiver_lla", 10,
            std::bind(&sensorSubNode::gnssCallback, this, std::placeholders::_1)
        );
    }

private:
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
            "IMU: ax=%.3f, ay=%.3f, az=%.3f",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
    );
}

void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
        "GNSS: lat=%.6f, lon=%.6f, alt=%.2f",
        msg->latitude, msg->longitude, msg->altitude
    );
}

rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensorSubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}