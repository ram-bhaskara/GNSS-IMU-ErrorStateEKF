#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "gnss_imu_esekf/esekf.hpp"
#include "gnss_imu_esekf/imu_gnss_buffer.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class ESEKFNode : public rclcpp::Node {
public:
    ESEKFNode() : Node("esekf_node") {
        frame_map_ = this->declare_parameter<std::string>("frame_map", "map");
        frame_base_ = this->declare_parameter<std::string>("frame_base", "base_link");
        use_first_fix_as_origin_ = this->declare_parameter<bool>("use_first_fix_as_origin", true);

        gravity_ = this->declare_parameter<double>("gravity", 9.81);
        gnss_pos_std_ = this->declare_parameter<double>("gnss_position_std", 1.5);

        double sig_g = this->declare_parameter<double>("imu_noise.gyro_std", 1e-3);
        double sig_a = this->declare_parameter<double>("imu_noise.accel_std", 1e-2);
        double rw_bg = this->declare_parameter<double>("imu_noise.gyro_bias_rw", 1e-6);
        double rw_ba = this->declare_parameter<double>("imu_noise.accel_bias_rw", 1e-5);
   
        ekf_.reset();
        ekf_.setGravity(gravity_);
        ekf_.setProcessNoises(sig_g, sig_a, rw_bg, rw_ba);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("esekf/odom", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("esekf/path", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu0", 100,
            std::bind(&ESEKFNode::imuCallback, this, _1)
        );  

        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ublox_driver/receiver_lla", 10,
            std::bind(&ESEKFNode::gnssCallback, this, _1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        path_msg_.header.frame_id = frame_map_;

        RCLCPP_INFO(this->get_logger(), "ESEKF Node's up.");
    }

    private:

     void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double t = toSec(msg->header.stamp);
        if (!have_last_imu_) {
            last_imu_time_ = t;
            have_last_imu_ = true;
            return;
        }

        double dt = t - last_imu_time_;
        last_imu_time_ = t;

        if (dt <= 0.0) return;

        Eigen::Vector3d gyro( msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z );  
        Eigen::Vector3d accel( msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z );
        
        ekf_.propagate(gyro, accel, dt); 
        publishOdom(msg->header.stamp); 
    }

void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!origin_set_) {
            if (use_first_fix_as_origin_) {
                origin_lat_ = msg->latitude;
                origin_lon_ = msg->longitude;
                origin_alt_ = std::isfinite(msg->altitude) ? msg->altitude : 0.0;
            }
            local_cartesian_.reset(new GeographicLib::LocalCartesian(origin_lat_, origin_lon_, origin_alt_));
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Set ENU origin: lat=%.8f lon=%.8f alt=%.3f",
                        origin_lat_, origin_lon_, origin_alt_);
        }
        if (!origin_set_) return;

        double x,y,z; 
        local_cartesian_->Forward(msg->latitude, msg->longitude,
                                  std::isfinite(msg->altitude) ? msg->altitude : origin_alt_,
                                  x, y, z);
        Eigen::Vector3d z_ENU(x, y, z);

        Eigen::Matrix3d Rm = Eigen::Matrix3d::Identity() * (gnss_pos_std_ * gnss_pos_std_);
        if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            Rm << msg->position_covariance[0], msg->position_covariance[1], msg->position_covariance[2],
                  msg->position_covariance[3], msg->position_covariance[4], msg->position_covariance[5],
                  msg->position_covariance[6], msg->position_covariance[7], msg->position_covariance[8];
        }
        ekf_.updatePosition(z_ENU, Rm);
        publishOdom(msg->header.stamp);
    }

static double toSec(const builtin_interfaces::msg::Time &time) {
            return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
        
        }

void publishOdom(const builtin_interfaces::msg::Time& stamp_msg) {
        const auto& s = ekf_.state(); 
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp_msg;
        odom.header.frame_id = frame_map_;
        odom.child_frame_id = frame_base_;
        odom.pose.pose.position.x = s.p_W(0);
        odom.pose.pose.position.y = s.p_W(1);
        odom.pose.pose.position.z = s.p_W(2);
        odom.pose.pose.orientation.w = s.q_WB.w();
        odom.pose.pose.orientation.x = s.q_WB.x();
        odom.pose.pose.orientation.y = s.q_WB.y();
        odom.pose.pose.orientation.z = s.q_WB.z();

        odom.twist.twist.linear.x = s.v_W(0);
        odom.twist.twist.linear.y = s.v_W(1);
        odom.twist.twist.linear.z = s.v_W(2);
        Eigen::Matrix<double,15,15> P = ekf_.covariance();

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                odom.pose.covariance[i*6 + j] = P(6 + i, 6 + j);

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                odom.pose.covariance[(3 + i)*6 + (3 + j)] = P(0 + i, 0 + j);

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                odom.pose.covariance[i*6 + (3 + j)] = P(6 + i, 0 + j); // pos vs rot
                odom.pose.covariance[(3 + i)*6 + j] = P(0 + i, 6 + j); // rot vs pos
            }

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                odom.twist.covariance[i*6 + j] = P(3 + i, 3 + j);

        double small_ang_var = 1e-3;
        for (int i = 0; i < 3; ++i)
            odom.twist.covariance[(3 + i)*6 + (3 + i)] = small_ang_var;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = odom.header.stamp;
        tf.header.frame_id = frame_map_;
        tf.child_frame_id = frame_base_;
        tf.transform.translation.x = s.p_W(0);
        tf.transform.translation.y = s.p_W(1);
        tf.transform.translation.z = s.p_W(2);
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);

        publishPath(odom);
    }

    void publishPath(const nav_msgs::msg::Odometry & odom) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;

        path_msg_.header = odom.header;
        path_msg_.header.frame_id = frame_map_;

        path_msg_.poses.push_back(pose_stamped);

        const size_t MAX_PATH = 20000;
        if (path_msg_.poses.size() > MAX_PATH)
            path_msg_.poses.erase(path_msg_.poses.begin());

        path_pub_->publish(path_msg_);
    }

    ESEKF ekf_; 
    ImuGnssBuffer buf_; 
    std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
    bool origin_set_{false};
    bool use_first_fix_as_origin_{true};
    double origin_lat_{0}, origin_lon_{0}, origin_alt_{0};
    double gnss_pos_std_{1.5};
    double gravity_{9.81};
    std::string frame_map_, frame_base_;

    bool have_last_imu_{false};
    double last_imu_time_{0.0};

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ESEKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}