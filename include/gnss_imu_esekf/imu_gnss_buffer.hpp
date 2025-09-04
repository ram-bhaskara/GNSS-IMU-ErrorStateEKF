#pragma once
#include <deque>
#include <mutex>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>


struct ImuMsg {
  sensor_msgs::msg::Imu::SharedPtr msg;
  double t;
};

struct GnssMsg {
  sensor_msgs::msg::NavSatFix::SharedPtr msg;
  double t;
};

class ImuGnssBuffer {
public:
    void pushImu(sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        double t = toSec(imu_msg->header.stamp);
        imu_buff_.push_back({imu_msg, t});
        while(imu_buff_.size() > 2000) imu_buff_.pop_front();
    }
    
    void pushGnss(sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        double t = toSec(gnss_msg->header.stamp);
        gnss_buff_.push_back({gnss_msg, t});
        while(gnss_buff_.size() > 200) gnss_buff_.pop_front();
    }

    std::vector<ImuMsg> popImuUpTo(double t) {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<ImuMsg> imu_msgs;
        while(!imu_buff_.empty() && imu_buff_.front().t <= t) {
            imu_msgs.push_back(imu_buff_.front());
            imu_buff_.pop_front();
        }
        return imu_msgs;
    }

    bool hasGnss() {
        std::lock_guard<std::mutex> lk(mutex_);
        return !gnss_buff_.empty();
    }

    GnssMsg popdestGnss() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (gnss_buff_.empty()) {
            throw std::runtime_error("No GNSS messages available to pop.");
        }
        GnssMsg gnss_msg = gnss_buff_.front();
        gnss_buff_.pop_front();
        return gnss_msg;
    }
    
    static double toSec(const builtin_interfaces::msg::Time &time) {
        return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
    }

private:
    std::deque<ImuMsg> imu_buff_;
    std::deque<GnssMsg> gnss_buff_;
    std::mutex mutex_;
};