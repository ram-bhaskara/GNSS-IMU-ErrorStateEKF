#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

struct nominalState {
    Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity(); 
    Eigen::Vector3d p_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
};


class ESEKF {
    public:
        ESEKF() {reset();}

    const nominalState& state() const { return x_; }
    const Eigen::Matrix<double,15,15>& covariance() const { return P_; }
    
    void reset() {
        x_.q_WB = Eigen::Quaterniond::Identity();
        x_.p_W = Eigen::Vector3d::Zero();
        x_.v_W = Eigen::Vector3d::Zero();
        x_.b_a = Eigen::Vector3d::Zero();
        x_.b_g = Eigen::Vector3d::Zero();
        P_.setIdentity();
        P_ *= 0.01; // Initial covariance
        g_ = 9.81;
        setProcessNoiseDefaults();
    }

    void setGravity(double g) { g_ = g; }

    void setProcessNoiseDefaults() {
    sigma_g_ = 1e-3; sigma_a_ = 1e-2;
    rw_bg_ = 1e-6; rw_ba_ = 1e-5;
  }

  void setProcessNoises(double sig_g, double sig_a, double rw_bg, double rw_ba) {
    sigma_g_ = sig_g; sigma_a_ = sig_a; rw_bg_ = rw_bg; rw_ba_ = rw_ba;
  }

    void propagate(const Eigen::Vector3d& omega_meas, const Eigen::Vector3d& acc_meas, double dt){
        if(dt <= 0) return;
        Eigen::Vector3d w = omega_meas - x_.b_g;
        Eigen::Vector3d a_b = acc_meas - x_.b_a;

        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_theta = w * dt;
        double th = delta_theta.norm();

        if(th > 1e-12) {
            delta_q.w() = std::cos(th / 2.0);
            delta_q.vec() = (delta_theta.normalized()) * std::sin(th / 2.0);
        }
        else {
            delta_q.w() = 1.0;
            delta_q.vec() = delta_theta / 2.0;
        }
        x_.q_WB = (x_.q_WB * delta_q).normalized();

        Eigen::Matrix3d R = x_.q_WB.toRotationMatrix();
        Eigen::Vector3d a_W = R * a_b + Eigen::Vector3d(0, 0, -g_);

        x_.v_W += a_W * dt;
        x_.p_W += x_.v_W * dt + 0.5 * a_W * dt * dt;

        // Cov
        using Mat = Eigen::Matrix<double,15,15>;
        Mat F = Mat::Zero();
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Om = skew(w);
        Eigen::Matrix3d Ab = skew(a_b); 

        F.block<3,3>(0,0) = -Om;
        F.block<3,3>(0,9) = -I3;
        F.block<3,3>(3,0) = -R * Ab;
        F.block<3,3>(3,12) = -R;
        F.block<3,3>(6,3) = I3;

        Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();
        G.block<3,3>(0,0) = -I3;
        G.block<3,3>(3,3) = R;
        G.block<3,3>(9,6) = I3;
        G.block<3,3>(12,9) = I3;

        Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
        Qc.block<3,3>(0,0) = (sigma_g_*sigma_g_)*I3;
        Qc.block<3,3>(3,3) = (sigma_a_*sigma_a_)*I3;
        Qc.block<3,3>(6,6) = (rw_bg_*rw_bg_)*I3;
        Qc.block<3,3>(9,9) = (rw_ba_*rw_ba_)*I3;

        Eigen::Matrix<double,15,15> Phi = Eigen::Matrix<double,15,15>::Identity() + F * dt;
        P_ = Phi * P_ * Phi.transpose() + (G * Qc * G.transpose()) * dt;
        symmetrize(P_);
    }

    void updatePosition(const Eigen::Vector3d &z_ENU, const Eigen::Matrix3d &R_meas) {
        Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
        H.block<3,3>(0,6) = Eigen::Matrix3d::Identity();

        Eigen::Vector3d y = z_ENU - x_.p_W;
        Eigen::Matrix3d S = H * P_ * H.transpose() + R_meas;
        Eigen::Matrix<double,15,3> K = P_ * H.transpose() * S.inverse();
        Eigen::Matrix<double,15,1> delta_x = K * y;

        Eigen::Vector3d delta_theta = delta_x.segment<3>(0);
        Eigen::Quaterniond delta_q(1.0, 0.5*delta_theta.x(), 0.5*delta_theta.y(), 0.5*delta_theta.z()); 
        x_.q_WB = (x_.q_WB * delta_q).normalized();
        x_.p_W += delta_x.segment<3>(3);
        x_.v_W += delta_x.segment<3>(6);
        x_.b_a += delta_x.segment<3>(9);
        x_.b_g += delta_x.segment<3>(12);

        Eigen::Matrix<double,15,15> I_KH = Eigen::Matrix<double,15,15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_meas * K.transpose();
        symmetrize(P_);
    }    

private:
    nominalState x_;
    Eigen::Matrix<double,15,15> P_;
    double g_ = 9.81; 
    double sigma_g_, sigma_a_, rw_bg_, rw_ba_;
    
    static Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
        Eigen::Matrix3d S;
        S << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return S;
    }

    static void symmetrize(Eigen::Matrix<double,15,15> &M) {
        M = 0.5 * (M + M.transpose());
    }

};