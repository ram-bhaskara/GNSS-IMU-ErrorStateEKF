#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

struct nominalState {
    // frames: W : map frame
    Eigen::Vector3d p_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity(); // attitude: body -> world
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_W = Eigen::Vector3d(0, 0, -9.81);
};


class ESEKF {
    public:
        ESEKF() {
            setProcessNoiseDefaults(); 
            P_.setZero(); 
            delta_x_.setZero();
            
            F_i_.setZero(); 
            F_i_.block<3,3>(3,0) = Eigen::Matrix3d::Identity();  
            F_i_.block<3,3>(6,3) = Eigen::Matrix3d::Identity();  
            F_i_.block<3,3>(9,6) = Eigen::Matrix3d::Identity();  
            F_i_.block<3,3>(12,9) = Eigen::Matrix3d::Identity(); 
            
            
            Qc.setZero();
            Qc.block<3,3>(0,0) = (sigma_a_ * sigma_a_) * Eigen::Matrix3d::Identity();  
            Qc.block<3,3>(3,3) = (sigma_g_ * sigma_g_) * Eigen::Matrix3d::Identity();  
            Qc.block<3,3>(6,6) = (rw_ba_ * rw_ba_) * Eigen::Matrix3d::Identity();      
            Qc.block<3,3>(9,9) = (rw_bg_ * rw_bg_) * Eigen::Matrix3d::Identity();     
            
            
            H_.setZero();
            H_t.setZero();
            Jac_error_to_nominal.setIdentity(); 
            G_.setIdentity();
                
            }

    const nominalState& state() const { return x_; }

    const Eigen::Matrix<double,18,18>& covariance() const { return P_; }

    void setProcessNoiseDefaults() {
        sigma_g_ = 1e-3; sigma_a_ = 1e-2;
        rw_bg_ = 1e-6; rw_ba_ = 1e-5;
  }

    void setProcessNoises(double sig_g, double sig_a, double rw_bg, double rw_ba) {
        sigma_g_ = sig_g; sigma_a_ = sig_a; rw_bg_ = rw_bg; rw_ba_ = rw_ba;
    }

    void propagate(const Eigen::Vector3d& omega_meas, const Eigen::Vector3d& acc_meas, double dt){
        if(dt <= 0) return;

        Eigen::Vector3d accel_diff = acc_meas - x_.b_a;
        Eigen::Vector3d gyro_diff = omega_meas - x_.b_g;
        
        // nominal state prop
        Eigen::Matrix3d R = x_.q_WB.toRotationMatrix();
        Eigen::Vector3d a_W = R * accel_diff + x_.g_W;
        x_.p_W += x_.v_W * dt + 0.5 * a_W * dt * dt;
        x_.v_W += a_W * dt;
        
        Eigen::Vector3d delta_theta = gyro_diff * dt;
        Eigen::Matrix3d delta_R = rotationMatrixFrom_dtheta(delta_theta);
        x_.q_WB = Eigen::Quaterniond(R * delta_R).normalized();

        // Cov prop
        using Mat18d = Eigen::Matrix<double,18,18>;
        Mat18d F = Mat18d::Identity();
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Ab = skew(accel_diff); 

        // syntax for mat assignment: F.block<rows, cols>(start_row, start_col)
        F.block<3,3>(0,3) = I3 * dt;
        F.block<3,3>(3,6) = -R * Ab * dt;
        F.block<3,3>(3,9) = -R * dt;
        F.block<3,3>(3,15) = I3 * dt; 
        F.block<3,3>(6,6) = delta_R.transpose(); 
        F.block<3,3>(6,12) = -I3 * dt;

        P_ = F * P_ * F.transpose() + (F_i_ * Qc * F_i_.transpose()) * dt; // integrate
        symmetrize(P_);
    }

    void updatePosition(const Eigen::Vector3d &z_ENU, const Eigen::Matrix3d &R_meas) {
        
        // GNSS position and velocity fix
        H.setZero();
        H.block<3,3>(0,0) = Eigen::Matrix<double,3,3>::Identity();

        // Jacobian from error state to measurement space
        Jac_error_to_nominal.block<6,6>(0,0) = Eigen::Matrix<double,6,6>::Identity();
        Jac_error_to_nominal.block<9,9>(10,9) = Eigen::Matrix<double,9,9>::Identity();
        Jac_error_to_nominal.block<4,3>(6,6) = Jac_ErrorQuat_WrtTheta(this->x_.q_WB);

        H_ = H * Jac_error_to_nominal; // for error state to measurement space mapping
        H_t = H_.transpose();

        Eigen::Vector3d innovation = z_ENU - x_.p_W; // innovation
        Eigen::Matrix3d S = H_ * P_ * H_t + R_meas;
        Eigen::Matrix<double,18,3> K = P_ * H_t * S.inverse();
        Eigen::Matrix<double,18,1> delta_x = K * innovation;

        P_ = ( Eigen::Matrix<double,18,18>::Identity() - K * H_ ) * P_;
    
        // injecting the error state into the nominal state
        x_.p_W += delta_x.segment<3>(0);
        x_.v_W += delta_x.segment<3>(3);
        Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
        Eigen::Quaterniond delta_q(1.0, 
                                    0.5*delta_theta.x(), 
                                    0.5*delta_theta.y(), 
                                    0.5*delta_theta.z());
        x_.q_WB = (x_.q_WB * delta_q).normalized();
        x_.b_a += delta_x.segment<3>(9);
        x_.b_g += delta_x.segment<3>(12);
        x_.g_W += delta_x.segment<3>(15); 

        // reset error state
        delta_x_.setZero();
        G_.setIdentity();
        G_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - 0.5 * skew(delta_theta);
        P_ = G_ * P_ * G_.transpose();
        symmetrize(P_);
    }    

private:
    nominalState x_;
    Eigen::Matrix<double,18,1> delta_x_; 
    Eigen::Matrix<double,18,18> P_;

    double sigma_g_, sigma_a_, rw_bg_, rw_ba_;
    
    Eigen::Matrix<double,18,12> F_i_ = Eigen::Matrix<double,18,12>::Zero();
    Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
    Eigen::Matrix<double,3,19> H = Eigen::Matrix<double,3,19>::Zero();
    Eigen::Matrix<double,19,18> Jac_error_to_nominal = Eigen::Matrix<double,19,18>::Zero();
    Eigen::Matrix<double, 3, 18> H_ = Eigen::Matrix<double, 3, 18>::Zero();
    Eigen::Matrix<double, 18, 3> H_t = Eigen::Matrix<double, 18, 3>::Zero();
    Eigen::Matrix<double,18,18> G_ = Eigen::Matrix<double,18,18>::Identity();

    static Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
        Eigen::Matrix3d S;
        S << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return S;
    }

    Eigen::Matrix3d rotationMatrixFrom_dtheta(const Eigen::Vector3d& v) {
        // Rodriguez formula for small angle approximation
        // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
        double theta = v.norm();
        if (theta < 1e-8) { // avoiding division by zero
        return Eigen::Matrix3d::Identity();
    }
        Eigen::Matrix3d V = skew(v);

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + 
                       (std::sin(theta) / theta) * V + 
                       ((1.0 - std::cos(theta)) / (theta * theta)) * V * V;
        
        return R; 
    }

    Eigen::Matrix<double,4,3> Jac_ErrorQuat_WrtTheta(const Eigen::Quaterniond& q) {
        Eigen::Matrix<double,4,3> J = Eigen::Matrix<double,4,3>::Zero();
        J.block<3,3>(1,0) = 0.5 * Eigen::Matrix3d::Identity(); 
        J = quatToLeftProductMatrix(q) * J;
        return J;
    }

    Eigen::Matrix4d quatToLeftProductMatrix(const Eigen::Quaterniond& q) {
    Eigen::Matrix4d QL = Eigen::Matrix4d::Identity() * q.w();
    QL.block<1, 3>(3, 0) += q.vec();
    QL.block<3, 1>(0, 3) -= q.vec();
    QL.block<3, 3>(1, 1) += skew(q.vec());
    return QL;
}

    static void symmetrize(Eigen::Matrix<double,18,18> &M) {
        // avoiding numerical errors in covariance matrix
        M = 0.5 * (M + M.transpose());
    }

};