//
// Created by zgh on 2024/9/26.
//

#ifndef PENDULUM_CONTROLLER_LQR_H
#define PENDULUM_CONTROLLER_LQR_H

#include "controller_base.h"

class LQRInvertedPendulumController : public InvertedPendulumController {
public:
    LQRInvertedPendulumController(ros::NodeHandle &nh, const Eigen::Matrix4d &A,
                                  const Eigen::Matrix<double, 4, 1> &B,
                                  const Eigen::Matrix4d &Q,
                                  const Eigen::Matrix<double, 1, 1> &R)
            : InvertedPendulumController(nh), A_(A), B_(B), Q_(Q), R_(R) {
        K = computeLQR();
        ROS_INFO_STREAM("LQR K:\n" << K);
    }

    double get_output() override;

private:
    Eigen::Matrix<double, 4, 1> K; //控制增益矩阵
    Eigen::Matrix4d A_;            //系统状态矩阵
    Eigen::Matrix<double, 4, 1> B_;//控制输入矩阵
    Eigen::Matrix4d Q_;            //状态权重矩阵
    Eigen::Matrix<double, 1, 1> R_;//控制输入权重矩阵

    Eigen::Matrix<double, 4, 1> computeLQR();
};

#endif //PENDULUM_CONTROLLER_LQR_H
