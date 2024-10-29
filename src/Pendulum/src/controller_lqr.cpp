//
// Created by zgh on 2024/9/25.
//
#include "controller_lqr.h"

Eigen::Matrix<double, 4, 1> LQRInvertedPendulumController::computeLQR() {
    Eigen::Matrix<double, 1, 1> R_inv = R_.inverse();
    Eigen::Matrix<double, 4, 4> BRIBT = B_ * R_inv(0, 0) * B_.transpose();
    //哈密顿矩阵
    Eigen::Matrix<double, 8, 8> H;
    H.topLeftCorner(4, 4) = A_;
    H.topRightCorner(4, 4) = -BRIBT;
    H.bottomLeftCorner(4, 4) = -Q_;
    H.bottomRightCorner(4, 4) = -A_.transpose();
    //计算哈密顿矩阵的特征向量和特征值
    //处理8*8
    Eigen::ComplexEigenSolver<Eigen::Matrix<double, 8, 8>> ces;
    ces.compute(H);
    //X表示形状未知，cd表示复数
    Eigen::VectorXcd eigenvalues = ces.eigenvalues();
    Eigen::MatrixXcd eigenvectors = ces.eigenvectors();
    //选择有负实部的特征向量
    Eigen::MatrixXcd Vs(8, 4);
    int index = 0;
    for (int i = 0; i < 8; ++i) {
        if (eigenvalues(i).real() < 0 && index < 4) {
            Vs.col(index) = eigenvectors.col(i);
            index++;
        }
    }
    if (index != 4) {
        ROS_ERROR("computeLQR : Unable to find 4 stable eigenvalues.");
        return K;
    }
    Eigen::MatrixXcd Vs1 = Vs.topRows(4);
    Eigen::MatrixXcd Vs2 = Vs.bottomRows(4);
    //计算出Riccati方程的解P
    Eigen::MatrixXcd P_c = Vs2 * Vs1.inverse();
    Eigen::Matrix4d P = P_c.real();
    //计算出LQR的增益矩阵K

    Eigen::Matrix<double, 1, 4> K_temp = R_inv(0, 0) * B_.transpose() * P;
    return K_temp.transpose();
}

double LQRInvertedPendulumController::get_output() {
    Eigen::Matrix<double, 4, 1> state_error = desired_state - current_state;
    return K.dot(state_error);
}
