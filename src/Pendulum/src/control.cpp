//
// Created by zgh on 2024/9/25.
//
#include "controller_lqr.h"

const int control_freq = 100;

const double M = 2.0;
const double m = 0.1;
const double g = 9.8;
const double l = 0.5 / 2.0;
const double I = 1.0 / 3.0 * m * l * l;
const double b = 0.0;
const double P = (M + m) * I + M * m * l * l;

std::string CONTROLLER;

Eigen::Matrix4d A;            //系统状态矩阵
Eigen::Matrix<double, 4, 1> B;//控制输入矩阵
Eigen::Matrix4d Q;            //状态权重矩阵
Eigen::Matrix<double, 1, 1> R;//控制输入权重矩阵

//char * 表示指向单个字符数组的指针（即字符串）。
//char ** 表示指向指针的指针，也就是指向一组字符指针的指针。
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"Controller"); //节点的名称，用于标识这个 ROS 节点。在 ROS 系统中，每个节点都有一个唯一的名称。
    ros::NodeHandle nh("~"); //"~" 表示使用私有命名空间，这意味着这个节点可以访问以其名称为前缀的参数。这有助于组织和管理参数，使得不同节点可以有自己的参数而不会冲突。
    std::string default_value="LQR";


    nh.param("controller", CONTROLLER, default_value);
    ROS_INFO_STREAM("Controller: " << CONTROLLER);
    ros::Rate loop_rate(control_freq);

    A << 0, 1, 0, 0,
            0, -b * (I + m * l * l) / P, m * m * g * l * l / P, 0,
            0, 0, 0, 1,
            0, -b * m * l / P, m * g * l * (M + m) / P, 0;
    B << 0, (I + m * l * l) / P, 0, m * l / P;

    Q << 10.0, 0.0, 0.0, 0.0,
            0.0, 10.0, 0.0, 0.0,
            0.0, 0.0, 10.0, 0.0,
            0.0, 0.0, 0.0, 10.0; //比较关心收敛速度
    R << 0.1; //不是很在意输入的大小

    std::unique_ptr<InvertedPendulumController> controller;
    if (CONTROLLER == "LQR")
    {
        controller = std::make_unique<LQRInvertedPendulumController>(nh, A, B, Q, R);
    }

    while (ros::ok())
    {
        ros::spinOnce();
        controller->balance();
        loop_rate.sleep();
    }

    return 0;

}