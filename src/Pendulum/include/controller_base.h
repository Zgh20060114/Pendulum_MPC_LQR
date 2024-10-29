//
// Created by zgh on 2024/9/25.
//

#ifndef ROS_PENDULIM_CONTROLLER_BASE_H
#define ROS_PENDULIM_CONTROLLER_BASE_H

#include <ros/ros.h>
//包含了 ROS 的基本功能和 API，用于初始化节点、处理通信、订阅和发布消息等
#include <control_msgs/JointControllerState.h>
//定义了 JointControllerState 消息类型，用于表示关节控制器的状态信息。它通常包含关节的位置、速度、力矩等数据，方便在 ROS 中进行控制和监测。
#include <std_msgs/Float64.h>
//标准消息类型，定义了一个双精度浮点数的消息
#include <Eigen/Dense>
//提供了线性代数的功能，包括矩阵和向量运算
#include <math.h>
//C/C++ 的标准数学库，提供基本的数学函数，如三角函数、指数函数等。
#include <memory>
//C++ 的智能指针库，提供了对动态内存的安全管理，避免内存泄漏。它允许使用 std::shared_ptr 和 std::unique_ptr 等智能指针。
#include <sensor_msgs/JointState.h>
//表示机器人关节的状态，包括关节的位置、速度和力矩等信息。

class InvertedPendulumController
{
public:
    InvertedPendulumController(ros::NodeHandle &nh)
    {
        command_pub=nh.advertise<std_msgs::Float64>("/pendulum/x_controller/command",10); //主题发布者，向 /pendulum/x_controller/command 发布 Float64 类型的消息
        joint_state_sub=nh.subscribe("/pendulum/joint_states",10,&InvertedPendulumController::jointStateCallback, this);
        //主题订阅者,订阅 /pendulum/joint_states 主题，队列大小为 10。当有新的关节状态消息到达时，它会调用 jointStateCallback 方法进行处理
        //谁发的joint_states主题
        current_state<<0.0,0.0,0.0,0.0;
        desired_state<<0.0,0.0,0.0,0.0;
    }
    virtual ~InvertedPendulumController() {}
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &data)
    {
        current_state(0)=data->position[0];
        current_state(1)=data->velocity[0];
        current_state(2)=-data->position[1];
        current_state(3)=-data->velocity[1];  //为什么是负的
        //每隔指定的时间打印日志信息
        ROS_INFO_THROTTLE(1,"x_pos: %f m",current_state(0));
        ROS_INFO_THROTTLE(1,"x_vel: %f m/s",current_state(1));
        ROS_INFO_THROTTLE(1,"theta_pos: %f rad",current_state(2));
        ROS_INFO_THROTTLE(1,"theta_vel: %f rad/s",current_state(3));
    }

    void balance()
    {
        double output = get_output();
        command_msg.data=output;
        command_pub.publish(command_msg);
        last_balance_time=ros::Time::now();
        ROS_INFO_THROTTLE(1,"command: %f",command_msg.data);
    }

    // = 0 表示这是一个纯虚函数，意味着这个函数在基类中没有实现。任何继承该基类的子类都必须提供这个函数的实现
    // 面向对象编程中的多态性机制，允许不同的子类有不同的实现
    virtual double get_output() =0;
private:
    ros::Publisher command_pub;
    ros::Subscriber joint_state_sub;
    std_msgs::Float64 command_msg;
protected:
    //它允许子类访问基类中的某些成员，同时又限制了这些成员被外部代码直接访问
    Eigen::Vector4d current_state; //4*1
    Eigen::Vector4d desired_state;
    ros::Time last_balance_time;


};




#endif //ROS_PENDULIM_CONTROLLER_BASE_H
