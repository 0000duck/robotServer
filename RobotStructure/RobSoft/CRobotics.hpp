#ifndef CROBOTICS_HPP
#define CROBOTICS_HPP

#include "CRobotParameter.hpp"

namespace robsoft{

class Robotics:public RobotParameter{
public:
    Robotics();
    ~Robotics();

    void setCurrentJointPosition(const Joints& currentJointPosition);   // 设置关节角度
    void setCurrentJointVelocity(const Joints& currentJointVelocity);   // 设置关节速度
    void setCurrentJointAcceleration(const Joints& currentJointAcceleration);   // 设置关节加速度
    void setCurrentJointTorque(const Joints& currentJointTorque);   // 设置关节力矩

    Joints getCurrentJointPosition() const; // 获取当前关节角度
    Joints getCurrentJointVelocity() const; // 获取当前关节速度
    Joints getCurrentJointAcceleration() const; // 获取当前关节加速度
    Joints getCurrentJointTorque() const;   // 获取当前关节力矩

    Terminal getCurrentTerminal();  // 获取基坐标系下的末端位姿
    Terminal getCurrentWorkTerminal();  // 获取工件坐标系下的末端位姿

    Terminal forwardKinematics(const Joints& joints, bool toolEnabled = true) const;    // 正运动学
    int inverseKinematics(Joints& joints, const Terminal& terminal, const Joints& lastJoints, COORDINATESYSTEM coord = COORDINATE_BASE) const;    // 逆运动学
    int inverseKinematics(JointsList& jointslist, const TerminalList &terminalList, const Joints &lastJoints, COORDINATESYSTEM coord = COORDINATE_BASE) const;    // 逆运动学批处理

    int checkJointWithinPerformance(const JointsMotionState& jointMotion);  // 检查是否符合机器人的性能要求

    double TCPCalibrate(const JointsList &jointslist, Terminal &frame); // 工具坐标系校准，仅校准位置
    double TCPCalibrateZ(const JointsList &jointslist, const Joints &jo, const Joints &jz, Terminal &frame);    // 工具坐标系校准，校准位置和工具Ｚ轴方向
    double TCPCalibrateXZ(const JointsList &jointslist, const Joints &jo, const Joints &jx, const Joints &jz, Terminal &frame); // 工具坐标系校准，校准位置和姿态
    void workFrameCalibrate(const Terminal &to, const Terminal &tx, const Terminal &ty, Terminal &frame);   // 工件坐标系校准

private:
    HomogeneousMatrix DHParameterToHomogeneousMatrix(double alpha, double a, double d, double theta) const; // DH模型转齐次矩阵

    JointsList inverse_kinematics_serial_six_convention(const Terminal &terminal, const Joints& joints) const;  // 六轴常规型机器人求逆
    JointsList inverse_kinematics_serial_six_cooperation(const Terminal &terminal, const Joints& joints) const; // 六轴协作型机器人求逆
    JointsList inverse_kinematics_serial_four_convention(const Terminal &terminal, const Joints& joints) const; // 四轴码垛型机器人求逆
    JointsList inverse_kinematics_scara_four_onerf(const Terminal &terminal, const Joints& joints) const;   // 四轴SCARA求逆，一轴为升降轴
    JointsList inverse_kinematics_scara_four_fourrf(const Terminal &terminal, const Joints& joints) const;  // 四轴SCARA求逆，四轴为升降轴
    JointsList inverse_kinematics_delta_four(const Terminal &terminal, const Joints& joints) const; // DELTA机器人求逆，一个姿态轴
    JointsList inverse_kinematics_delta_six(const Terminal &terminal, const Joints& joints) const;  // DELTA机器人求逆，三个姿态轴

    Joints chooseBestJoints(JointsList jointslist, Joints joints) const;    // 根据二范数选择最优关节角度解
    bool isAvailableJoints(Joints joints) const;    // 判断角度值是否在关节旋转范围内
    void fitJointRange(double& angle, JOINTINDEX index) const;  // 归一化角度值到一个周期内

protected:
    Joints  m_currentJointPosition;         //当前关节角度
    Joints  m_currentJointVelocity;         //当前关节速度
    Joints  m_currentJointAcceleration;     //当前关节加速度
    Joints  m_currentJointTorque;           //当前关节力矩
    Terminal m_currentTerminal;             //当前末端位姿
    Terminal m_currentWorkTerminal;         //当前工件坐标系下姿态
};

}
#endif
