#ifndef CMOTIONPLANNING_HPP
#define CMOTIONPLANNING_HPP

#include "CPolynomial.hpp"
#include "CDataStructure.hpp"
#include "CRobotics.hpp"
#include "CPathPlanning.hpp"

namespace robsoft{

class MOVEJOG{
public:
    MOVEJOG();
    MOVEJOG(const Robotics &robot);
    ~MOVEJOG();

    void setRobotics(const Robotics &robot);
    void setJogMode(JOINTINDEX index, double vel);
    void setJogMode(TERMINALINDEX index, double vel, COORDINATESYSTEM coord = COORDINATE_BASE);

    void resetState();
    bool isFinish();  // 查看点动是否结束
    int getJogPoint(JointsMotionState &jointMotionState);    // 获取一个新的点
    void setFinish();  // 结束点动

private:
    Robotics m_robot;
    JOGPolynomial m_jogPolynomial;

    Joints m_initJoint;
    Terminal m_initTerminal;
    Terminal m_initWorkTerminal;

    int m_jointTerminalFlag;    // 0:joint 1:terminal
    COORDINATESYSTEM m_coord;
    JOINTINDEX m_jointIndex;
    TERMINALINDEX m_terminalIndex;
    JointsMotionState m_initJointMotionState, m_lastJointMotionState;
};

class MOVESTEP{
public:
    MOVESTEP();
    MOVESTEP(const Robotics &robot);
    ~MOVESTEP();

    void setRobotics(const Robotics &robot);
    void setStepMode(JOINTINDEX index, double len, double vel);
    void setStepMode(TERMINALINDEX index, double len, double vel, COORDINATESYSTEM coord = COORDINATE_BASE);

    int start();    // 开始规划
    JointsMotionStateList getTraj() const;   //　获取规划的轨迹点
    void getTraj(JointsMotionStateList& jointTraj) const; // 获取规划的轨迹点

    void resetState();

private:
    Robotics m_robot;
    SPolynomial m_sPolynomial;

    Joints m_initJoint;
    Terminal m_initTerminal;
    Terminal m_initWorkTerminal;

    int m_jointTerminalFlag;    // 0:joint 1:terminal
    COORDINATESYSTEM m_coord;
    JOINTINDEX m_jointIndex;
    TERMINALINDEX m_terminalIndex;
    JointsMotionState m_initJointMotionState, m_lastJointMotionState;

    JointsMotionStateList m_jointTraj;
};

class MOVEJOINT{
public:
    MOVEJOINT(const Robotics &robot);
    ~MOVEJOINT();

    void setWayPointsWithTime(const JointsList &joints, const std::vector<double> &times);
    void setWayPointsWithTime(const JointsList &joints, const JointsList &vel, const JointsList &acc, const std::vector<double> &times);
    void setWayPointsWithTime(const TerminalList &terminals, const std::vector<double> &times, COORDINATESYSTEM coord = COORDINATE_BASE);
    void setWayPointsWithVel(const JointsList &joints, const double &vel);
    void setWayPointsWithVel(const TerminalList &terminals, const double &vel, COORDINATESYSTEM coord = COORDINATE_BASE);

    int getTrajNum() const; // 获取轨迹段数

    int startNext();    // 开始规划下一段轨迹
    int start();    // 规划整段轨迹
    JointsMotionStateList getTraj() const;   //　获取规划的轨迹点
    void getTraj(JointsMotionStateList& jointTraj) const; // 获取规划的轨迹点

    void resetState();  // 重置状态

private:
    Robotics m_robot;
    std::vector<QuinticNewtonPolynomial> m_quinticNewtonPolynomial;
    std::vector<QuinticContinuousPolynomial> m_quinticContinuousPolynominal;
    JointsMotionState m_initJointMotionState, m_lastJointMotionState;

    JointsMotionStateList m_jointTraj;
};

class MOVELC{
public:
    MOVELC(const Robotics &robot);
    ~MOVELC();

    void setWayPoints(const Terminal &t1, double vel, double acc, double jerk,  COORDINATESYSTEM coord = COORDINATE_BASE);
    void setWayPoints(CIRCLETYPE cir, const Terminal &t1, const Terminal &t2, double vel, double acc, double jerk, COORDINATESYSTEM coord = COORDINATE_BASE);
    void setWayPoints(const Terminal &t1, double vel, double acc, double jerk,  double turn, COORDINATESYSTEM coord = COORDINATE_BASE);
    void setWayPoints(CIRCLETYPE cir, const Terminal &t1, const Terminal &t2, double vel, double acc, double jerk, double turn, COORDINATESYSTEM coord = COORDINATE_BASE);

    int start();    // 开始规划计算
    void clear();   // 清楚规划得到的轨迹点，释放内存

    JointsMotionStateList getTraj() const;   //　获取规划的轨迹点
    void getTraj(JointsMotionStateList& jointTraj) const;   //　获取规划的轨迹点
    void getLeftTurnTraj(JointsMotionStateList& jointTraj);  // 获取除左侧转弯区以外的点
    void getRightTurnTraj(JointsMotionStateList& jointTraj); // 获取除右侧转弯区以外的点
    void getBothTureTraj(JointsMotionStateList& jointTraj);  // 获取除两侧转弯区以外的点
    double getLeftTurnTime(); // 获取左侧转弯区的时间
    double getRightTurnTime();    // 获取右侧转弯区时间
    JointsMotionState getLeftTurnPoint(); // 获得左侧转弯区分界点
    JointsMotionState getRightTurnPoint();    // 获得右侧转弯区分界点

private:
    Robotics m_robot;

    int m_type; // 直线还是圆弧 0:直线 1:圆弧
    double m_turn;  // 转弯半径
    double m_acc, m_jerk;

    double m_len1, m_len2, m_len3;

    Line m_line;
    Circle m_cir;
    Slerp m_slerp1, m_slerp2, m_slerp3;
    SPolynomial m_sPolynominalP, m_sPolynominalA;

    JointsMotionStateList m_jointTraj;
};

class MOVETCON{
public:
    MOVETCON();
    MOVETCON(const Robotics &robot);
    ~MOVETCON();

    void setRobotics(const Robotics &robot);

    int addPath(const MOVELC &lc); // 添加一条路径
    void endPath(); //　结束路径添加
    void setEnd();  // 连续运动结束标志
    bool isEnd() const; // 判断是否结束
    RobotMotion getLastRobotMotion() const;

    int getTrajNum() const; // 返回已规划但未获取的轨迹段数
    JointsMotionStateList getTraj();   //　获取规划的轨迹点
    void getTraj(JointsMotionStateList& jointTraj);  // 分段依次获取轨迹点

    void start();  // 重置状态

private:
    Robotics m_robot;
    bool m_endState;
    RobotMotion m_robotMotion;

    std::list<MOVELC> m_moveList;
    std::list<JointsMotionStateList> m_jointTraj;
};

class MOVEBCURVE{
public:
    MOVEBCURVE(const Robotics &robot);
    ~MOVEBCURVE();

    void setWayPoints(const TerminalList &terminals, std::vector<double> vel, double acc, double jerk, double angle, double precision, COORDINATESYSTEM coord = COORDINATE_BASE);
    int start();   // 开始规划计算
    JointsMotionStateList getTraj() const;   //　获取规划的轨迹点
    void getTraj(JointsMotionStateList& jointTraj) const;   //　获取规划的轨迹点

private:
    Robotics m_robot;

     JointsMotionStateList m_jointTraj;
};

}

#endif
