#ifndef ROBOTLINK_H
#define ROBOTLINK_H

#include <list>
#include "robotStructure.h"
#include "systemLib.h"

#define  SIMULATION

namespace rclib{

enum DRIVER_MODE {DRIVER_ANGLE_MODE, DRIVER_VELOCITY_MODE, DRIVER_TORQUE_MODE};

class RobotLink{
public:
    RobotLink();
    ~RobotLink();

    void initLink(double period, const Joints& encoderResolution, const Joints& reduceRatio, const Joints& rateTorque); // 初始化
    void startLink();
    void closeLink();

    bool setDriverMode(DRIVER_MODE mode);   // 设置驱动器模式
    DRIVER_MODE getDriveMode();    // 获取驱动器模式

    void setCurrentJointPosition(const Joints &joint); //　设置当前角度值

    bool setZero(JOINTINDEX index); // 置零
    void setJointCompensation(const Joints &joint); // 更新零位补偿

    void addMotor(const RobotMotion& rm);  // 添加关节控制量
    int getRobotMotionListNum() const;  // 获取运动数据链表长度

    void setServo(SWITCHSTATE state);   // 设置伺服状态
    SWITCHSTATE getServo(); // 获取伺服状态

    int isError(Joints &jointError);  // 获取错误信息
    void clearError();  // 清除错误

    SWITCHSTATE getHomeState(JOINTINDEX index); // 获取关节HOME传感器的状态
    void getHomeState(); // 获取关节八个HOME传感器的状态
    void setHomeEnabled(JOINTINDEX index);  // 设置HOME传感器使能，如果输入参数是WHOLE，则所有关节不使能

    void recordRobotJoints();   // 记录当前机器人关节值

    RobotMotion getRobotMotion();
    RobotMotion getLastRobotMotion();

    RobotIO getRobotIOState();
    SWITCHSTATE getIOConnectState();
    DigitalInputState getDigitalInputState();
    SWITCHSTATE getDigitalInputState(PORTINDEX index);
    DigitalOutputState getDigitalOutputState();
    SWITCHSTATE getDigitalOutputState(PORTINDEX index);
    void setDigitalOutputState(const DigitalOutputState &state);
    void setDigitalOutputState(PORTINDEX index, SWITCHSTATE state);
    AnalogInputState getAnalogInputState();
    double getAnalogInputState(PORTINDEX index);
    AnalogOutputState getAnalogOutputState();
    double getAnalogOutputState(PORTINDEX index);
    void setAnalogOutputState(const AnalogOutputState &state);
    void setAnalogOutputState(PORTINDEX index, double state);

    void setRunStateLight(SYSRUNSTATE state);
    void setPlayStateLight(SYSPLAYSTATE state);
    void setSystemStateLight(SWITCHSTATE state);
    void setErrorStateLight(SWITCHSTATE state);

    void modbusInit();
    void modbusClose();
    void modbusSend();
    void modbusRecv();
	std::string GetModuleFullPath(bool bLastPath);

    SWITCHSTATE m_getHomeState[8] ;        //传感器状态
    double m_fBeepTime;                   //蜂鸣器响的时间
private:
    void setMotor(const RobotMotion &rm); // 设置当前电机状态
    RobotMotion getMotor(); // 从电机获取当前状态
    void setRobotIOState(const RobotIO &rio); // 设置当前ＩＯ状态
    Joints readJoints(const char* path);
    void writeJoints(const char* path, const Joints &joint);
    void setPeriod();
    void setPulseNumberWithJoint(const Joints &joint);
    Joints getJointWithPulseNumber();
    virtual void updateRobotTerminal(RobotMotion &rm){}
    virtual void updateRemoteState(REMOTESTATE state){}
    virtual void updateIOState(const RobotIO &rio){}
    virtual void updateMotionState(SWITCHSTATE servo, const RobotMotion &rm){}
    virtual void updateErrorState(int error, const Joints &jointError){}

#ifdef __linux__
	static THREAD_ROUTINE_RETURN cycleTaskMotion(void* lpParameter);
    static THREAD_ROUTINE_RETURN cycleTaskIO(void* lpParameter);
#else
	static THREAD_ROUTINE_RETURN (__stdcall cycleTaskMotion)(void* lpParameter);
	static THREAD_ROUTINE_RETURN (__stdcall cycleTaskIO)(void* lpParameter);
#endif // __linux__

    

private:
    static RobotLink* robotLink;

    Thread m_threadMotion;
    Thread m_threadIO;
    Timer m_timerMotion;
    Timer m_timerIO;

    double m_period;    // 通信周期
    Joints m_encoderResolution; // 编码器精度
    Joints m_reduceRatio;   // 减速比
    Joints m_rateTorque;    // 额定力矩
    Joints m_unitPulse; // 一角度需要的脉冲数

    Joints m_jointCompensation; // 绝对值编码器零点补偿

    SYSRUNSTATE m_sysRunState;
    SYSPLAYSTATE m_sysPlayState;
    SWITCHSTATE m_SysState;
    SWITCHSTATE m_SysErrorState;

    DRIVER_MODE m_setDriverMode;
    DRIVER_MODE m_getDriveMode;
    SWITCHSTATE m_setServoState;
    SWITCHSTATE m_getServoState;
    int m_errorFlag = 0;
    Joints m_errorCode;

    RobotIO m_setRobotIO;
    RobotIO m_getRobotIO;  // 机器人ＩＯ

    std::list<RobotMotion> m_robotMotionList;   // 机器人运动控制量队列
    RobotMotion m_addRobotMotion;  // 添加机器人运动
    RobotMotion m_setRobotMotion;  // 设置机器人运动
    RobotMotion m_getRobotMotion;   // 获取机器人运动
    Mutex m_mutexRM;                //机器人运动锁

};

void set_beep_time(double time);    // 设置蜂鸣器开启一段时间
void set_beep_state(bool state);    // 设置蜂鸣器的开关状态

}

#endif
