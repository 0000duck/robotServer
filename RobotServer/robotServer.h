#ifndef ROBOTSERVER_H
#define ROBOTSERVER_H

#include "robotServerBase.h"
#include "robotInterpreter.h"
#include "robotStructure.h"
#include "robotServerPrintInfo.h"
#include "robotLink.h"

#include "RobSoft/CMotionPlanning.hpp"

namespace rclib {

class RobotServer:public RobotServerBase, public RobotInterpreter, public RobotLink, public RobotServerPrintInfo{
private:
    RobotServer();
    ~RobotServer();

public:
    static RobotServer* m_robotServer;
    static RobotServer* initance();
    static void delInitance();

    bool initSystem(int listen_port = 8080);    // 初始化系统
    void controlSystem();   // 获取系统控制权限
    SWITCHSTATE getControlState() const;    // 判断是否具有控制权限

    virtual void setErrorClear();    // 清除错误
    virtual bool setJointZero(JOINTINDEX joint); // 置零

    virtual int returnZero(double vel = 0.1); // 回到初始位置
    virtual int moveToZero(double vel = 0.05); // 传感器回零
    virtual int jointJOG(JOINTINDEX index, MOVEDIRECTION dir, double vel);  // 关节点动
    virtual int terminalJOG(TERMINALINDEX index, MOVEDIRECTION dir, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);   // 末端点动
    virtual void stopJOG(); // 停止点动
    virtual int jointStep(JOINTINDEX index, MOVEDIRECTION dir, double step, double vel);  // 关节步进
    virtual int terminalStep(TERMINALINDEX index, MOVEDIRECTION dir, double step, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);   // 末端步进

    virtual int moveABSJoint(const Joints& ps, double vel);
    virtual int moveABSJoint(const JointsList& ps, double vel);
    virtual int moveABSJoint(const JointsList& ps, std::vector<double>& tm);
    virtual int moveABSJointR(const Joints& ps, double vel);
    virtual int moveABSJointR(const JointsList &ps, double vel);
    virtual int moveABSJointR(const JointsList &ps, std::vector<double> &tm);

    virtual int moveJoint(const Terminal& p, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveJoint(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveJointR(const Terminal& p, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveJointR(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);

    virtual int moveLine(const HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveLine(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveLineR(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveCircle(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveCircleR(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);

    virtual int moveLineCon(const HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveLineCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveLineRCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveCircleCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveCircleRCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    virtual int moveStartCon(int delay = 2);
    virtual int moveEndCon();

    virtual int moveCurve(const TerminalList& ps, std::vector<double>& vel, double acc = 0.8, double jerk = 0.8, double angle = 0.8, double bpre = 0.01, COORDINATESYSTEM frame = COORDINATE_BASE);

    virtual int moveJump(const HomogeneousMatrix& m, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3);    // 门运动
    virtual int moveJump(const Terminal& p, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3);

    virtual int waitCommandEnd();

    virtual void dragModeStart();
    virtual void dragModeEnd();
    virtual bool dragModeTimeStart(int period_ms, int timeLength_s);
    virtual bool dragModePlayPrepare(double vel);
    virtual bool dragModePlay(int period_ms);
    std::list<Joints> getDragPoint();

    virtual double calibrateTCP(const JointsList& js, Terminal& t);
    virtual double calibrateTCFZ(const JointsList& js, const Joints& jo, const Joints& jz, Terminal& t);
    virtual double calibrateTCFX(const JointsList& js, const Joints& jo, const Joints& jx, const Joints& jz, Terminal& t);
    virtual void calibrateUSRF(const Terminal& to, const Terminal& tx, const Terminal& ty, Terminal& t);

    virtual bool programVelocity(double vel);
    virtual double getProgramVelocity();
    virtual void programLoad(const char* path);
    virtual void programRun();
    virtual void programPause(bool bPause=true);
    virtual void programStop();
    virtual void programStepRun();
    virtual void programStepPause();
    virtual void setProgramPointer(const ProgramPointer &pointer);  // 设置程序指针
    virtual void setBreakPointer(const ProgramPointer &pointer);  // 设置程序断点指针

    virtual SWITCHSTATE getVisionConnenct(LOCATEVISIONINDEX index); // 判断相机是否连接
    virtual Terminal getVisionLocation(LOCATEVISIONINDEX index);  // 获取对应序号的视觉位置点
    virtual void setVisionLocation(LOCATEVISIONINDEX index, const Terminal &t); // 设置视觉定位点

    virtual void setRobotParameter(const RobotParameter& robot); // 修改机器人参数
    RobotParameter getRobotParameter() const; // 获取当前机器人参数

    virtual void setRobotPreference(const RobotPreference &rp);
    RobotPreference getRobotPreference() const;

    virtual int modifyToolFrame(std::string name); // 修改工具坐标系
    virtual int modifyWorkFrame(std::string name); // 修改工件坐标系
    virtual void setRobotFrame(const RobotFrame &rf);
    RobotFrame getRobotFrame() const;

    virtual void setDigitalOutput(DigitalOutputState state);   // 设置数字输出状态
    virtual void setDigitalOutput(PORTINDEX index, SWITCHSTATE state); // 设置数字输出状态
    virtual void setAnalogOutput(AnalogOutputState state); // 设置模拟输出状态
    virtual void setAnalogOutput(PORTINDEX index, double state);   // 设置模拟输出状态
    RobotIO getRobotIO() const;
    virtual SWITCHSTATE getIOConnect() const;    // 是否连接了ＩＯ模块
    virtual DigitalInputState getDigitalInput() const;
    virtual SWITCHSTATE getDigitalInput(PORTINDEX index) const; // 获取数字输入状态
    virtual DigitalOutputState getDigitalOutput() const;
    virtual SWITCHSTATE getDigitalOutput(PORTINDEX index) const;    // 获取数字输出状态
    virtual AnalogInputState getAnalogInput() const;
    virtual double getAnalogInput(PORTINDEX index) const;   // 获取模拟输入状态
    virtual AnalogOutputState getAnalogOutput() const;
    virtual double getAnalogOutput(PORTINDEX index) const;  // 获取模拟输出状态

    virtual void setTryRunState(SWITCHSTATE state);
    virtual void setDebugState(SWITCHSTATE state);
    virtual void setPlayState(SYSPLAYSTATE state);
    virtual void setServoState(SWITCHSTATE state);
    virtual void setRunState(SYSRUNSTATE state);
    RobotState getRobotState() const;

    ProgramPointer getProgramPointer();

    void getInfo(INFOSTATE &infoType, std::string &infoString);

    virtual void setTeachType(int n);      //设置示教器类型
    virtual void setBeep(double time);     //设置蜂鸣器
    virtual void setSZYY(int n);           //设置语言类型
private:
    void updateRobotStateMotionIO();    // 更新机器人的状态、运动和ＩＯ
    virtual void updateRobotClient();   // 把全部状态更新给客户端
    virtual void printInfo(INFOSTATE infoType, std::string infoString); // 打印消息
    virtual void updateProgramPointer(const ProgramPointer &pointer);  // 更新程序当前的指针
    virtual void updateRobotTerminal(RobotMotion &rm);  // 根据关节计算机器人的末端
    virtual void updateRemoteState(REMOTESTATE state); // 根据远程状态控制程序运行
    virtual void updateIOState(const RobotIO &rio); // 更新外部ＩＯ状态
    virtual void updateMotionState(SWITCHSTATE servo, const RobotMotion &rm);   // 更新运动状态
    virtual void updateErrorState(int error, const Joints &jointError); // 更新错误状态
    virtual void updateVisionLocation(int index, const Terminal &t); // 更新视觉位置点
#ifdef _WIN32
	static THREAD_ROUTINE_RETURN (__stdcall cycleTaskupdateState)(void* lpParameter);
	static THREAD_ROUTINE_RETURN(__stdcall threadSendTraj)(void* lpParameter);
	static THREAD_ROUTINE_RETURN(__stdcall threadSendConTraj)(void* lpParameter);
	static THREAD_ROUTINE_RETURN(__stdcall threadSendMoveJOG)(void* lpParameter);
	static THREAD_ROUTINE_RETURN(__stdcall threadMoveToZero)(void* lpParameter);
#else
	static THREAD_ROUTINE_RETURN cycleTaskupdateState(void* lpParameter);
    static THREAD_ROUTINE_RETURN threadSendTraj(void* lpParameter);
    static THREAD_ROUTINE_RETURN threadSendConTraj(void* lpParameter);
    static THREAD_ROUTINE_RETURN threadSendMoveJOG(void* lpParameter);
    static THREAD_ROUTINE_RETURN threadMoveToZero(void* lpParameter);
#endif // _Win32_

    

    int startSendPointList(const JointsMotionStateList &js);

    void setCMDRightFlag(int flag);
    int waitCMDRightFlag();
    bool isCMDFinished();   // 判断是否运行结束
    void setCMDFinishFlag(int flag);
    int waitCMDFinishFlag();
    void setCMDOtherFlag(int flag); //用于与视觉的同步
    int waitCMDOtherFlag();

    int judgeMovePrepare();   // 判断运动的准备条件是否充分

private:
    static RobotServer* robotServer;

    Robotics m_robot;
    RobotParameter m_robotParameter;    // 机器人参数
    RobotState m_robotState;    // 机器人状态
    RobotPreference m_robotPreference;  // 机器人配置
    RobotIO m_robotIO;  // 机器人外部接口状态
    RobotFrame m_robotFrame;    // 机器人坐标系
    RobotInfo m_robotInfo;  // 机器人消息
    SWITCHSTATE m_controlFlag;  // 机器人控制状态

    REMOTESTATE m_remoteState = REMOTE_NONE;

	int isFirst;		//第一次急停
    int m_errorFlag;    // 错误标志
    Joints m_errorCode; // 各个关节的错误码
    SWITCHSTATE m_homeState;    // 是否触发零位传感器

    std::list<Joints> m_dragPointList;

    int m_cmdRightFlag; // 指令是否正确标志位，-1:执行中 0:正确 >1:出错
    int m_cmdFinishFlag;    // 指令是否完成标志位，-1:执行中 0:正确 >1:出错
    int m_cmdOtherFlag;    // 指令是否完成标志位，-1:执行中 0:正确 >1:出错

    Timer m_timerUpdateState;
    Thread m_threadUpdateState; // 更新客户端状态的线程

    JointsMotionStateList m_trajList;    // 规划好的待发送的点列
    std::list<JointsMotionStateList> m_trajListCon;    // 连续运动规划好的待发送的点列
    Thread m_threadSendTraj;    // 发送关节控制量的线程

    int m_conTrajWaitNum;
    bool m_conTrajFlag;  // 连续运动状态

    MOVEJOG m_jogMove;  // 点动运行
    MOVETCON m_conMove; // 连续轨迹运行

    ProgramPointer m_programPointer;
};

}

#endif
