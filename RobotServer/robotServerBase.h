#ifndef ROBOSERVERTBASE_H
#define ROBOTSERVERBASE_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "tcpSocket.h"
#include "robotStructure.h"
#include "robotInterpreter.h"

namespace rclib {

class RobotServerBase : public TCPServer{
public:
    RobotServerBase();
    ~RobotServerBase();

    void initSystemBase(int listen_port = 8080);
    void setDof(int dof);
    SWITCHSTATE getCameraConnect(LOCATEVISIONINDEX index);

    void sendCSWJ(const char* path);    // 机器人参数文件
    void sendPZWJ(const char* path);    // 机器人配置文件
    void sendZBWJ(const char* path);    // 坐标系文件

    void sendCWXX(std::string information); // 错误信息
    void sendJGXX(std::string information); // 警告信息
    void sendTSXX(std::string information); // 提示信息

    void sendWCIO(int state);   // 完成IO操作
    void sendWCZL(int state);   // 完成指令操作
    void sendCXXH(ProgramPointer pointer);   // 程序序号
    void sendGJMD(const RobotMotion &rm);    // 关节末端数据
    void sendTDDL(const JointsList &js);    // 拖动点列
    void sendSJXH(LOCATEVISIONINDEX index);   // 视觉定位，相机序号
    void sendSJWZ(LOCATEVISIONINDEX index, const Terminal &t); // 视觉定位，相机序号+目标位姿

    void sendIOZT(SWITCHSTATE state);    // IO连接状态
    void sendSRZT(const DigitalInputState &state);    // 数字口输入状态
    void sendSCZT(const DigitalOutputState &state);    // 数字口输出状态
    void sendMNSR(const AnalogInputState &state);    // 模拟口输入状态
    void sendMNSC(const AnalogOutputState &state);    // 模拟口输出状态
    void sendCSYX(SWITCHSTATE state);    // 试运行
    void sendFSZT(SYSRUNSTATE state);    // 发送状态
    void sendSFZT(SWITCHSTATE state);   // 伺服状态
    void sendTSZT(SWITCHSTATE state);    // 调试状态
    void sendYXMS(SYSPLAYSTATE state);    // 运行模式
    void sendKZXT(int fd);    // 控制状态
    void sendCXSD(double vel);  // 全局速度

    void sendXGGJ(std::string frame);
    void sendXGYH(std::string frame);
    void sendTCPF(const Terminal &t, double p);
    void sendUSRF(const Terminal &t);
    void sendHOME(SWITCHSTATE state[8]);

    void nullApi();         //编译用接口

private:
    void initParameter();

    enum FILE_TYPE_VALUE {CSWJ, MLWJ, PZWJ, ZBWJ};
    enum COMMAND_TYPE_VALUE {JLLJ, GBLJ, QCCW, CXSD, SJXH, SJWZ, HLJZ, XWHL,
                             SCZT, MNSC, MNDL, CSYX, FSZT, SFZT, TSZT, YXMS, KZXT,
                             XGGJ, XGYH, TCPF, TCFZ, TCFX, USRF,
                             GJDD, MDDD, DDTZ, GJBJ, MDBJ, GJYX, MLYX, MLZT, MLJX, MLTZ, DBYX, DBZT, CXXH,
                             KSTD, JSTD, JLTD, ZXZB, ZXTD, SZLX, BEEP, SZYY,
                            TEST};

    virtual void excuteFile(int fd, TCPSOURCETYPE src, std::string command);
    void commandCSWJ(int fd, TCPSOURCETYPE src);
    void commandMLWJ(int fd, TCPSOURCETYPE src);
    void commandPZWJ(int fd, TCPSOURCETYPE src);
    void commandZBWJ(int fd, TCPSOURCETYPE src);

    virtual void excuteCommand(int fd, TCPSOURCETYPE src, std::string command, std::string content);
    void commandJLLJ(int fd, TCPSOURCETYPE src, std::string content);  // 连接系统
    void commandGBLJ(int fd, TCPSOURCETYPE src, std::string content);  // 关闭连接
    void commandQCCW(int fd, TCPSOURCETYPE src, std::string content);  // 清除错误
    void commandCXSD(int fd, TCPSOURCETYPE src, std::string content);  // 全局速度
    void commandSJXH(int fd, TCPSOURCETYPE src, std::string content);  // 视觉序号
    void commandSJWZ(int fd, TCPSOURCETYPE src, std::string content);  // 视觉序号＋位姿
    void commandXWHL(int fd, TCPSOURCETYPE src, std::string content);  // 限位回零
    void commandHLJZ(int fd, TCPSOURCETYPE src, std::string content);  // 置零

    void commandSCZT(int fd, TCPSOURCETYPE src, std::string content);  // 数字输出状态
    void commandMNSC(int fd, TCPSOURCETYPE src, std::string content);  // 模拟输出状态
    void commandMNDL(int fd, TCPSOURCETYPE src, std::string content);  // 模拟输出状态
    void commandCSYX(int fd, TCPSOURCETYPE src, std::string content);  // 试运行
    void commandFSZT(int fd, TCPSOURCETYPE src, std::string content);  // 运行状态
    void commandSFZT(int fd, TCPSOURCETYPE src, std::string content);  // 伺服状态
    void commandTSZT(int fd, TCPSOURCETYPE src, std::string content);  // 调试状态
    void commandYXMS(int fd, TCPSOURCETYPE src, std::string content);  // 运行模式
    void commandKZXT(int fd, TCPSOURCETYPE src, std::string content);  // 控制系统

    void commandXGGJ(int fd, TCPSOURCETYPE src, std::string content);  // 修改工具坐标系
    void commandXGYH(int fd, TCPSOURCETYPE src, std::string content);  // 修改工件坐标系
    void commandTCPF(int fd, TCPSOURCETYPE src, std::string content);
    void commandTCFZ(int fd, TCPSOURCETYPE src, std::string content);
    void commandTCFX(int fd, TCPSOURCETYPE src, std::string content);
    void commandUSRF(int fd, TCPSOURCETYPE src, std::string content);

    void commandGJDD(int fd, TCPSOURCETYPE src, std::string content);  // 关节点动
    void commandMDDD(int fd, TCPSOURCETYPE src, std::string content);  // 末端点动
    void commandDDTZ(int fd, TCPSOURCETYPE src, std::string content);  // 停止点动
    void commandGJBJ(int fd, TCPSOURCETYPE src, std::string content);  // 关节步进
    void commandMDBJ(int fd, TCPSOURCETYPE src, std::string content);  // 末端步进
    void commandGJYX(int fd, TCPSOURCETYPE src, std::string content);  // 轨迹运行
    void commandMLYX(int fd, TCPSOURCETYPE src, std::string content);  // 程序运行
    void commandMLZT(int fd, TCPSOURCETYPE src, std::string content);  // 程序暂停/运行
    void commandMLJX(int fd, TCPSOURCETYPE src, std::string content);  // 程序暂停/运行
    void commandMLTZ(int fd, TCPSOURCETYPE src, std::string content);  // 程序停止
    void commandDBYX(int fd, TCPSOURCETYPE src, std::string content);  // 程序单步运行
    void commandDBZT(int fd, TCPSOURCETYPE src, std::string content);  // 程序单步暂停
    void commandCXXH(int fd, TCPSOURCETYPE src, std::string content);  // 程序指针

    void commandKSTD(int fd, TCPSOURCETYPE src, std::string content);  // 开始拖动
    void commandJSTD(int fd, TCPSOURCETYPE src, std::string content);  // 结束拖动
    void commandJLTD(int fd, TCPSOURCETYPE src, std::string content);  // 记录拖动
    void commandZXZB(int fd, TCPSOURCETYPE src, std::string content);  // 再现准备
    void commandZXTD(int fd, TCPSOURCETYPE src, std::string content);  // 再现拖动

    void commandSZLX(int fd, TCPSOURCETYPE src, std::string content);  // 设置示教器类型
    void commandBEEP(int fd, TCPSOURCETYPE src, std::string content);  // 设置蜂鸣器
    void commandSZYY(int fd, TCPSOURCETYPE src, std::string content);  // 设置语言

    void commandTEST(int fd, TCPSOURCETYPE src, std::string content);

    virtual void updateRobotClient(){}

    virtual void setErrorClear(){}    // 清除错误
    virtual bool setJointZero(JOINTINDEX joint) = 0; // 置零

    virtual int returnZero(double vel = 0.1) = 0; // 回到初始位置
    virtual int moveToZero(double vel = 0.05) = 0; // 传感器回零
    virtual int jointJOG(JOINTINDEX index, MOVEDIRECTION direction, double velocity) = 0;  // 关节点动
    virtual int terminalJOG(TERMINALINDEX index, MOVEDIRECTION direction, double velocity, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;   // 末端点动
    virtual void stopJOG(){} // 停止点动
    virtual int jointStep(JOINTINDEX index, MOVEDIRECTION direction, double step, double vel) = 0;  // 关节步进
    virtual int terminalStep(TERMINALINDEX index, MOVEDIRECTION direction, double step, double vel, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;   // 末端步进

	virtual int moveABSJoint(const Joints& ps, double vel) = 0;
    virtual int moveABSJoint(const JointsList& ps, double vel) = 0;
    virtual int moveABSJoint(const JointsList& ps, std::vector<double>& tm) = 0;
    virtual int moveABSJointR(const Joints& ps, double vel) = 0;
    virtual int moveABSJointR(const JointsList &ps, double vel) = 0;
    virtual int moveABSJointR(const JointsList &ps, std::vector<double> &tm) = 0;

    virtual int moveJoint(const Terminal& p, double vel, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveJoint(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveJointR(const Terminal& p, double vel, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveJointR(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;

    virtual int moveLine(const HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveLine(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveLineR(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveCircle(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveCircleR(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;

    virtual int moveLineCon(const HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveLineCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveLineRCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveCircleCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveCircleRCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;
    virtual int moveStartCon(int delay = 2) = 0;
    virtual int moveEndCon()= 0;

    virtual int moveCurve(const TerminalList& ps, std::vector<double>& vel, double acc = 0.8, double jerk = 0.8, double angle = 0.8, double bpre = 0.01, COORDINATESYSTEM frame = COORDINATE_BASE) = 0;

    virtual int moveJump(const HomogeneousMatrix& m, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3) = 0;    // 门运动
    virtual int moveJump(const Terminal& p, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3) = 0;

	virtual int waitCommandEnd()=0;

	virtual void dragModeStart() = 0;
	virtual void dragModeEnd() = 0;
	virtual bool dragModeTimeStart(int period_ms, int timeLength_s) = 0;
	virtual bool dragModePlayPrepare(double vel) = 0;
	virtual bool dragModePlay(int period_ms) = 0;

	virtual double calibrateTCP(const JointsList& js, Terminal& t) = 0;
	virtual double calibrateTCFZ(const JointsList& js, const Joints& jo, const Joints& jz, Terminal& t) = 0;
	virtual double calibrateTCFX(const JointsList& js, const Joints& jo, const Joints& jx, const Joints& jz, Terminal& t) = 0;
	virtual void calibrateUSRF(const Terminal& to, const Terminal& tx, const Terminal& ty, Terminal& t) = 0;

	virtual bool programVelocity(double vel) = 0;
	virtual void programLoad(const char* path) {}
    virtual void programRun()= 0;
    virtual void programPause(bool bPause=true){}
    virtual void programStop(){}
    virtual void programStepRun(){}
    virtual void programStepPause(){}
    virtual void setProgramPointer(const ProgramPointer &pointer){}  // 设置程序指针
    virtual void setBreakPointer(const ProgramPointer &pointer){}  // 设置程序指针

    virtual Terminal getVisionLocation(LOCATEVISIONINDEX index) = 0;  // 获取对应序号的视觉位置点
    virtual void setVisionLocation(LOCATEVISIONINDEX index, const Terminal &t){} // 设置视觉定位点

    virtual void setRobotParameter(const RobotParameter& robot){} // 修改机器人参数

    virtual void setRobotPreference(const RobotPreference &rp){}

    virtual int modifyToolFrame(std::string name) = 0; // 修改工具坐标系
    virtual int modifyWorkFrame(std::string name) = 0; // 修改工件坐标系
    virtual void setRobotFrame(const RobotFrame &rf){}

    virtual void setDigitalOutput(DigitalOutputState state){}   // 设置数字输出状态
    virtual void setDigitalOutput(PORTINDEX index, SWITCHSTATE state){} // 设置数字输出状态
    virtual void setAnalogOutput(AnalogOutputState state){}      // 设置模拟输出状态
    virtual void setAnalogOutput(PORTINDEX index, double state){}   // 设置模拟输出状态

    virtual void setTryRunState(SWITCHSTATE state){}
    virtual void setDebugState(SWITCHSTATE state){}
    virtual void setPlayState(SYSPLAYSTATE state){}
    virtual void setServoState(SWITCHSTATE state){}
    virtual void setRunState(SYSRUNSTATE state){}

    virtual void setTeachType(int n)=0;                 //设置示教器类型
    virtual void setBeep(double time)=0;
    virtual void setSZYY(int type)=0;
private:
    std::map<std::string, FILE_TYPE_VALUE> m_fileType;
    std::map<std::string, COMMAND_TYPE_VALUE> m_commandType;

    std::map<int, LOCATEVISIONINDEX> m_cameraFd;

    int m_dof;
};

}

#endif
