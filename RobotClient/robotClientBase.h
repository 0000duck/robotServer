#ifndef ROBOTCLIENTBASE_H
#define ROBOTCLIENTBASE_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "tcpSocket.h"
#include "robotStructure.h"
#include "robotInterpreter.h"

namespace rclib{

MY_CLASS RobotClientBase
        : public TCPClient
        ,public RobotInterpreter
{
public:
    RobotClientBase();
    ~RobotClientBase();

public:
    std::list<Joints> getDragPoint();

    double calibrateTCP(const JointsList& js, Terminal& t);
    double calibrateTCFZ(const JointsList& js, const Joints& jo, const Joints& jz, Terminal& t);
    double calibrateTCFX(const JointsList& js, const Joints& jo, const Joints& jx, const Joints& jz, Terminal& t);
    void calibrateUSRF(const Terminal& to, const Terminal& tx, const Terminal& ty, Terminal& t);

    Terminal getVisionLocation(LOCATEVISIONINDEX index);  // 获取对应序号的视觉位置点

    void getInfo(INFOSTATE &infoType, std::string &infoString);
    virtual void printInfo(INFOSTATE infoType, std::string infoString);

    void setCMDRightFlag(int flag);
    int waitCMDRightFlag();
    bool isCMDFinished();   // 判断是否运行结束
    void setCMDFinishFlag(int flag);
    int waitCMDFinishFlag();

    SWITCHSTATE getControlState() const;    // 判断是否具有控制权限

    void setRobotParameter(const RobotParameter& robot);    // 修改机器人参数
    RobotParameter getRobotParameter() const; // 获取当前机器人参数

    void setRobotPreference(const RobotPreference &rp);
    RobotPreference getRobotPreference() const;

    int modifyToolFrame(std::string name);  // 修改工具坐标系
    int modifyWorkFrame(std::string name);  // 修改工件坐标系
    void setRobotFrame(RobotFrame &rf);
    RobotFrame getRobotFrame() const;

    void setDigitalOutput(DigitalOutputState state);    // 设置数字输出状态
    void setDigitalOutput(PORTINDEX index, SWITCHSTATE state);  // 设置数字输出状态
    void setAnalogOutput(AnalogOutputState state);  // 设置模拟输出状态
    void setAnalogOutput(PORTINDEX index, double state);    // 设置模拟输出状态
    RobotIO getRobotIO() const;
    SWITCHSTATE getIOConnect() const;   // 是否连接了ＩＯ模块
    DigitalInputState getDigitalInput() const;
    SWITCHSTATE getDigitalInput(PORTINDEX index) const; // 获取数字输入状态
    DigitalOutputState getDigitalOutput() const;
    SWITCHSTATE getDigitalOutput(PORTINDEX index) const;    // 获取数字输出状态
    AnalogInputState getAnalogInput() const;
    double getAnalogInput(PORTINDEX index) const;   // 获取模拟输入状态
    AnalogOutputState getAnalogOutput() const;
    double getAnalogOutput(PORTINDEX index) const;  // 获取模拟输出状态

    void setTryRunState(SWITCHSTATE state);
    void setDebugState(SWITCHSTATE state);
    void setPlayState(SYSPLAYSTATE state);
    void setServoState(SWITCHSTATE state);
    void setRunState(SYSRUNSTATE state);
    RobotState getRobotState() const;
    RobotMotion getRobotMotion() ;

    ProgramPointer getProgramPointer();
    virtual void updateProgramPointer(const ProgramPointer &pointer);  // 更新程序当前的指针

    bool bSetFileUpdate();


    SWITCHSTATE m_getHomeState[8];        //传感器状态
public:
    bool initSystemBase(const char* server_ip, int server_port);

    void sendCSWJ(const char* path);    // 参数文件
    void sendMLWJ(const char* path);    // 程序文件
    void sendPZWJ(const char* path);    // 配置文件
    void sendZBWJ(const char* path);    // 坐标文件

    void sendJLLJ();   // 建立连接
    void sendQCCW();    // 清除错误
    void sendCXSD(double vel);  // 全局速度
    void sendSJXH(LOCATEVISIONINDEX index);    // 视觉序号
    void sendXWHL(double vel);    // 限位回零
    void sendHLJZ(JOINTINDEX index);   // 回零校准

    void sendSCZT(const DigitalOutputState &state);  // 输出状态
    void sendMNSC(const AnalogOutputState &state);    // 模拟输出状态
    void sendMNDL(PORTINDEX index, double value);    // 单路模拟输出状态
    void sendCSYX(SWITCHSTATE state);   // 试运行
    void sendFSZT(SYSRUNSTATE state);   // 运行状态
    void sendSFZT(SWITCHSTATE state);   // 伺服状态
    void sendTSZT(SWITCHSTATE state);   // 调试状态
    void sendYXMS(SYSPLAYSTATE state);  // 运行模式
    void sendKZXT(SWITCHSTATE state);   // 控制系统

    void sendXGGJ(std::string frame);   // 修改工具坐标系
    void sendXGYH(std::string frame);   // 修改工件坐标系
    void sendTCPF(const JointsList& js);
    void sendTCFZ(const JointsList &js, const Joints &jo, const Joints &jz);
    void sendTCFX(const JointsList &js, const Joints &jo, const Joints &jx, const Joints &jz);
    void sendUSRF(const Terminal& to, const Terminal& tx, const Terminal& ty);

    void sendGJDD(JOINTINDEX index, MOVEDIRECTION dir, double vel);  // 关节点动
    void sendMDDD(TERMINALINDEX index, MOVEDIRECTION dir, double vel, COORDINATESYSTEM frame);    // 末端点动
    void sendDDTZ();    // 点动停止
    void sendGJBJ(JOINTINDEX index, MOVEDIRECTION dir, double step, double vel);  // 关节步进
    void sendMDBJ(TERMINALINDEX index, MOVEDIRECTION dir, double step, double vel, COORDINATESYSTEM frame);    // 末端步进
    void sendGJYX(MotionNode& node);    // 轨迹运行
    void sendMLYX(const ProgramPointer &pointer);    // 程序运行
    void sendMLZT();    // 程序暂停
    void sendMLJX();    // 程序继续
    void sendMLTZ();    // 程序停止
    void sendDBYX();    // 程序单步运行
    void sendDBZT();    // 程序单步暂停
    void sendCXXH(const ProgramPointer &pointer);   // 程序序号

    void sendKSTD();    // 开始拖动
    void sendJSTD();    // 结束拖动
    void sendJLTD(int period_ms, int timeLength_s); // 记录拖动
    void sendZXZB(double vel);  // 再现准备
    void sendZXTD(int period_ms);   // 再现拖动

    void sendTEST();

    void nullApi();         //编译专用借口

    void sendSZLX(int n);    // 开始拖动
    void sendBEEP(double time);    // 开启蜂鸣器
    void sendSZYY(ERROR_INIF_LANGUAGE time);    // 设置语言类型
private:
    void initParameter();
    enum FILE_TYPE_VALUE {CSWJ, PZWJ, ZBWJ};
    enum COMMAND_TYPE_VALUE {CWXX, JGXX, TSXX,
                             WCIO, WCZL, CXSD, CXXH, GJMD, TDDL, SJWZ,
                             IOZT, SRZT, SCZT, MNSR, MNSC, MNDL, CSYX, FSZT, SFZT, TSZT, YXMS, KZXT,
                             XGGJ, XGYH, TCPF, USRF, HOME};

    virtual void excuteFile(int fd, TCPSOURCETYPE src, std::string command);
    void commandCSWJ(int fd, TCPSOURCETYPE src);    // 机器人参数文件
    void commandPZWJ(int fd, TCPSOURCETYPE src);    // 机器人配置文件
    void commandZBWJ(int fd, TCPSOURCETYPE src);    // 机器人坐标系文件

    virtual void excuteCommand(int fd, TCPSOURCETYPE src, std::string command, std::string content);
    void commandCWXX(int fd, TCPSOURCETYPE src, std::string content);   // 错误信息
    void commandJGXX(int fd, TCPSOURCETYPE src, std::string content);   // 警告信息
    void commandTSXX(int fd, TCPSOURCETYPE src, std::string content);   // 提示信息

    void commandWCIO(int fd, TCPSOURCETYPE src, std::string content);   // 完成IO操作
    void commandWCZL(int fd, TCPSOURCETYPE src, std::string content);   // 完成指令操作
    void commandCXXH(int fd, TCPSOURCETYPE src, std::string content);   // 程序序号
    void commandGJMD(int fd, TCPSOURCETYPE src, std::string content);   // 关节末端数据
    void commandTDDL(int fd, TCPSOURCETYPE src, std::string content);   // 拖动点列
    void commandSJWZ(int fd, TCPSOURCETYPE src, std::string content);   // 视觉定位，相机序号+目标位姿

    void commandIOZT(int fd, TCPSOURCETYPE src, std::string content);   // IO连接状态
    void commandSRZT(int fd, TCPSOURCETYPE src, std::string content);   // 数字口输入状态
    void commandSCZT(int fd, TCPSOURCETYPE src, std::string content);   // 数字口输出状态
    void commandMNSR(int fd, TCPSOURCETYPE src, std::string content);   // 模拟口输入状态
    void commandMNSC(int fd, TCPSOURCETYPE src, std::string content);   // 模拟口输出状态
    void commandCSYX(int fd, TCPSOURCETYPE src, std::string content);   // 试运行
    void commandFSZT(int fd, TCPSOURCETYPE src, std::string content);   // 发送状态
    void commandSFZT(int fd, TCPSOURCETYPE src, std::string content);   // 伺服状态
    void commandTSZT(int fd, TCPSOURCETYPE src, std::string content);   // 调试状态
    void commandYXMS(int fd, TCPSOURCETYPE src, std::string content);   // 运行模式
    void commandKZXT(int fd, TCPSOURCETYPE src, std::string content);   // 控制状态
    void commandCXSD(int fd, TCPSOURCETYPE src, std::string content);   // 全局速度

    void commandXGGJ(int fd, TCPSOURCETYPE src, std::string content);
    void commandXGYH(int fd, TCPSOURCETYPE src, std::string content);
    void commandTCPF(int fd, TCPSOURCETYPE src, std::string content);
    void commandUSRF(int fd, TCPSOURCETYPE src, std::string content);
    void commandHOME(int fd, TCPSOURCETYPE src, std::string content);

private:
    std::map<std::string, FILE_TYPE_VALUE> m_fileType;
    std::map<std::string, COMMAND_TYPE_VALUE> m_commandType;

    int m_cmdRightFlag; // 指令是否正确标志位，-1:执行中 0:正确 >1:出错
    int m_cmdFinishFlag;    // 指令是否完成标志位，-1:执行中 0:正确 >1:出错

    RobotParameter m_robotParameter;    // 机器人参数
    RobotState m_robotState;    // 机器人状态
    RobotPreference m_robotPreference;  // 机器人配置
    RobotMotion m_robotMotion;  // 机器人运动状态
    RobotIO m_robotIO;  // 机器人外部接口状态
    RobotFrame m_robotFrame;    // 机器人坐标系
    RobotInfo m_robotInfo;  // 机器人消息
    SWITCHSTATE m_controlFlag;  // 机器人控制状态

    Terminal m_visionLocation[3];   // 视觉位姿记录
    std::list<Joints> m_dragPointList;  // 拖动点列

    ProgramPointer m_programPointer;    // 程序指针

    double m_calibrateToolFramePre; // 标定精度
    Terminal m_calibrateToolFrame;  // 标定的工具坐标
    Terminal m_calibrateWorkFrame;  // 标定的工件坐标

    bool    m_bSetFileUpdate;       //设置文件更新
    Mutex    m_robotMotionMutex;  // 机器人运动状态锁
};

}

#endif
