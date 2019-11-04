#ifndef ROBOTSTRUCTURE_H
#define ROBOTSTRUCTURE_H

#include "RobSoft/CDataStructure.hpp"
#include "RobSoft/CRobotParameter.hpp"
#include "RobSoft/CRobotics.hpp"
#include "systemLib.h"

#include "robotFile.h"
#include <vector>
#include <string>
#include <map>
#include <list>

#include "json.h"
#include "json-forwards.h"
#include "jsonWrapper.h"

namespace rclib{

enum ERROR_INIF_LANGUAGE{
    CHINESE,
    ENGLISH

};
using namespace robsoft;

#define DIGITAL_INPUTPORT_NUM 16
#define DIGITAL_OUTPUTPORT_NUM 16
#define ANALOG_INPUT_NUM 4
#define ANALOG_OUTPUT_NUM 4

enum PORTINDEX {
    PORT_1, PORT_2, PORT_3, PORT_4, PORT_5, PORT_6, PORT_7, PORT_8,
    PORT_9, PORT_10, PORT_11, PORT_12, PORT_13, PORT_14, PORT_15, PORT_16
};

enum LOCATEVISIONINDEX {
    LOCATEVISION_1, LOCATEVISION_2, LOCATEVISION_3
};

enum SWITCHSTATE {SWITCHOFF, SWITCHON};

enum MOVEDIRECTION {MOVE_NEGATIVE, MOVE_POSITIVE};

enum MOTIONTYPE {
    MOTION_MOVEABSJ, MOTION_MOVEABSJR, MOTION_MOVEABSJ_TIME, MOTION_MOVEABSJR_TIME,
    MOTION_MOVEJ, MOTION_MOVEJR,
    MOTION_MOVEL, MOTION_MOVELR, MOTION_MOVEC, MOTION_MOVECR,
    MOTION_MOVEB, MOTION_MOVEJUMP,
    MOTION_MOVEL_CON, MOTION_MOVELR_CON, MOTION_MOVEC_CON, MOTION_MOVECR_CON, MOTION_START_CON, MOTION_END_CON
};

struct DECLSPEC_DLLEXPORT DigitalInputState{
    void init();
    int size();

    SWITCHSTATE& operator[](PORTINDEX index);
    const SWITCHSTATE& operator[](PORTINDEX index) const;

    bool operator ==(const DigitalInputState &state) const;
    bool operator !=(const DigitalInputState &state) const;

    SWITCHSTATE m_state[DIGITAL_INPUTPORT_NUM];
};

struct DECLSPEC_DLLEXPORT DigitalOutputState{
    void init();
    int size();

    SWITCHSTATE& operator[](PORTINDEX index);
    const SWITCHSTATE& operator[](PORTINDEX index) const;

    bool operator ==(const DigitalOutputState &state) const;
    bool operator !=(const DigitalOutputState &state) const;

    SWITCHSTATE m_state[DIGITAL_OUTPUTPORT_NUM];
};

struct DECLSPEC_DLLEXPORT AnalogInputState{
    void init();
    int size();

    double& operator[](PORTINDEX index);
    const double& operator[](PORTINDEX index) const;

    double m_state[ANALOG_INPUT_NUM];
};

struct DECLSPEC_DLLEXPORT AnalogOutputState{
    void init();
    int size();

    double& operator[](PORTINDEX index);
    const double& operator[](PORTINDEX index) const;

    double m_state[ANALOG_OUTPUT_NUM];
};

MY_CLASS RobotIO{
public:
    RobotIO();
    ~RobotIO();

    SWITCHSTATE getIOConnectState() const;
    void setIOConnectState(SWITCHSTATE state);

    DigitalInputState getDigitalInputState() const;
    SWITCHSTATE getDigitalInputState(PORTINDEX index) const;
    void setDigitalInputState(const DigitalInputState& state);

    DigitalOutputState getDigitalOutputState() const;
    SWITCHSTATE getDigitalOutputState(PORTINDEX index) const;
    void setDigitalOutputState(const DigitalOutputState& state);

    AnalogInputState getAnalogInputState() const;
    double getAnalogInputState(PORTINDEX index) const;
    void setAnalogInputState(const AnalogInputState& state);

    AnalogOutputState getAnalogOutputState() const;
    double getAnalogOutputState(PORTINDEX index) const;
    void setAnalogOutputState(const AnalogOutputState& state);

private:
    SWITCHSTATE m_IOState; //io连接状态
    DigitalInputState m_DIState; // 数字输入状态
    DigitalOutputState m_DOState;    //数字输出状态
    AnalogInputState m_AIState; // 模拟输入状态
    AnalogOutputState m_AOState;    // 模拟输出状态
};

enum SYSRUNSTATE {SYSRUN_STOP, SYSRUN_PAUSE, SYSRUN_RUN};
enum SYSPLAYSTATE {SYSPLAY_TEACH, SYSPLAY_PLAY, SYSPLAY_REMOTE};
enum REMOTESTATE{REMOTE_NONE = 0, REMOTE_RETURN = 1, REMOTE_PAUSE = 2, REMOTE_RUN = 4};

MY_CLASS RobotState{
public:
    RobotState();
    ~RobotState();

    SYSRUNSTATE getRunState() const;
    void setRunState(SYSRUNSTATE state) ;

    SYSPLAYSTATE getPlayState() const;
    void setPlayState(SYSPLAYSTATE state);

    SWITCHSTATE getServoState() const;
    void setServoState(SWITCHSTATE state);

    SWITCHSTATE getVirtualState() const;
    void setVirtualState(SWITCHSTATE state);

    SWITCHSTATE getDebugState() const;
    void setDebugState(SWITCHSTATE state);

    double getVel() const;
    void setVel(double vel);

private:
    SYSRUNSTATE m_runStateFlag;         //运行状态
    SYSPLAYSTATE m_playStateFlag;       //运行模式
    SWITCHSTATE m_servoStateFlag;       //伺服状态
    SWITCHSTATE m_virtualStateFlag;     //虚拟模式
    SWITCHSTATE m_debugStateFlag;       //调试模式
    double m_vel;   // 机器人速度
};

enum INFOSTATE {INFO_NONE, INFO_ERROR, INFO_WARNING, INFO_RECOM};

MY_CLASS RobotInfo{
public:
    RobotInfo();
    ~RobotInfo();

    void getInfo(INFOSTATE &infoType, std::string &infoString);
    void setInfo(INFOSTATE infoType, std::string infoString);

private:
    std::list<INFOSTATE> m_infoType;               //消息类型
    std::list<std::string> m_infoString;           //消息内容
    Mutex m_mutexInfo;          //消息锁
};

MY_CLASS RobotFrame{
public:
    RobotFrame();
    ~RobotFrame();
    void readRobotFrame(const char* path);
    void writeRobotFrame(const char* path);

    std::map<std::string, Terminal> getToolFrameList() const;
    std::string getCurrentToolFrame() const;
    Terminal getToolFrame(std::string name);
    int addToolFrame(std::string name, const Terminal& frame);
    int modifyToolFrame(std::string name, const Terminal& frame);
    int deleteToolFrame(std::string name);
    void clearToolFrame();
    int setCurrentToolFrame(std::string name);
    void setDefaultToolFrame();

    std::map<std::string, Terminal> getWorkFrameList() const;
    std::string getCurrentWorkFrame() const;
    Terminal getWorkFrame(std::string name);
    int addWorkFrame(std::string name, const Terminal& frame);
    int modifyWorkFrame(std::string name, const Terminal& frame);
    int deleteWorkFrame(std::string name);
    void clearWorkFrame();
    int setCurrentWorkFrame(std::string name);
    void setDefaultWorkFrame();

    bool operator ==(RobotFrame frame);
    bool operator !=(RobotFrame frame);

private:
    std::string m_currentToolFrame;
    std::string m_currentWorkFrame;
    std::map<std::string, Terminal> m_toolFrameList;
    std::map<std::string, Terminal> m_workFrameList;

    std::string m_defaultName = "Default";
};

MY_CLASS RobotPreference{
public:
    RobotPreference();
    ~RobotPreference();
    void readRobotPreference(const char* path);
    void writeRobotPreference(const char* path) const;

    Joints getInitJointPosition() const;
    void setInitJointPosition(const Joints& joint);

    Joints getJointCompensation() const;
    void setJointCompensation(const Joints& joint);

    Joints getJointReturnSequence() const;
    void setJointReturnSequence(const Joints& joint);

    bool operator ==(const RobotPreference &prefer) const;
    bool operator !=(const RobotPreference &prefer) const;

private:
    Joints m_initJointPosition;
    Joints m_jointCompensation;
    Joints m_jointReturnSequence;
};

MY_CLASS MotionNode{
public:
    MotionNode();
    ~MotionNode();

    MOTIONTYPE motionType;
    double accRatio;
    double jerkRatio;
    double angleRatio;
    double precision;   // 连续轨迹转弯半径，B样条精度
    COORDINATESYSTEM referFrame;     // 坐标系 0:base   1:user
    CIRCLETYPE circleType;     // 曲线类型类型 0:whole/open  1:part/close
    double height;      // 门运动高度
    int delayNum;       // 连续轨迹延迟段数
    std::vector<double> velList;
    JointsList jointList;
    TerminalList terminalList;
};

DECLSPEC_DLLEXPORT std::string num_to_string(double num);
DECLSPEC_DLLEXPORT std::string num_to_string(int num);
DECLSPEC_DLLEXPORT double string_to_double(const std::string& str);
DECLSPEC_DLLEXPORT int string_to_int(const std::string& str);

}

#endif
