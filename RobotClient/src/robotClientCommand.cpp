#include"robotClientBase.h"
#include "robotFile.h"
#include "robotStructure.h"
#include <string>
#include <sstream>
#include "functions.h"
#define SEGCHAR string("|")

using namespace std;
using namespace rclib;

static double get_next_number(string& content){
    int index = content.find(SEGCHAR);
    if(index < 0){
        throw string("Error: no more number to get");
        cout << "Error: no more number to get" << endl;
    }
    double num = string_to_double(content.substr(0, index));
    content = content.substr(index + 1);
    return num;
}

void RobotClientBase::excuteFile(int fd, TCPSOURCETYPE src, string command) {
    switch (m_fileType[command]) {
    case CSWJ: commandCSWJ(fd, src); break;
    case PZWJ: commandPZWJ(fd, src); break;
    case ZBWJ: commandZBWJ(fd, src); break;
    default: break;
    }
    m_bSetFileUpdate = true;
}

void RobotClientBase::commandCSWJ(int fd, TCPSOURCETYPE src){
    m_robotParameter.readRobotParameter((m_strPath+ROBOT_TEMP_RECV_FILE).c_str());
}

void RobotClientBase::commandPZWJ(int fd, TCPSOURCETYPE src){
    m_robotPreference.readRobotPreference((m_strPath+ROBOT_TEMP_RECV_FILE).c_str());
}

void RobotClientBase::commandZBWJ(int fd, TCPSOURCETYPE src){
    m_robotFrame.readRobotFrame((m_strPath+ROBOT_TEMP_RECV_FILE).c_str());
}

void RobotClientBase::excuteCommand(int fd, TCPSOURCETYPE src, string command, string content) {
    switch (m_commandType[command]) {
    case CWXX: commandCWXX(fd, src, content); break;
    case JGXX: commandJGXX(fd, src, content); break;
    case TSXX: commandTSXX(fd, src, content); break;
    case WCIO: commandWCIO(fd, src, content); break;
    case WCZL: commandWCZL(fd, src, content); break;
    case CXXH: commandCXXH(fd, src, content); break;
    case GJMD: commandGJMD(fd, src, content); break;
    case TDDL: commandTDDL(fd, src, content); break;
    case SJWZ: commandSJWZ(fd, src, content); break;
    case IOZT: commandIOZT(fd, src, content); break;
    case SRZT: commandSRZT(fd, src, content); break;
    case SCZT: commandSCZT(fd, src, content); break;
    case MNSR: commandMNSR(fd, src, content); break;
    case MNSC: commandMNSC(fd, src, content); break;
    case CSYX: commandCSYX(fd, src, content); break;
    case FSZT: commandFSZT(fd, src, content); break;
    case SFZT: commandSFZT(fd, src, content); break;
    case TSZT: commandTSZT(fd, src, content); break;
    case YXMS: commandYXMS(fd, src, content); break;
    case KZXT: commandKZXT(fd, src, content); break;
    case CXSD: commandCXSD(fd, src, content); break;
    case XGGJ: commandXGGJ(fd, src, content); break;
    case XGYH: commandXGYH(fd, src, content); break;
    case TCPF: commandTCPF(fd, src, content); break;
    case USRF: commandUSRF(fd, src, content); break;
    case HOME: commandHOME(fd, src, content); break;
    default: break;
    }
}

void RobotClientBase::commandCWXX(int fd, TCPSOURCETYPE src, std::string content){
    printInfo(INFO_WARNING, content);
}

void RobotClientBase::commandJGXX(int fd, TCPSOURCETYPE src, std::string content){
    printInfo(INFO_WARNING, content);
}

void RobotClientBase::commandTSXX(int fd, TCPSOURCETYPE src, std::string content){
    printInfo(INFO_RECOM, content);
}

void RobotClientBase::commandWCIO(int fd, TCPSOURCETYPE src, std::string content){
    setCMDRightFlag(get_next_number(content));
}

void RobotClientBase::commandWCZL(int fd, TCPSOURCETYPE src, std::string content){
    setCMDFinishFlag(get_next_number(content));
}

void RobotClientBase::commandCXXH(int fd, TCPSOURCETYPE src, std::string content){
    m_programPointer.nFunction = get_next_number(content);
    m_programPointer.nSentence = get_next_number(content);
}

void RobotClientBase::commandGJMD(int fd, TCPSOURCETYPE src, std::string content){
    m_robotMotionMutex.lockMutex();
    Joints joint(m_robotParameter.getWholeDOF());
    for(int i=0; i<joint.getJointsDOF(); i++){
        joint[JOINTINDEX(i)] = get_next_number(content);
    }
    m_robotMotion.setCurrentJointPosition(joint);
    for(int i=0; i<joint.getJointsDOF(); i++){
        joint[JOINTINDEX(i)] = get_next_number(content);
    }
    m_robotMotion.setCurrentJointVelocity(joint);
    for(int i=0; i<joint.getJointsDOF(); i++){
        joint[JOINTINDEX(i)] = get_next_number(content);
    }
    m_robotMotion.setCurrentJointAcceleration(joint);
    for(int i=0; i<joint.getJointsDOF(); i++){
        joint[JOINTINDEX(i)] = get_next_number(content);
    }
    m_robotMotion.setCurrentJointTorque(joint);
    Terminal terminal;
    for(int i=0; i<6; i++){
        terminal[TERMINALINDEX(i)] = get_next_number(content);
    }
    m_robotMotion.setCurrentTerminal(terminal);
    for(int i=0; i<6; i++){
        terminal[TERMINALINDEX(i)] = get_next_number(content);
    }
    m_robotMotion.setCurrentWorkTerminal(terminal);
    m_robotMotionMutex.unlockMutex();
}

void RobotClientBase::commandTDDL(int fd, TCPSOURCETYPE src, std::string content){
    m_dragPointList.clear();
    while(!content.empty()){
        Joints joint(m_robotParameter.getWholeDOF());
        for(int i=0; i<joint.getJointsDOF(); i++){
            joint[JOINTINDEX(i)] = get_next_number(content);
        }
        m_dragPointList.push_back(joint);
    }
}

void RobotClientBase::commandSJWZ(int fd, TCPSOURCETYPE src, std::string content){
    int index = get_next_number(content);
    Terminal terminal;
    for(int i=0; i<6; i++){
        terminal[TERMINALINDEX(i)] = get_next_number(content);
    }
    m_visionLocation[index] = terminal;
}


void RobotClientBase::commandIOZT(int fd, TCPSOURCETYPE src, std::string content){
    m_robotIO.setIOConnectState(SWITCHSTATE((int)get_next_number(content)));
}

void RobotClientBase::commandSRZT(int fd, TCPSOURCETYPE src, std::string content){
    DigitalInputState state;
    for(int i=0; i<DIGITAL_INPUTPORT_NUM; i++){
        state[PORTINDEX(i)] = SWITCHSTATE((int)get_next_number(content));
    }
    m_robotIO.setDigitalInputState(state);
}

void RobotClientBase::commandSCZT(int fd, TCPSOURCETYPE src, std::string content){
    DigitalOutputState state;
    for(int i=0; i<DIGITAL_OUTPUTPORT_NUM; i++){
        state[PORTINDEX(i)] = SWITCHSTATE((int)get_next_number(content));
    }
    m_robotIO.setDigitalOutputState(state);
}

void RobotClientBase::commandMNSR(int fd, TCPSOURCETYPE src, std::string content){
    AnalogInputState state;
    for(int i=0; i<ANALOG_INPUT_NUM; i++){
        state[PORTINDEX(i)] = get_next_number(content);
    }
    m_robotIO.setAnalogInputState(state);
}

void RobotClientBase::commandMNSC(int fd, TCPSOURCETYPE src, std::string content){
    AnalogOutputState state;
    for(int i=0; i<ANALOG_OUTPUT_NUM; i++){
        state[PORTINDEX(i)] = get_next_number(content);
    }
    m_robotIO.setAnalogOutputState(state);
}

void RobotClientBase::commandCSYX(int fd, TCPSOURCETYPE src, std::string content){
    m_robotState.setVirtualState(SWITCHSTATE((int)get_next_number(content)));
}

void RobotClientBase::commandFSZT(int fd, TCPSOURCETYPE src, std::string content){
    m_robotState.setRunState(SYSRUNSTATE((int)get_next_number(content)));
}

void RobotClientBase::commandSFZT(int fd, TCPSOURCETYPE src, std::string content){
    m_robotState.setServoState(SWITCHSTATE((int)get_next_number(content)));
}

void RobotClientBase::commandTSZT(int fd, TCPSOURCETYPE src, std::string content){
    m_robotState.setDebugState(SWITCHSTATE((int)get_next_number(content)));
}

void RobotClientBase::commandYXMS(int fd, TCPSOURCETYPE src, std::string content){
    m_robotState.setPlayState(SYSPLAYSTATE((int)get_next_number(content)));
}

void RobotClientBase::commandKZXT(int fd, TCPSOURCETYPE src, std::string content){
    m_controlFlag = SWITCHSTATE((int)get_next_number(content));
}

void RobotClientBase::commandCXSD(int fd, TCPSOURCETYPE src, std::string content){
    m_robotState.setVel(get_next_number(content));
}


void RobotClientBase::commandXGGJ(int fd, TCPSOURCETYPE src, std::string content){
    m_robotFrame.setCurrentToolFrame(content);
}

void RobotClientBase::commandXGYH(int fd, TCPSOURCETYPE src, std::string content){
    m_robotFrame.setCurrentWorkFrame(content);
}

void RobotClientBase::commandTCPF(int fd, TCPSOURCETYPE src, std::string content){
    Terminal terminal;
    for(int i=0; i<6; i++){
        terminal[TERMINALINDEX(i)] = get_next_number(content);
    }
    m_calibrateToolFramePre = get_next_number(content);
    m_calibrateToolFrame = terminal;
}

void RobotClientBase::commandUSRF(int fd, TCPSOURCETYPE src, std::string content){
    Terminal terminal;
    for(int i=0; i<6; i++){
        terminal[TERMINALINDEX(i)] = get_next_number(content);
    }
    m_calibrateWorkFrame = terminal;
}

void RobotClientBase::commandHOME(int fd, TCPSOURCETYPE src, string content)
{
    for(int i=0; i<8; i++){
        m_getHomeState[i] = SWITCHSTATE ((int)get_next_number(content));
    }
}
