#include "robotClientBase.h"
#include "robotStructure.h"

using namespace std;
using namespace rclib;

SWITCHSTATE RobotClientBase::getControlState() const{
    return m_controlFlag;
}

void RobotClientBase::setRobotParameter(const RobotParameter& robot){
    robot.writeRobotParameter((m_strPath+ ROBOT_TEMP_SEND_FILE).c_str());
    sendCSWJ((m_strPath + ROBOT_TEMP_SEND_FILE).c_str());
}

RobotParameter RobotClientBase::getRobotParameter() const{
    return m_robotParameter;
}


void RobotClientBase::setRobotPreference(const RobotPreference &rp){
    rp.writeRobotPreference((m_strPath+ ROBOT_TEMP_SEND_FILE).c_str());
    sendPZWJ((m_strPath + ROBOT_TEMP_SEND_FILE).c_str());
}

RobotPreference RobotClientBase::getRobotPreference() const{
    return m_robotPreference;
}


int RobotClientBase::modifyToolFrame(std::string name){
    setCMDRightFlag(-1);

    sendXGGJ(name);

    return waitCMDRightFlag();
}

int RobotClientBase::modifyWorkFrame(std::string name){
    setCMDRightFlag(-1);

    sendXGYH(name);

    return waitCMDRightFlag();
}

void RobotClientBase::setRobotFrame(RobotFrame &rf){
    rf.writeRobotFrame((m_strPath+ ROBOT_TEMP_SEND_FILE).c_str());
    sendZBWJ((m_strPath + ROBOT_TEMP_SEND_FILE).c_str());
}

RobotFrame RobotClientBase::getRobotFrame() const{
    return m_robotFrame;
}

void RobotClientBase::setDigitalOutput(DigitalOutputState state){
    sendSCZT(state);
}

void RobotClientBase::setDigitalOutput(PORTINDEX index, SWITCHSTATE state){
    DigitalOutputState dstate = m_robotIO.getDigitalOutputState();
    dstate[index] = state;
    sendSCZT(dstate);
}

void RobotClientBase::setAnalogOutput(AnalogOutputState state){
    sendMNSC(state);
}

void RobotClientBase::setAnalogOutput(PORTINDEX index, double state){
    sendMNDL(index,state);
}

RobotIO RobotClientBase::getRobotIO() const{
    return m_robotIO;
}

SWITCHSTATE RobotClientBase::getIOConnect() const{
    return m_robotIO.getIOConnectState();
}

DigitalInputState RobotClientBase::getDigitalInput() const{
    return m_robotIO.getDigitalInputState();
}

SWITCHSTATE RobotClientBase::getDigitalInput(PORTINDEX index) const{
    return m_robotIO.getDigitalInputState()[index];
}

DigitalOutputState RobotClientBase::getDigitalOutput() const{
    return m_robotIO.getDigitalOutputState();
}

SWITCHSTATE RobotClientBase::getDigitalOutput(PORTINDEX index) const{
    return m_robotIO.getDigitalOutputState()[index];
}

AnalogInputState RobotClientBase::getAnalogInput() const{
    return m_robotIO.getAnalogInputState();
}

double RobotClientBase::getAnalogInput(PORTINDEX index) const{
    return m_robotIO.getAnalogInputState()[index];
}

AnalogOutputState RobotClientBase::getAnalogOutput() const{
    return m_robotIO.getAnalogOutputState();
}

double RobotClientBase::getAnalogOutput(PORTINDEX index) const{
    return m_robotIO.getAnalogOutputState()[index];
}


void RobotClientBase::setTryRunState(SWITCHSTATE state){
    sendCSYX(state);
}

void RobotClientBase::setDebugState(SWITCHSTATE state){
    sendTSZT(state);
}

void RobotClientBase::setPlayState(SYSPLAYSTATE state){
    sendYXMS(state);
}

void RobotClientBase::setServoState(SWITCHSTATE state){
    sendSFZT(state);
}

void RobotClientBase::setRunState(SYSRUNSTATE state){
    sendFSZT(state);
}

RobotState RobotClientBase::getRobotState() const{
    return m_robotState;
}

RobotMotion RobotClientBase::getRobotMotion()
{
    m_robotMotionMutex.lockMutex();
    RobotMotion rm = m_robotMotion;
    m_robotMotionMutex.unlockMutex();
    return rm;
}

ProgramPointer RobotClientBase::getProgramPointer()
{
    return m_programPointer;
}

void RobotClientBase::updateProgramPointer(const ProgramPointer &pointer)
{
    m_programPointer = pointer;
}

bool RobotClientBase::bSetFileUpdate()
{
    bool bRt = m_bSetFileUpdate;
    m_bSetFileUpdate = false;

    return  bRt;
}



