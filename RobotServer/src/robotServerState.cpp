#include "robotServer.h"
#include "robotStructure.h"
#include "robotFile.h"

using namespace std;
using namespace rclib;

void RobotServer::updateRobotClient(){
    sendCSWJ((m_strPath + ROBOT_PARAMETER_PATH).c_str());

    sendPZWJ((m_strPath + ROBOT_PREFERENCE_PATH).c_str());

    sendZBWJ((m_strPath + ROBOT_FRAME_PATH).c_str());
    sendXGGJ(m_robotFrame.getCurrentToolFrame());
    sendXGYH(m_robotFrame.getCurrentWorkFrame());

    //updateRobotStateMotionIO();
}

void RobotServer::setRobotParameter(const RobotParameter& robot){
    if(!m_robotParameter.isDriverSame(robot)){
        printDriverParameterChanged();
    }

    m_robotParameter = robot;
    m_robot.setRobotParameter(m_robotParameter);

    setDof(m_robotParameter.getWholeDOF());

    m_robotParameter.writeRobotParameter((m_strPath + ROBOT_PARAMETER_PATH).c_str());
    sendCSWJ((m_strPath + ROBOT_PARAMETER_PATH).c_str());
}

RobotParameter RobotServer::getRobotParameter() const{
    return m_robotParameter;
}


void RobotServer::setRobotPreference(const RobotPreference &rp){
    m_robotPreference = rp;

    m_robotPreference.writeRobotPreference((m_strPath + ROBOT_PREFERENCE_PATH).c_str());
    sendPZWJ((m_strPath + ROBOT_PREFERENCE_PATH).c_str());
}

RobotPreference RobotServer::getRobotPreference() const{
    return m_robotPreference;
}


int RobotServer::modifyToolFrame(string name){
    setCMDRightFlag(-1);

    int ret = m_robotFrame.setCurrentToolFrame(name);
    if(ret == 0){
        m_robotParameter.setToolFrame(m_robotFrame.getToolFrame(name));
        m_robot.setRobotParameter(m_robotParameter);
        m_robotFrame.writeRobotFrame((m_strPath + ROBOT_FRAME_PATH).c_str());
        sendXGGJ(m_robotFrame.getCurrentToolFrame());
    }

    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::modifyWorkFrame(string name){
    setCMDRightFlag(-1);

    int ret = m_robotFrame.setCurrentWorkFrame(name);
    if(ret == 0){
        m_robotParameter.setWorkFrame(m_robotFrame.getWorkFrame(name));
        m_robot.setRobotParameter(m_robotParameter);
        m_robotFrame.writeRobotFrame((m_strPath + ROBOT_FRAME_PATH).c_str());
        sendXGYH(m_robotFrame.getCurrentWorkFrame());
    }

    setCMDRightFlag(ret);
    return ret;
}

void RobotServer::setRobotFrame(const RobotFrame &rf){
    m_robotFrame = rf;

    m_robotFrame.writeRobotFrame((m_strPath + ROBOT_FRAME_PATH).c_str());
    sendZBWJ((m_strPath + ROBOT_FRAME_PATH).c_str());
}

RobotFrame RobotServer::getRobotFrame() const{
    return m_robotFrame;
}


void RobotServer::setDigitalOutput(DigitalOutputState state){
    setDigitalOutputState(state);
}

void RobotServer::setDigitalOutput(PORTINDEX index, SWITCHSTATE state){
    setDigitalOutputState(index, state);
}

void RobotServer::setAnalogOutput(AnalogOutputState state){
    setAnalogOutputState(state);
}

void RobotServer::setAnalogOutput(PORTINDEX index, double state){
    setAnalogOutputState(index, state);
}

RobotIO RobotServer::getRobotIO() const{
    return m_robotIO;
}

SWITCHSTATE RobotServer::getIOConnect() const{
    return m_robotIO.getIOConnectState();
}

DigitalInputState RobotServer::getDigitalInput() const{
    return m_robotIO.getDigitalInputState();
}

SWITCHSTATE RobotServer::getDigitalInput(PORTINDEX index) const{
    return m_robotIO.getDigitalInputState()[index];
}

DigitalOutputState RobotServer::getDigitalOutput() const{
    return m_robotIO.getDigitalOutputState();
}

SWITCHSTATE RobotServer::getDigitalOutput(PORTINDEX index) const{
    return m_robotIO.getDigitalOutputState()[index];
}

AnalogInputState RobotServer::getAnalogInput() const{
    return m_robotIO.getAnalogInputState();
}

double RobotServer::getAnalogInput(PORTINDEX index) const{
    return m_robotIO.getAnalogInputState()[index];
}

AnalogOutputState RobotServer::getAnalogOutput() const{
    return m_robotIO.getAnalogOutputState();
}

double RobotServer::getAnalogOutput(PORTINDEX index) const{
    return m_robotIO.getAnalogOutputState()[index];
}


void RobotServer::setTryRunState(SWITCHSTATE state){
    m_robotState.setVirtualState(state);
}

void RobotServer::setDebugState(SWITCHSTATE state){
    m_robotState.setDebugState(state);
    RobotInterpreter::setDebugState(state);
}

void RobotServer::setPlayState(SYSPLAYSTATE state){
    m_robotState.setPlayState(state);
    //sendYXMS(m_robotState.getPlayState());
    setPlayStateLight(state);

    switch(state){
    case SYSPLAY_TEACH:
        m_robotState.setVel(0.05);
        break;
    case SYSPLAY_PLAY:
        m_robotState.setVel(0.05);
        break;
    case SYSPLAY_REMOTE:
        m_robotState.setVel(1);
        break;
    default:
        break;
    }
}

void RobotServer::setServoState(SWITCHSTATE state){
    m_robotState.setServoState(state);
    setServo(state);
    sleep_ms(5);
    setErrorClear();
}

void RobotServer::setRunState(SYSRUNSTATE state){
    m_robotState.setRunState(state);
    setRunStateLight(state);
}

RobotState RobotServer::getRobotState() const{
    return m_robotState;
}

ProgramPointer RobotServer::getProgramPointer(){
    return m_programPointer;
}

void RobotServer::getInfo(INFOSTATE &infoType, std::string &infoString){
    m_robotInfo.getInfo(infoType, infoString);
}

void RobotServer::setTeachType(int n)
{
    setTeach(TEACH_TYPE(n));
}

void RobotServer::setBeep(double time)
{
    //set_beep_time(time);
    m_fBeepTime = time;
}

void RobotServer::setSZYY(int n)
{
    setErrorInfoLanguage(ERROR_INIF_LANGUAGE(n));
    languageType(ERROR_INIF_LANGUAGE(n));
}

void RobotServer::printInfo(INFOSTATE infoType, std::string infoString){
    m_robotInfo.setInfo(infoType, infoString);
    switch (infoType) {
    case INFO_ERROR:
        sendCWXX(infoString);
        break;
    case INFO_WARNING:
        sendJGXX(infoString);
        break;
    case INFO_RECOM:
        sendTSXX(infoString);
        break;
    default:
        break;
    }
}

void RobotServer::updateRemoteState(REMOTESTATE state){
    if(m_robotState.getPlayState() == SYSPLAY_REMOTE){
        if(state){
            m_remoteState = REMOTESTATE(state);
        }
    }
}

