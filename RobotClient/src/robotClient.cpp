#include "robotClient.h"

using namespace std;
using namespace rclib;

RobotClient* RobotClient::m_robotClient = new RobotClient;

RobotClient::RobotClient(){

}

RobotClient* RobotClient::initance(){
    return m_robotClient;
}

void RobotClient::delInittance()
{
    if(m_robotClient)
    {
        delete m_robotClient;
        m_robotClient = NULL;
    }
}

RobotClient::~RobotClient(){

}

bool RobotClient::initSystem(const char* server_ip, int server_port){
    return initSystemBase(server_ip, server_port);
}

bool RobotClient::controlSystem(){
    sendKZXT(SWITCHON);
    sleep_ms(200);

    return SWITCHON ==getControlState();
}

void RobotClient::setErrorClear(){
    sendQCCW();
}

bool RobotClient::setJointZero(JOINTINDEX joint){
    setCMDRightFlag(-1);
    sendHLJZ(joint);
    int ret = waitCMDRightFlag();
    if(ret){
        return false;
    }
    else{
        return true;
    }
}

void RobotClient::testFunctionButton(){
    sendTEST();
}

bool RobotClient::programVelocity(double vel){
    sendCXSD(vel);
	return true;
}

void RobotClient::programLoad(const char* path){
    sendMLWJ(path);
}

void RobotClient::programRun(){
    sendMLYX(m_breakPointer);
}

void RobotClient::programPause(bool bPause){
    if(bPause)
    {
        sendMLZT();
    }else
    {
        sendMLJX();
    }
}

void RobotClient::programStop(){
    sendMLTZ();
}

void RobotClient::programStepRun(){
    sendDBYX();
}

void RobotClient::programStepPause(){
    sendDBZT();
}

void RobotClient::setProgramPointer(const ProgramPointer &pointer){
    sendCXXH(pointer);
}
