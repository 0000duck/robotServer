#include "robotLink.h"
#include "robotFile.h"
#include "RobSoft/CFileIO.hpp"
#ifndef _WIN32
#include <libgen.h>
#endif

using namespace std;
using namespace rclib;

RobotLink* RobotLink::robotLink;

std::string RobotLink::GetModuleFullPath(bool bLastPath)
{
        char chPath[256] = { 0 };

#ifdef _WIN32
        ::GetModuleFileName(NULL, chPath, sizeof(chPath));
#else
        memset(chPath, 0, sizeof(chPath));
        ::readlink("/proc/self/exe", chPath, sizeof(chPath));
#endif
        std::string strModuleFileName = chPath;

        if (bLastPath == false)
        {
                return strModuleFileName.c_str();
        }

        //如果文件名称有生僻汉字并且第二字节是'\\'时，用find_last_of的方法就会有问题
        //比如："c:\\ABC癨.txt"就会返回"c:\\ABC\\"，而用_splitpath和_makepath正确返回"c:\\"
//	size_t nPos = strModuleFileName.find_last_of(PATH_SEP_STRING);
//	std::string strLastPath = strModuleFileName.substr(0, nPos);

#ifdef _WIN32
        char szDrive[_MAX_DRIVE];
        char szDir[_MAX_DIR];
        ::_splitpath(chPath, szDrive, szDir, NULL, NULL);
        ::_makepath(chPath, szDrive, szDir, NULL, NULL);
        //去除最后的'\\'
        if (chPath[strlen(chPath) - 1] == '\\')
        {
                chPath[strlen(chPath) - 1] = '\0';
        }

#else
        ::dirname(chPath);
#endif

        std::string strLastPath = chPath;
        return strLastPath.c_str();
}
//路径分隔符
#ifdef _WIN32
#define PATH_SEPARATOR      "\\"
#else
#define PATH_SEPARATOR     "/"
#endif

RobotLink::RobotLink()
        : m_fBeepTime(0)
{
    robotLink = this;
}

RobotLink::~RobotLink(){
    closeLink();
}

void RobotLink::initLink(double period, const Joints& encoderResolution, const Joints& reduceRatio, const Joints& rateTorque){
    m_period = period;
    m_encoderResolution = encoderResolution;
    m_reduceRatio = reduceRatio;
    m_rateTorque = rateTorque;

    m_errorCode.setValue(m_encoderResolution.getJointsDOF());

    m_unitPulse.setValue(m_encoderResolution.getJointsDOF());
    for(int i=0; i<m_encoderResolution.getJointsDOF(); i++){
        m_unitPulse[JOINTINDEX(i)] = m_reduceRatio[JOINTINDEX(i)] * m_encoderResolution[JOINTINDEX(i)] / 360;
    }

    startLink();
}

void RobotLink::startLink(){
    modbusInit();

    setServo(SWITCHOFF);

    // 开机读取储存的角度值
	string strPath = GetModuleFullPath(true)+ PATH_SEPARATOR"config";
	strPath += PATH_SEPARATOR;
    Joints joint = readJoints((strPath+ROBOT_JOINTS_PATH).c_str());
    setPulseNumberWithJoint(joint);
    joint = getJointWithPulseNumber();
    m_getRobotMotion.setCurrentJointPosition(joint);
    m_setRobotMotion.setCurrentJointPosition(joint);
    m_addRobotMotion.setCurrentJointPosition(joint);

    // 使得驱动器发送队列不为０
    for(int i=0; i<3; i++){
        setMotor(m_addRobotMotion);
    }

    int ret;
    if(ret = m_threadMotion.create(cycleTaskMotion, NULL)){
        printf("create link thread failed %d\n", ret);
        exit(-1);
    }
    if(ret = m_threadIO.create(cycleTaskIO, NULL)){
        printf("create link thread failed %d\n", ret);
        exit(-1);
    }
}

void RobotLink::closeLink(){
    modbusClose();
    m_threadMotion.cancel();
    m_threadIO.cancel();
    m_threadMotion.join();
    m_threadIO.join();
}

bool RobotLink::setDriverMode(DRIVER_MODE mode){
    m_setDriverMode = mode;
    return true;
}
DRIVER_MODE RobotLink::getDriveMode(){
    return m_getDriveMode;
}

void RobotLink::setCurrentJointPosition(const Joints &joint){
    do{
        sleep_ms(m_period*10000);
    }while(getRobotMotionListNum()>0);

    m_mutexRM.lockMutex();
    setPulseNumberWithJoint(joint);
    Joints jointTmp = getJointWithPulseNumber();
    m_setRobotMotion.setCurrentJointPosition(jointTmp);
    m_addRobotMotion.setCurrentJointPosition(jointTmp);
    m_mutexRM.unlockMutex();
}

bool RobotLink::setZero(JOINTINDEX index){
    do{
        sleep_ms(m_period*10000);
    }while(getRobotMotionListNum()>0);

    m_mutexRM.lockMutex();
    Joints joint= getJointWithPulseNumber();
    m_mutexRM.unlockMutex();
    if(index == JOINT_WHOLE){
        for(int i=0; i<m_encoderResolution.getJointsDOF(); i++){
            joint[JOINTINDEX(i)] = 0;
        }
    }
    else{
        joint[index] = 0;
    }
    setCurrentJointPosition(joint);
    return true;
}

void RobotLink::setJointCompensation(const Joints &joint){
    m_jointCompensation = joint;
}

void RobotLink::addMotor(const RobotMotion& rm){
    m_mutexRM.lockMutex();
    m_addRobotMotion = rm;
    m_robotMotionList.push_back(rm);
    m_mutexRM.unlockMutex();
}

int RobotLink::getRobotMotionListNum() const{
    robotLink->m_mutexRM.lockMutex();
    int n = m_robotMotionList.size();
    robotLink->m_mutexRM.unlockMutex();
    return n;
}

void RobotLink::setServo(SWITCHSTATE state){
    m_setServoState = state;
}

SWITCHSTATE RobotLink::getServo(){
    return m_getServoState;
}

int RobotLink::isError(Joints &jointError){
    jointError = m_errorCode;
    return m_errorFlag;
}

void RobotLink::clearError(){
    robotLink->m_errorFlag = SWITCHOFF;

    m_mutexRM.lockMutex();
    Joints jointTmp = getJointWithPulseNumber();
    m_setRobotMotion.setCurrentJointPosition(jointTmp);
    m_addRobotMotion.setCurrentJointPosition(jointTmp);
    m_mutexRM.unlockMutex();
}

SWITCHSTATE RobotLink::getHomeState(JOINTINDEX index){
    return SWITCHON;
}

void RobotLink::getHomeState()
{
    for (int i=0; i<8;i++) {
        m_getHomeState[i] = SWITCHON;
    }
}

void RobotLink::setHomeEnabled(JOINTINDEX index){

}

void RobotLink::recordRobotJoints(){
    m_mutexRM.lockMutex();
    Joints joint = m_getRobotMotion.getCurrentJointPosition();
    m_mutexRM.unlockMutex();

    string strPath = GetModuleFullPath(true)+ PATH_SEPARATOR"config";
	strPath += PATH_SEPARATOR;
    writeJoints((strPath+ROBOT_JOINTS_PATH).c_str(), joint);
}


RobotMotion RobotLink::getRobotMotion(){
    m_mutexRM.lockMutex();
    RobotMotion rm = m_getRobotMotion;
    m_mutexRM.unlockMutex();
    updateRobotTerminal(rm);
    return rm;
}

RobotMotion RobotLink::getLastRobotMotion(){
    RobotMotion rm = m_addRobotMotion;
    updateRobotTerminal(rm);
    return rm;
}

RobotIO RobotLink::getRobotIOState(){
    // 获取ＩＯ连接状态
    m_getRobotIO.setIOConnectState(SWITCHON);

    // 获取数字输入状态
    m_getRobotIO.setDigitalInputState(m_setRobotIO.getDigitalInputState());

    // 获取数字输出状态
    m_getRobotIO.setDigitalOutputState(m_setRobotIO.getDigitalOutputState());

    // 获取模拟输入状态
    m_getRobotIO.setAnalogInputState(m_setRobotIO.getAnalogInputState());

    // 获取模拟输出状态
    m_getRobotIO.setAnalogOutputState(m_setRobotIO.getAnalogOutputState());

    return m_getRobotIO;
}

void RobotLink::setRobotIOState(const RobotIO &rio){
    m_setRobotIO.setDigitalOutputState(rio.getDigitalOutputState());
    m_setRobotIO.setAnalogOutputState(rio.getAnalogOutputState());
}

SWITCHSTATE RobotLink::getIOConnectState(){
    return m_getRobotIO.getIOConnectState();
}

DigitalInputState RobotLink::getDigitalInputState(){
    return m_getRobotIO.getDigitalInputState();
}

SWITCHSTATE RobotLink::getDigitalInputState(PORTINDEX index){
    getDigitalInputState();
    return m_getRobotIO.getDigitalInputState()[index];
}

DigitalOutputState RobotLink::getDigitalOutputState(){
    return m_getRobotIO.getDigitalOutputState();
}

SWITCHSTATE RobotLink::getDigitalOutputState(PORTINDEX index){
    getDigitalOutputState();
    return m_getRobotIO.getDigitalOutputState()[index];
}

void RobotLink::setDigitalOutputState(const DigitalOutputState &state){
    m_setRobotIO.setDigitalOutputState(state);
}

void RobotLink::setDigitalOutputState(PORTINDEX index, SWITCHSTATE state){
    DigitalOutputState dstate = m_setRobotIO.getDigitalOutputState();
    dstate[index] = state;
    m_setRobotIO.setDigitalOutputState(dstate);
}

AnalogInputState RobotLink::getAnalogInputState(){
    return m_getRobotIO.getAnalogInputState();
}

double RobotLink::getAnalogInputState(PORTINDEX index){
    getAnalogInputState();
    return m_getRobotIO.getAnalogInputState()[index];
}

AnalogOutputState RobotLink::getAnalogOutputState(){
    return m_getRobotIO.getAnalogOutputState();
}

double RobotLink::getAnalogOutputState(PORTINDEX index){
    getAnalogOutputState();
    return m_getRobotIO.getAnalogOutputState()[index];
}

void RobotLink::setAnalogOutputState(const AnalogOutputState &state){
    m_setRobotIO.setAnalogOutputState(state);
}

void RobotLink::setAnalogOutputState(PORTINDEX index, double state){
    AnalogOutputState astate = m_setRobotIO.getAnalogOutputState();
    astate[index] = state;
    m_setRobotIO.setAnalogOutputState(astate);
}


void RobotLink::setRunStateLight(SYSRUNSTATE state){
    m_sysRunState = state;
}

void RobotLink::setPlayStateLight(SYSPLAYSTATE state){
    m_sysPlayState = state;
}

void RobotLink::setSystemStateLight(SWITCHSTATE state){
    m_SysState = state;
}

void RobotLink::setErrorStateLight(SWITCHSTATE state){
    m_SysErrorState = state;
}

void RobotLink::modbusInit(){

}

void RobotLink::modbusClose(){

}

void RobotLink::modbusSend(){

}

void RobotLink::modbusRecv(){

}

void RobotLink::setMotor(const RobotMotion& rm){
    uint32_t temp_pulse;
    m_mutexRM.lockMutex();
    Joints joint = m_setRobotMotion.getCurrentJointPosition();
    m_mutexRM.unlockMutex();
    for(int i=0; i<m_encoderResolution.getJointsDOF(); i++){
        double diff = rm.getCurrentJointPosition()[JOINTINDEX(i)]-joint[JOINTINDEX(i)];

        temp_pulse = fabs(diff)*m_unitPulse[JOINTINDEX(i)];

        double tempAngle = temp_pulse * 1.0 / m_unitPulse[JOINTINDEX(i)];
        if(diff >= 0){
            joint[JOINTINDEX(i)] = joint[JOINTINDEX(i)] + tempAngle;
        }
        else{
            joint[JOINTINDEX(i)] = joint[JOINTINDEX(i)] - tempAngle;
        }
    }
    m_mutexRM.lockMutex();
    m_setRobotMotion = rm;
    m_setRobotMotion.setCurrentJointPosition(joint);
    m_mutexRM.unlockMutex();
}

RobotMotion RobotLink::getMotor(){
    RobotMotion rm;
    rm.setCurrentJointPosition(getJointWithPulseNumber());
    return rm;
}

Joints RobotLink::readJoints(const char* path){
    Joints joint(m_encoderResolution.getJointsDOF());

    xmlDocPtr xmlDoc = xmlReadFile(path, "UTF-8", XML_PARSE_NOBLANKS);
    xmlNodePtr rootNode = xmlDocGetRootElement(xmlDoc);
    if(!rootNode){
        return Joints(m_encoderResolution.getJointsDOF());
    }

    xmlNodePtr ptrNode = rootNode->children;
    while(ptrNode != NULL){
        xml_judge_read_joints("m_currentJointPosition", ptrNode, joint);

        ptrNode = ptrNode->next;
    }

    xmlFreeDoc(xmlDoc);
    return joint;
}

void RobotLink::writeJoints(const char* path, const Joints &joint){
    xmlDocPtr xmlDoc = xmlNewDoc((const xmlChar*)"1.0");
    xmlNodePtr rootNode = xmlNewNode(0, BAD_CAST "robot_current_joint");
    xmlDocSetRootElement(xmlDoc, rootNode);

    xml_addchild_joints("m_currentJointPosition", joint, rootNode);

    xmlSaveFormatFileEnc(path, xmlDoc, "UTF-8", 1);
    xmlFreeDoc(xmlDoc);
}

void RobotLink::setPeriod(){

}

void RobotLink::setPulseNumberWithJoint(const Joints &joint){
    m_setRobotMotion.setCurrentJointPosition(joint);
}

Joints RobotLink::getJointWithPulseNumber(){
    return m_setRobotMotion.getCurrentJointPosition();
}

THREAD_ROUTINE_RETURN RobotLink::cycleTaskMotion(void* lpParameter){
    thread_detach();
#ifdef __linux__
	thread_name("RobotLink::cycleTaskMotion");
#endif // __linux__

    robotLink->m_setServoState = SWITCHOFF;
    robotLink->m_errorFlag = 0;

    robotLink->m_timerMotion.start(robotLink->m_period);
    while(1){
        // 更新驱动器运行模式

        // 获取驱动器运行模式
        robotLink->m_getDriveMode = DRIVER_ANGLE_MODE;

        //开启蜂鸣器
        if(robotLink->m_fBeepTime>0.000001)
        {
            set_beep_time(robotLink->m_fBeepTime);
            robotLink->m_fBeepTime=0;
        }
        // 更新伺服状态

        // 获取伺服状态
        robotLink->m_getServoState = robotLink->m_setServoState;

        // 更新电机运行状态
        robotLink->m_mutexRM.lockMutex();
        RobotMotion nextRobotMotion;
        if(!robotLink->m_robotMotionList.empty()){
            nextRobotMotion = robotLink->m_robotMotionList.front();
            robotLink->m_robotMotionList.pop_front();
        }
        else{
            nextRobotMotion = robotLink->m_addRobotMotion;
        }
        robotLink->m_mutexRM.unlockMutex();

        robotLink->setMotor(nextRobotMotion);
        // 获取电机运行状态
        robotLink->m_mutexRM.lockMutex();
        robotLink->m_getRobotMotion = robotLink->getMotor();
        robotLink->m_mutexRM.unlockMutex();

        // 获取错误码
        if(robotLink->m_getServoState == SWITCHOFF)
        {
            robotLink->m_errorFlag = 0x02;
        }else {
            robotLink->m_errorFlag = 0x00;
        }

        robotLink->updateMotionState(robotLink->m_getServoState, robotLink->m_getRobotMotion);
        robotLink->updateErrorState(robotLink->m_errorFlag, robotLink->m_errorCode);

        //更新传感器状态
        robotLink->getHomeState();

        robotLink->m_timerMotion.wait();
    }
    thread_exit();
}


THREAD_ROUTINE_RETURN RobotLink::cycleTaskIO(void *lpParameter){
    thread_detach();
#ifdef __linux__
    thread_name("RobotLink::cycleTaskIO");
#endif // 

    int recordFlag = 1;

    robotLink->m_timerIO.start(0.05);
    while(1){
        // 更新指示灯状态


        // 更新IO状态
        robotLink->setRobotIOState(robotLink->m_setRobotIO);
        // 获取IO状态
        robotLink->getRobotIOState();

        robotLink->updateIOState(robotLink->m_getRobotIO);

        if(recordFlag == 1){
            robotLink->recordRobotJoints();
            recordFlag = 2;
        }
        if(robotLink->m_sysRunState == SYSRUN_STOP){
            if(recordFlag == 0){
                recordFlag = 1;
            }
        }
        else{
            recordFlag = 0;
        }

        robotLink->m_timerIO.wait();
    }
    thread_exit();
}

namespace rclib {

void set_beep_time(double time){
    int n = time*10;
    while(n--)
    {
        printf("BEEP...\n");
        sleep_ms(10);
    }
}

void set_beep_state(bool state){
    printf("BEEP ON...\n");
}

}
