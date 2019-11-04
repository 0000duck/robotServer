#include "robotClientBase.h"
#include "robotFile.h"
#include <signal.h>

using namespace std;
using namespace rclib;

RobotClientBase::RobotClientBase()
            : m_controlFlag (SWITCHOFF)
            , m_bSetFileUpdate(false)
{
    initParameter();
}

RobotClientBase::~RobotClientBase(){

}

void RobotClientBase::initParameter(){
    m_fileType["CSWJ"] = CSWJ;
    m_fileType["PZWJ"] = PZWJ;
    m_fileType["ZBWJ"] = ZBWJ;

    m_commandType["CWXX"] = CWXX;
    m_commandType["JGXX"] = JGXX;
    m_commandType["TSXX"] = TSXX;

    m_commandType["WCIO"] = WCIO;
    m_commandType["WCZL"] = WCZL;
    m_commandType["CXSD"] = CXSD;
    m_commandType["CXXH"] = CXXH;
    m_commandType["GJMD"] = GJMD;
    m_commandType["TDDL"] = TDDL;
    m_commandType["SJWZ"] = SJWZ;

    m_commandType["IOZT"] = IOZT;
    m_commandType["SRZT"] = SRZT;
    m_commandType["SCZT"] = SCZT;
    m_commandType["MNSR"] = MNSR;
    m_commandType["MNSC"] = MNSC;
    m_commandType["MNDL"] = MNDL;
    m_commandType["CSYX"] = CSYX;
    m_commandType["FSZT"] = FSZT;
    m_commandType["SFZT"] = SFZT;
    m_commandType["TSZT"] = TSZT;
    m_commandType["YXMS"] = YXMS;
    m_commandType["KZXT"] = KZXT;

    m_commandType["XGGJ"] = XGGJ;
    m_commandType["XGYH"] = XGYH;
    m_commandType["TCPF"] = TCPF;
    m_commandType["USRF"] = USRF;
    m_commandType["HOME"] = HOME;
}
void handle_SIGPIPE(int sig)
{
#ifdef __linux__
    //不做任何处理即可
    string strPath = GetModuleFullPath(false);
    if(strPath.find("robotModelWindow")!=strPath.npos)
    {
        exit(0);
    }
#endif
}
bool RobotClientBase::initSystemBase(const char* server_ip, int server_port){
#ifdef __linux__
    struct sigaction action;
    action.sa_handler = handle_SIGPIPE;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGPIPE, &action, NULL);
#endif
    bool state = tcpClientInit(server_ip, server_port);
    if(state){
        sendJLLJ();
        sleep_ms(1000);

        sendKZXT(SWITCHON);
    }
    return state;
}

list<Joints> RobotClientBase::getDragPoint(){
    return m_dragPointList;
}

double RobotClientBase::calibrateTCP(const JointsList &js, Terminal& t){
    setCMDRightFlag(-1);

    sendTCPF(js);

    if(waitCMDRightFlag()){
        return -1;
    }

    t = m_calibrateToolFrame;
    return m_calibrateToolFramePre;
}

double RobotClientBase::calibrateTCFZ(const JointsList &js, const Joints &jo, const Joints &jz, Terminal &t){
    setCMDRightFlag(-1);

    sendTCFZ(js, jo, jz);

    if(waitCMDRightFlag()){
        return -1;
    }

    t = m_calibrateToolFrame;
    return m_calibrateToolFramePre;
}

double RobotClientBase::calibrateTCFX(const JointsList &js, const Joints &jo, const Joints &jx, const Joints &jz, Terminal &t){
    setCMDRightFlag(-1);

    sendTCFX(js, jo, jx, jz);

    if(waitCMDRightFlag()){
        return -1;
    }

    t = m_calibrateToolFrame;
    return m_calibrateToolFramePre;
}

void RobotClientBase::calibrateUSRF(const Terminal& to, const Terminal& tx, const Terminal& ty, Terminal &t){
    setCMDRightFlag(-1);

    sendUSRF(to, tx, ty);

    waitCMDRightFlag();

    t = m_calibrateWorkFrame;
}

Terminal RobotClientBase::getVisionLocation(LOCATEVISIONINDEX index){
    setCMDRightFlag(-1);

    sendSJXH(index);

    waitCMDRightFlag();

    return m_visionLocation[index];
}

void RobotClientBase::getInfo(INFOSTATE &infoType, std::string &infoString){
    m_robotInfo.getInfo(infoType, infoString);
}

void RobotClientBase::printInfo(INFOSTATE infoType, std::string infoString){
    m_robotInfo.setInfo(infoType, infoString);
}

void RobotClientBase::setCMDRightFlag(int flag){
    m_cmdRightFlag = flag;
}

int RobotClientBase::waitCMDRightFlag(){
    while(1){
        if(m_cmdRightFlag >= 0){
            return m_cmdRightFlag;
        }
        sleep_ms(m_robotParameter.getSamplePeriod()*1000);
    }
}

bool RobotClientBase::isCMDFinished(){
    if(m_cmdFinishFlag >= 0){
        return true;
    }
    return false;
}

void RobotClientBase::setCMDFinishFlag(int flag){
    m_cmdFinishFlag = flag;
}

int RobotClientBase::waitCMDFinishFlag(){
    while(1){
        if(m_cmdFinishFlag >= 0){
            return m_cmdFinishFlag;
        }
        sleep_ms(m_robotParameter.getSamplePeriod()*1000);
    }
}



//bool RobotClientBase::checkServo(){
//    bool state;
//    m_mutex.lockMutex();
//    if(m_robotState.m_servoStateFlag == 0 && m_robotState.m_virtualStateFlag == 0){
//        state = false;
//    }
//    else{
//        state = true;
//    }
//    m_mutex.unlockMutex();
//    return state;
//}
