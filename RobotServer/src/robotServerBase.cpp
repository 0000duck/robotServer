#include "robotServerBase.h"
#include "robotFile.h"
#include <signal.h>
using namespace std;
using namespace rclib;

RobotServerBase::RobotServerBase(){
    initParameter();
}

RobotServerBase::~RobotServerBase(){

}
void handle_pipe(int sig)
{
    //不做任何处理即可
    //cout << "client over!!!!!"<<endl;
}
void RobotServerBase::initSystemBase(int listen_port){
#ifdef __linux__
    struct sigaction action;
    action.sa_handler = handle_pipe;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGPIPE, &action, NULL);
#endif
    tcpServerInit(listen_port);
}

void RobotServerBase::setDof(int dof){
    m_dof = dof;
}

SWITCHSTATE RobotServerBase::getCameraConnect(LOCATEVISIONINDEX index){
    for(map<int, LOCATEVISIONINDEX>::iterator it = m_cameraFd.begin(); it != m_cameraFd.end(); it++){
        if(index == it->second){
            return SWITCHON;
        }
    }
    return SWITCHOFF;
}

void RobotServerBase::initParameter(){
    m_fileType["CSWJ"] = CSWJ;
    m_fileType["MLWJ"] = MLWJ;
    m_fileType["PZWJ"] = PZWJ;
    m_fileType["ZBWJ"] = ZBWJ;

    m_commandType["JLLJ"] = JLLJ;
    m_commandType["GBLJ"] = GBLJ;
    m_commandType["QCCW"] = QCCW;
    m_commandType["CXSD"] = CXSD;
    m_commandType["SJXH"] = SJXH;
    m_commandType["SJWZ"] = SJWZ;
    m_commandType["HLJZ"] = HLJZ;
    m_commandType["XWHL"] = XWHL;

    m_commandType["SCZT"] = SCZT;
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
    m_commandType["TCFZ"] = TCFZ;
    m_commandType["TCFX"] = TCFX;
    m_commandType["USRF"] = USRF;

    m_commandType["GJDD"] = GJDD;
    m_commandType["MDDD"] = MDDD;
    m_commandType["DDTZ"] = DDTZ;
    m_commandType["GJBJ"] = GJBJ;
    m_commandType["MDBJ"] = MDBJ;
    m_commandType["GJYX"] = GJYX;
    m_commandType["MLYX"] = MLYX;
    m_commandType["MLZT"] = MLZT;
    m_commandType["MLJX"] = MLJX;
    m_commandType["MLTZ"] = MLTZ;
    m_commandType["DBYX"] = DBYX;
    m_commandType["DBZT"] = DBZT;
    m_commandType["CXXH"] = CXXH;

    m_commandType["KSTD"] = KSTD;
    m_commandType["JSTD"] = JSTD;
    m_commandType["JLTD"] = JLTD;
    m_commandType["ZXZB"] = ZXZB;
    m_commandType["ZXTD"] = ZXTD;
    m_commandType["SZLX"] = SZLX;
    m_commandType["BEEP"] = BEEP;
    m_commandType["SZYY"] = SZYY;

    m_commandType["TEST"] = TEST;
}
