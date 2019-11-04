#include"robotServerBase.h"
#include "robotFile.h"
#include "robotStructure.h"
#include "functions.h"
#include <string>

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

void RobotServerBase::excuteFile(int fd, TCPSOURCETYPE src, string command) {
    switch (m_fileType[command]) {
    case CSWJ: commandCSWJ(fd, src); break;
    case MLWJ: commandMLWJ(fd, src); break;
    case PZWJ: commandPZWJ(fd, src); break;
    case ZBWJ: commandZBWJ(fd, src); break;
    default: break;
    }
}

void RobotServerBase::commandCSWJ(int fd, TCPSOURCETYPE src){
    RobotParameter robot;
    robot.readRobotParameter((m_strPath+ ROBOT_TEMP_RECV_FILE).c_str());
    setRobotParameter(robot);
    copy_file((m_strPath + ROBOT_TEMP_RECV_FILE).c_str(), (m_strPath+ ROBOT_PARAMETER_PATH).c_str());
}

void RobotServerBase::commandMLWJ(int fd, TCPSOURCETYPE src){
    copy_file((m_strPath + ROBOT_TEMP_RECV_FILE).c_str(), (m_strPath+ ROBOT_PROGRAM_PATH).c_str());
    programLoad((m_strPath + ROBOT_PROGRAM_PATH).c_str());
}

void RobotServerBase::commandPZWJ(int fd, TCPSOURCETYPE src){
    copy_file((m_strPath + ROBOT_TEMP_RECV_FILE).c_str(), (m_strPath+ ROBOT_PREFERENCE_PATH).c_str());
    RobotPreference rp;
    rp.readRobotPreference((m_strPath+ ROBOT_TEMP_RECV_FILE).c_str());
    setRobotPreference(rp);
}

void RobotServerBase::commandZBWJ(int fd, TCPSOURCETYPE src){
    copy_file((m_strPath + ROBOT_TEMP_RECV_FILE).c_str(), (m_strPath+ ROBOT_FRAME_PATH).c_str());
    RobotFrame rf;
    rf.readRobotFrame((m_strPath + ROBOT_TEMP_RECV_FILE).c_str());
    setRobotFrame(rf);
}

void RobotServerBase::excuteCommand(int fd, TCPSOURCETYPE src, string command, string content){
    switch(m_commandType[command]){
    case JLLJ: commandJLLJ(fd, src, content); break;
    case GBLJ: commandGBLJ(fd, src, content); break;
    case QCCW: commandQCCW(fd, src, content); break;
    case CXSD: commandCXSD(fd, src, content); break;
    case SJXH: commandSJXH(fd, src, content); break;
    case SJWZ: commandSJWZ(fd, src, content); break;
    case HLJZ: commandHLJZ(fd, src, content); break;
    case XWHL: commandXWHL(fd, src, content); break;

    case SCZT: commandSCZT(fd, src, content); break;
    case MNSC: commandMNSC(fd, src, content); break;
    case MNDL: commandMNDL(fd, src, content); break;
    case CSYX: commandCSYX(fd, src, content); break;
    case FSZT: commandFSZT(fd, src, content); break;
    case SFZT: commandSFZT(fd, src, content); break;
    case TSZT: commandTSZT(fd, src, content); break;
    case YXMS: commandYXMS(fd, src, content); break;
    case KZXT: commandKZXT(fd, src, content); break;

    case XGGJ: commandXGGJ(fd, src, content); break;
    case XGYH: commandXGYH(fd, src, content); break;
    case TCPF: commandTCPF(fd, src, content); break;
    case TCFZ: commandTCFZ(fd, src, content); break;
    case TCFX: commandTCFX(fd, src, content); break;
    case USRF: commandUSRF(fd, src, content); break;

    case GJDD: commandGJDD(fd, src, content); break;
    case MDDD: commandMDDD(fd, src, content); break;
    case DDTZ: commandDDTZ(fd, src, content); break;
    case GJBJ: commandGJBJ(fd, src, content); break;
    case MDBJ: commandMDBJ(fd, src, content); break;
    case GJYX: commandGJYX(fd, src, content); break;
    case MLYX: commandMLYX(fd, src, content); break;
    case MLZT: commandMLZT(fd, src, content); break;
    case MLJX: commandMLJX(fd, src, content); break;
    case MLTZ: commandMLTZ(fd, src, content); break;
    case DBYX: commandDBYX(fd, src, content); break;
    case DBZT: commandDBZT(fd, src, content); break;
    case CXXH: commandCXXH(fd, src, content); break;

    case KSTD: commandKSTD(fd, src, content); break;
    case JSTD: commandJSTD(fd, src, content); break;
    case JLTD: commandJLTD(fd, src, content); break;
    case ZXZB: commandZXZB(fd, src, content); break;
    case ZXTD: commandZXTD(fd, src, content); break;

    case SZLX: commandSZLX(fd, src, content); break;
    case BEEP: commandBEEP(fd, src, content); break;
    case SZYY: commandSZYY(fd, src, content); break;

    case TEST: commandTEST(fd, src, content); break;

    default: break;
    }
}

void RobotServerBase::commandJLLJ(int fd, TCPSOURCETYPE src, string content){
    if(!setFirstConnect(fd, src)){
        if(src == TCPSOURCE_CLIENT){
            sendKZXT(controlFd());
            updateRobotClient();
        }
        if(src == TCPSOURCE_OTHER){
            m_cameraFd[fd] = LOCATEVISIONINDEX((int)get_next_number(content));
        }
    }
}

void RobotServerBase::commandGBLJ(int fd, TCPSOURCETYPE src, string content){
    if(src == TCPSOURCE_OTHER){
        m_cameraFd.erase(fd);
    }
}

void RobotServerBase::commandQCCW(int fd, TCPSOURCETYPE src, string content){
    setErrorClear();
}

void RobotServerBase::commandCXSD(int fd, TCPSOURCETYPE src, string content){
    programVelocity(get_next_number(content));
}

void RobotServerBase::commandSJXH(int fd, TCPSOURCETYPE src, string content){
    getVisionLocation(LOCATEVISIONINDEX((int)get_next_number(content)));
}

void RobotServerBase::commandSJWZ(int fd, TCPSOURCETYPE src, string content){
    LOCATEVISIONINDEX index = LOCATEVISIONINDEX((int)get_next_number(content));
    Terminal terminal;

    VECTOR_STRING vecContent = Common::ToVectors(content,SEGCHAR.c_str());
    int nSize = vecContent.size();
    VECTOR_DOUBLE vecDouble;
    for(int i=0; i<nSize; i++){
        vecDouble.push_back(string_to_double(vecContent[i]));
    }
    switch (nSize) {
    case 6:
        for(int i=0; i<6; i++){
            terminal[TERMINALINDEX(i)] = vecDouble[i];
        }
        break;
    case 7:
    {
        Point po(vecDouble[0],vecDouble[1],vecDouble[2]);
        Quaternion qn(vecDouble[3],vecDouble[4],vecDouble[5],vecDouble[6]);
        terminal.setValue(po,qn.getAttitudeAngle());
        break;
    }
    case 16:
    {
        CMatrix<double> value(4,4,vecDouble);
        HomogeneousMatrix homo(value);
        terminal.setValue(homo);
        break;
    }
    default:
        break;
    }

    setVisionLocation(index, terminal);
}

void RobotServerBase::commandXWHL(int fd, TCPSOURCETYPE src, string content){
    moveToZero(get_next_number(content));
}

void RobotServerBase::commandHLJZ(int fd, TCPSOURCETYPE src, string content){
    setJointZero(JOINTINDEX((int)get_next_number(content)));
}


void RobotServerBase::commandSCZT(int fd, TCPSOURCETYPE src, string content){
    DigitalOutputState state;
    for(int i=0; i<DIGITAL_OUTPUTPORT_NUM; i++){
        state[PORTINDEX(i)] = SWITCHSTATE((int)get_next_number(content));
    }
    setDigitalOutput(state);
}

void RobotServerBase::commandMNSC(int fd, TCPSOURCETYPE src, string content){
    AnalogOutputState state;
    for(int i=0; i<ANALOG_OUTPUT_NUM; i++){
        state[PORTINDEX(i)] = get_next_number(content);
    }
    setAnalogOutput(state);
}

void RobotServerBase::commandMNDL(int fd, TCPSOURCETYPE src, string content)
{
    PORTINDEX index = PORTINDEX((int)get_next_number(content));
    double value = get_next_number(content);
    setAnalogOutput(index,value);
}

void RobotServerBase::commandCSYX(int fd, TCPSOURCETYPE src, string content){
    setTryRunState(SWITCHSTATE((int)get_next_number(content)));
}

void RobotServerBase::commandFSZT(int fd, TCPSOURCETYPE src, string content){
    setRunState(SYSRUNSTATE((int)get_next_number(content)));
}

void RobotServerBase::commandSFZT(int fd, TCPSOURCETYPE src, string content){
    setServoState(SWITCHSTATE((int)get_next_number(content)));
}

void RobotServerBase::commandTSZT(int fd, TCPSOURCETYPE src, string content){
    setDebugState(SWITCHSTATE((int)get_next_number(content)));
}

void RobotServerBase::commandYXMS(int fd, TCPSOURCETYPE src, string content){
    setPlayState(SYSPLAYSTATE((int)get_next_number(content)));
}

void RobotServerBase::commandKZXT(int fd, TCPSOURCETYPE src, string content){
    setControlFd(fd);
    sendKZXT(controlFd());
}


void RobotServerBase::commandXGGJ(int fd, TCPSOURCETYPE src, string content){
    modifyToolFrame(content);
}

void RobotServerBase::commandXGYH(int fd, TCPSOURCETYPE src, string content){
    modifyWorkFrame(content);
}

void RobotServerBase::commandTCPF(int fd, TCPSOURCETYPE src, string content){
    JointsList js;
    while(!content.empty()){
        Joints joint(m_dof);
        for(int i=0; i<joint.getJointsDOF(); i++){
            joint[JOINTINDEX(i)] = get_next_number(content);
        }
        js.push_back(joint);
    }
    Terminal terminal;
    calibrateTCP(js, terminal);
}

void RobotServerBase::commandTCFZ(int fd, TCPSOURCETYPE src, string content){
    Joints jo(m_dof);
    for(int i=0; i<jo.getJointsDOF(); i++){
        jo[JOINTINDEX(i)] = get_next_number(content);
    }
    Joints jz(m_dof);
    for(int i=0; i<jz.getJointsDOF(); i++){
        jz[JOINTINDEX(i)] = get_next_number(content);
    }
    JointsList js;
    while(!content.empty()){
        Joints joint(m_dof);
        for(int i=0; i<joint.getJointsDOF(); i++){
            joint[JOINTINDEX(i)] = get_next_number(content);
        }
        js.push_back(joint);
    }
    Terminal terminal;
    calibrateTCFZ(js, jo, jz, terminal);
}

void RobotServerBase::commandTCFX(int fd, TCPSOURCETYPE src, string content){
    Joints jo(m_dof);
    for(int i=0; i<jo.getJointsDOF(); i++){
        jo[JOINTINDEX(i)] = get_next_number(content);
    }
    Joints jx(m_dof);
    for(int i=0; i<jx.getJointsDOF(); i++){
        jx[JOINTINDEX(i)] = get_next_number(content);
    }
    Joints jz(m_dof);
    for(int i=0; i<jz.getJointsDOF(); i++){
        jz[JOINTINDEX(i)] = get_next_number(content);
    }
    JointsList js;
    while(!content.empty()){
        Joints joint(m_dof);
        for(int i=0; i<joint.getJointsDOF(); i++){
            joint[JOINTINDEX(i)] = get_next_number(content);
        }
        js.push_back(joint);
    }
    Terminal terminal;
    calibrateTCFX(js, jo, jx, jz, terminal);
}

void RobotServerBase::commandUSRF(int fd, TCPSOURCETYPE src, string content){
    Terminal to;
    for(int i=0; i<6; i++){
        to[TERMINALINDEX(i)] = get_next_number(content);
    }
    Terminal tx;
    for(int i=0; i<6; i++){
        tx[TERMINALINDEX(i)] = get_next_number(content);
    }
    Terminal ty;
    for(int i=0; i<6; i++){
        ty[TERMINALINDEX(i)] = get_next_number(content);
    }
    Terminal terminal;
    calibrateUSRF(to, tx, ty, terminal);
}


void RobotServerBase::commandGJDD(int fd, TCPSOURCETYPE src, string content){
    JOINTINDEX index = JOINTINDEX((int)get_next_number(content));
    MOVEDIRECTION dir = MOVEDIRECTION((int)get_next_number(content));
    double vel = get_next_number(content);
    jointJOG(index, dir, vel);
}

void RobotServerBase::commandMDDD(int fd, TCPSOURCETYPE src, string content){
    TERMINALINDEX index = TERMINALINDEX((int )get_next_number(content));
    MOVEDIRECTION dir = MOVEDIRECTION((int)get_next_number(content));
    double vel = get_next_number(content);
    COORDINATESYSTEM frame = COORDINATESYSTEM((int)get_next_number(content));
    terminalJOG(index, dir, vel, frame);
}

void RobotServerBase::commandDDTZ(int fd, TCPSOURCETYPE src, string content){
    stopJOG();
}

void RobotServerBase::commandGJBJ(int fd, TCPSOURCETYPE src, string content){
    JOINTINDEX index = JOINTINDEX((int)get_next_number(content));
    MOVEDIRECTION dir = MOVEDIRECTION((int)get_next_number(content));
    double step = get_next_number(content);
    double vel = get_next_number(content);
    jointStep(index, dir, step, vel);
}

void RobotServerBase::commandMDBJ(int fd, TCPSOURCETYPE src, string content){
    TERMINALINDEX index = TERMINALINDEX((int)get_next_number(content));
    MOVEDIRECTION dir = MOVEDIRECTION((int)get_next_number(content));
    double vel = get_next_number(content);
    double step = get_next_number(content);
    COORDINATESYSTEM frame = COORDINATESYSTEM((int)get_next_number(content));
    terminalStep(index, dir, vel, step, frame);
}

void RobotServerBase::commandGJYX(int fd, TCPSOURCETYPE src, string content){
    MotionNode node;
    node.motionType = MOTIONTYPE((int)get_next_number(content));
    node.accRatio = get_next_number(content);
    node.jerkRatio = get_next_number(content);
    node.angleRatio = get_next_number(content);
    node.precision = get_next_number(content);
    node.referFrame = COORDINATESYSTEM((int)get_next_number(content));
    node.circleType = CIRCLETYPE((int)get_next_number(content));
    node.height = get_next_number(content);
    node.delayNum = get_next_number(content);
    int num = get_next_number(content);
    node.velList.clear();
    for(int i=0; i<num; i++){
        node.velList.push_back(get_next_number(content));
    }
    num = get_next_number(content);
    node.jointList.clear();
    for(int i=0; i<num; i++){
        Joints joint(m_dof);
        for(int j=0; j<m_dof; j++){
            joint[JOINTINDEX(j)] = get_next_number(content);
        }
        node.jointList.push_back(joint);
    }
    num = get_next_number(content);
    node.terminalList.clear();
    for(int i=0; i<num; i++){
        Terminal terminal;
        for(int j=0; j<6; j++){
            terminal[TERMINALINDEX(j)] = get_next_number(content);
        }
        node.terminalList.push_back(terminal);
    }

    try{
    switch(node.motionType){
    case MOTION_MOVEABSJ: moveABSJoint(node.jointList, node.velList[0]); break;
    case MOTION_MOVEABSJR:moveABSJointR(node.jointList, node.velList[0]); break;
    case MOTION_MOVEABSJ_TIME:moveABSJoint(node.jointList, node.velList); break;
    case MOTION_MOVEABSJR_TIME:moveABSJointR(node.jointList, node.velList); break;
    case MOTION_MOVEJ: moveJoint(node.terminalList, node.velList[0], node.referFrame); break;
    case MOTION_MOVEJR: moveJointR(node.terminalList, node.velList[0], node.referFrame); break;
    case MOTION_MOVEL: moveLine(node.terminalList[0], node.velList[0], node.accRatio, node.jerkRatio, node.referFrame); break;
    case MOTION_MOVELR: moveLineR(node.terminalList[0], node.velList[0], node.accRatio, node.jerkRatio, node.referFrame); break;
    case MOTION_MOVEC: moveCircle(node.circleType, node.terminalList[0], node.terminalList[1], node.velList[0], node.accRatio, node.jerkRatio, node.referFrame); break;
    case MOTION_MOVECR: moveCircleR(node.circleType, node.terminalList[0], node.terminalList[1], node.velList[0], node.accRatio, node.jerkRatio, node.referFrame); break;
    case MOTION_MOVEB: moveCurve(node.terminalList, node.velList, node.accRatio, node.jerkRatio, node.angleRatio, node.precision, node.referFrame); break;
    case MOTION_MOVEJUMP: moveJump(node.terminalList[0], node.height, node.velList[0], node.accRatio, node.jerkRatio, node.precision); break;
    case MOTION_MOVEL_CON: moveLineCon(node.terminalList[0], node.velList[0], node.accRatio, node.jerkRatio, node.precision, node.referFrame); break;
    case MOTION_MOVELR_CON: moveLineRCon(node.terminalList[0], node.velList[0], node.accRatio, node.jerkRatio, node.precision, node.referFrame); break;
    case MOTION_MOVEC_CON: moveCircleCon(node.circleType, node.terminalList[0], node.terminalList[1], node.velList[0], node.accRatio, node.jerkRatio, node.precision, node.referFrame); break;
    case MOTION_MOVECR_CON: moveCircleRCon(node.circleType, node.terminalList[0], node.terminalList[1], node.velList[0], node.accRatio, node.jerkRatio, node.precision, node.referFrame); break;
    case MOTION_START_CON: moveStartCon(node.delayNum); break;
    case MOTION_END_CON: moveEndCon(); break;
    default:break;
    }
    }
    catch(string e){
        cout << e << endl;
    }
}

void RobotServerBase::commandMLYX(int fd, TCPSOURCETYPE src, string content){
    ProgramPointer pointer;
    pointer.nFunction = (int)get_next_number(content);
    pointer.nSentence = (int)get_next_number(content);
    setBreakPointer( pointer);
    programRun();
}

void RobotServerBase::commandMLZT(int fd, TCPSOURCETYPE src, string content){
    programPause();
}

void RobotServerBase::commandMLJX(int fd, TCPSOURCETYPE src, string content)
{
    programPause(false);
}

void RobotServerBase::commandMLTZ(int fd, TCPSOURCETYPE src, string content){
    programStop();
}

void RobotServerBase::commandDBYX(int fd, TCPSOURCETYPE src, string content){
    programStepRun();
}

void RobotServerBase::commandDBZT(int fd, TCPSOURCETYPE src, string content){
    programStepPause();
}

void RobotServerBase::commandCXXH(int fd, TCPSOURCETYPE src, string content){
    ProgramPointer point;
    point.nFunction = get_next_number(content);
    point.nSentence = get_next_number(content);
    setProgramPointer(point);
}


void RobotServerBase::commandKSTD(int fd, TCPSOURCETYPE src, string content){
    dragModeStart();
}

void RobotServerBase::commandJSTD(int fd, TCPSOURCETYPE src, string content){
    dragModeEnd();
}

void RobotServerBase::commandJLTD(int fd, TCPSOURCETYPE src, string content){
    int period_ms = get_next_number(content);
    int timeLength_s = get_next_number(content);
    dragModeTimeStart(period_ms, timeLength_s);
}

void RobotServerBase::commandZXZB(int fd, TCPSOURCETYPE src, string content){
    double vel = get_next_number(content);
    dragModePlayPrepare(vel);
}

void RobotServerBase::commandZXTD(int fd, TCPSOURCETYPE src, string content){
    int period_ms = get_next_number(content);
    dragModePlay(period_ms);
}

void RobotServerBase::commandSZLX(int fd, TCPSOURCETYPE src, string content)
{
    int n = get_next_number(content);
    setTeachType(n);
}

void RobotServerBase::commandBEEP(int fd, TCPSOURCETYPE src, string content)
{
    double time = get_next_number(content);
    setBeep(time);
}

void RobotServerBase::commandSZYY(int fd, TCPSOURCETYPE src, string content)
{
    int type = get_next_number(content);
    setSZYY(type);
}


void RobotServerBase::commandTEST(int fd, TCPSOURCETYPE src, string content){
    getVisionLocation(LOCATEVISION_2);
}
