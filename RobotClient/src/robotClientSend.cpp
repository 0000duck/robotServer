#include "robotClientBase.h"
#include "robotStructure.h"
#include "robotFile.h"
#include <sstream>
#include <string>

#define SEGCHAR string("|")

using namespace std;
using namespace rclib;

static void add_next_number(string& content, int num){
    content += num_to_string(num) + string(SEGCHAR);
}

static void add_next_number(string& content, double num){
    content += num_to_string(num) + string(SEGCHAR);
}

void RobotClientBase::sendCSWJ(const char* path){
    send_file("CSWJ", path);
}

void RobotClientBase::sendMLWJ(const char* path){
    send_file("MLWJ", path);
}

void RobotClientBase::sendPZWJ(const char* path){
    send_file("PZWJ", path);
}

void RobotClientBase::sendZBWJ(const char* path){
    send_file("ZBWJ", path);
}


void RobotClientBase::sendJLLJ(){
    send_msg("JLLJ", "1|");
}

void RobotClientBase::sendQCCW(){
    send_msg("QCCW", "1|");
}

void RobotClientBase::sendCXSD(double vel){
    string content;
    add_next_number(content, vel);
    send_msg("CXSD", content);
}

void RobotClientBase::sendSJXH(LOCATEVISIONINDEX index){
    string content;
    add_next_number(content, index);
    send_msg("SJXH", content);
}

void RobotClientBase::sendXWHL(double vel){
    string content;
    add_next_number(content, vel);
    send_msg("XWHL", content);
}

void RobotClientBase::sendHLJZ(JOINTINDEX index){
    string content;
    add_next_number(content, index);
    send_msg("HLJZ", content);
}


void RobotClientBase::sendSCZT(const DigitalOutputState &state){
    string content;
    for(int i=0; i<DIGITAL_OUTPUTPORT_NUM; i++){
        add_next_number(content, state[PORTINDEX(i)]);
    }
    send_msg("SCZT", content);
}

void RobotClientBase::sendMNSC(const AnalogOutputState &state){
    string content;
    for(int i=0; i<ANALOG_OUTPUT_NUM; i++){
        add_next_number(content, state[PORTINDEX(i)]);
    }
    send_msg("MNSC", content);
}

void RobotClientBase::sendMNDL(PORTINDEX index, double value)
{
    string content;
    add_next_number(content, index);
    add_next_number(content, value);
    send_msg("MNDL", content);
}

void RobotClientBase::sendCSYX(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg("CSYX", content);
}

void RobotClientBase::sendFSZT(SYSRUNSTATE state){
    string content;
    add_next_number(content, state);
    send_msg("FSZT", content);
}

void RobotClientBase::sendSFZT(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg("SFZT", content);
}

void RobotClientBase::sendTSZT(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg("TSZT", content);
}

void RobotClientBase::sendYXMS(SYSPLAYSTATE state){
    string content;
    add_next_number(content, state);
    send_msg("YXMS", content);
}

void RobotClientBase::sendKZXT(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg("KZXT", content);
}


void RobotClientBase::sendXGGJ(std::string frame){
    send_msg("XGGJ", frame);
}

void RobotClientBase::sendXGYH(std::string frame){
    send_msg("XGYH", frame);
}

void RobotClientBase::sendTCPF(const JointsList& js){
    string content;
    for(int i=0; i<js.size(); i++){
        for(int j=0; j<js[i].getJointsDOF(); j++){
            add_next_number(content, js[i][JOINTINDEX(j)]);
        }
    }
    send_msg("TCPF", content);
}

void RobotClientBase::sendTCFZ(const JointsList &js, const Joints &jo, const Joints &jz){
    string content;
    for(int j=0; j<jo.getJointsDOF(); j++){
        add_next_number(content, jo[JOINTINDEX(j)]);
    }
    for(int j=0; j<jz.getJointsDOF(); j++){
        add_next_number(content, jz[JOINTINDEX(j)]);
    }
    for(int i=0; i<js.size(); i++){
        for(int j=0; j<js[i].getJointsDOF(); j++){
            add_next_number(content, js[i][JOINTINDEX(j)]);
        }
    }
    send_msg("TCFZ", content);
}

void RobotClientBase::sendTCFX(const JointsList &js, const Joints &jo, const Joints &jx, const Joints &jz){
    string content;
    for(int j=0; j<jo.getJointsDOF(); j++){
        add_next_number(content, jo[JOINTINDEX(j)]);
    }
    for(int j=0; j<jx.getJointsDOF(); j++){
        add_next_number(content, jx[JOINTINDEX(j)]);
    }
    for(int j=0; j<jz.getJointsDOF(); j++){
        add_next_number(content, jz[JOINTINDEX(j)]);
    }
    for(int i=0; i<js.size(); i++){
        for(int j=0; j<js[i].getJointsDOF(); j++){
            add_next_number(content, js[i][JOINTINDEX(j)]);
        }
    }
    send_msg("TCFX", content);
}

void RobotClientBase::sendUSRF(const Terminal& to, const Terminal& tx, const Terminal& ty){
    string content;
    for(int j=0; j<6; j++){
        add_next_number(content, to[TERMINALINDEX(j)]);
    }
    for(int j=0; j<6; j++){
        add_next_number(content, tx[TERMINALINDEX(j)]);
    }
    for(int j=0; j<6; j++){
        add_next_number(content, ty[TERMINALINDEX(j)]);
    }
    send_msg("USRF", content);
}

void RobotClientBase::sendGJDD(JOINTINDEX index, MOVEDIRECTION dir, double vel){
    string content;
    add_next_number(content, index);
    add_next_number(content, dir);
    add_next_number(content, vel);
    send_msg("GJDD", content);
}

void RobotClientBase::sendMDDD(TERMINALINDEX index, MOVEDIRECTION dir, double vel, COORDINATESYSTEM frame){
    string content;
    add_next_number(content, index);
    add_next_number(content, dir);
    add_next_number(content, vel);
    add_next_number(content, frame);
    send_msg("MDDD", content);
}

void RobotClientBase::sendDDTZ(){
    send_msg("DDTZ", "1");
}

void RobotClientBase::sendGJBJ(JOINTINDEX index, MOVEDIRECTION dir, double step, double vel){
    string content;
    add_next_number(content, index);
    add_next_number(content, dir);
    add_next_number(content, step);
    add_next_number(content, vel);
    //send_msg("GJDD", content);
    send_msg("GJBJ", content);
}

void RobotClientBase::sendMDBJ(TERMINALINDEX index, MOVEDIRECTION dir, double step, double vel, COORDINATESYSTEM frame){
    string content;
    add_next_number(content, index);
    add_next_number(content, dir);
    add_next_number(content, step);
    add_next_number(content, vel);
    add_next_number(content, frame);
    send_msg("MDBJ", content);
}

void RobotClientBase::sendGJYX(MotionNode& node){
    string content;
    add_next_number(content, node.motionType);
    add_next_number(content, node.accRatio);
    add_next_number(content, node.jerkRatio);
    add_next_number(content, node.angleRatio);
    add_next_number(content, node.precision);
    add_next_number(content, node.referFrame);
    add_next_number(content, node.circleType);
    add_next_number(content, node.height);
    add_next_number(content, node.delayNum);

    add_next_number(content, (int)node.velList.size());
    for(int i=0; i<node.velList.size(); i++){
        add_next_number(content, node.velList[i]);
    }

    add_next_number(content, (int)node.jointList.size());
    for(int i=0; i<node.jointList.size(); i++){
        for(int j=0; j<m_robotParameter.getWholeDOF(); j++){
            add_next_number(content, node.jointList[i].getValue(JOINTINDEX(j)));
        }
    }

    add_next_number(content, (int)node.terminalList.size());
    for(int i=0; i<node.terminalList.size(); i++){
        for(int j=0; j<6; j++){
            add_next_number(content, node.terminalList[i].getValue(TERMINALINDEX(j)));
        }
    }
    send_msg("GJYX", content);
}


void RobotClientBase::sendMLYX(const ProgramPointer &pointer){
    string content;
    add_next_number(content, pointer.nFunction);
    add_next_number(content, pointer.nSentence);
    send_msg("MLYX", content);
}

void RobotClientBase::sendMLZT(){
    send_msg("MLZT", "1");
}

void RobotClientBase::sendMLJX()
{
    send_msg("MLJX", "1");
}

void RobotClientBase::sendMLTZ(){
    send_msg("MLTZ", "1");
}

void RobotClientBase::sendDBYX(){
    send_msg("DBYX", "1");
}

void RobotClientBase::sendDBZT(){
    send_msg("DBZT", "1");
}

void RobotClientBase::sendCXXH(const ProgramPointer &pointer){
    string content;
    add_next_number(content, pointer.nFunction);
    add_next_number(content, pointer.nFunction);
    send_msg("CXXH", content);
}


void RobotClientBase::sendKSTD(){
    send_msg("KSTD", "1");
}

void RobotClientBase::sendJSTD(){
    send_msg("JSTD", "1");
}

void RobotClientBase::sendJLTD(int period_ms, int timeLength_s){
    string content;
    add_next_number(content, period_ms);
    add_next_number(content, timeLength_s);
    send_msg("JLTD", content);
}

void RobotClientBase::sendZXZB(double vel){
    string content;
    add_next_number(content, vel);
    send_msg("ZXZB", content);
}

void RobotClientBase::sendZXTD(int period_ms){
    string content;
    add_next_number(content, period_ms);
    send_msg("ZXTD", content);
}


void RobotClientBase::sendTEST(){
    send_msg("TEST", "1");
}

void RobotClientBase::nullApi()
{
    excuteCommand(int(), TCPSOURCETYPE(0), string(), string());
}

void RobotClientBase::sendSZLX(int n)
{
    string content;
    add_next_number(content, n);
    send_msg("SZLX", content);
}

void RobotClientBase::sendBEEP(double time)
{
    string content;
    add_next_number(content, time);
    send_msg("BEEP", content);
}

void RobotClientBase::sendSZYY(ERROR_INIF_LANGUAGE time)
{
    string content;
    add_next_number(content, time);
    send_msg("SZYY", content);
}
