#include "robotServerBase.h"
#include "robotStructure.h"
#include "robotFile.h"
#include <string>
//#include "baselib.h"

#define SEGCHAR string("|")

using namespace std;
using namespace rclib;

static void add_next_number(string& content, int num){
    content += num_to_string(num) + string(SEGCHAR);
}

static void add_next_number(string& content, double num){
    content += num_to_string(num) + string(SEGCHAR);
}

void RobotServerBase::sendCSWJ(const char* path){
    send_file_type(TCPSOURCE_CLIENT, "CSWJ", path);
}

void RobotServerBase::sendPZWJ(const char* path){
    send_file_type(TCPSOURCE_CLIENT, "PZWJ", path);
}

void RobotServerBase::sendZBWJ(const char *path){
    send_file_type(TCPSOURCE_CLIENT, "ZBWJ", path);
}


void RobotServerBase::sendCWXX(std::string information){
    send_msg_type(TCPSOURCE_CLIENT, "CWXX", information);
    //DBGPRINT(DBG_SERVER, "CWXX"<< information);
}

void RobotServerBase::sendJGXX(std::string information){
    send_msg_type(TCPSOURCE_CLIENT, "JGXX", information);
    //DBGPRINT(DBG_SERVER, "JGXX"<< information);
}

void RobotServerBase::sendTSXX(std::string information){
    send_msg_type(TCPSOURCE_CLIENT, "TSXX", information);
    //DBGPRINT(DBG_SERVER, "TSXX"<< information);
}


void RobotServerBase::sendWCIO(int state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "WCIO", content);
}

void RobotServerBase::sendWCZL(int state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "WCZL", content);
}

void RobotServerBase::sendCXXH(ProgramPointer pointer){
    string content;
    add_next_number(content, pointer.nFunction);
    add_next_number(content, pointer.nSentence);
    send_msg_type(TCPSOURCE_CLIENT, "CXXH", content);
}

void RobotServerBase::sendGJMD(const RobotMotion &rm){
    string content;
    for(int j=0; j<rm.getCurrentJointPosition().getJointsDOF(); j++){
        add_next_number(content, rm.getCurrentJointPosition()[JOINTINDEX(j)]);
    }
    for(int j=0; j<rm.getCurrentJointVelocity().getJointsDOF(); j++){
        add_next_number(content, rm.getCurrentJointVelocity()[JOINTINDEX(j)]);
    }
    for(int j=0; j<rm.getCurrentJointAcceleration().getJointsDOF(); j++){
        add_next_number(content, rm.getCurrentJointAcceleration()[JOINTINDEX(j)]);
    }
    for(int j=0; j<rm.getCurrentJointsTorque().getJointsDOF(); j++){
        add_next_number(content, rm.getCurrentJointsTorque()[JOINTINDEX(j)]);
    }
    for(int j=0; j<6; j++){
        add_next_number(content, rm.getCurrentTerminal()[TERMINALINDEX(j)]);
    }
    for(int j=0; j<6; j++){
        add_next_number(content, rm.getCurrentWorkTerminal()[TERMINALINDEX(j)]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "GJMD", content);
}

void RobotServerBase::sendTDDL(const JointsList &js){
    string content;
    for(int i=0; i<js.size(); i++){
        for(int j=0; j<js[i].getJointsDOF(); j++){
            add_next_number(content, js[i][JOINTINDEX(j)]);
        }
    }
    send_msg_type(TCPSOURCE_CLIENT, "TDDL", content);
}

void RobotServerBase::sendSJXH(LOCATEVISIONINDEX index){
    string content;
    add_next_number(content, index);
    send_msg_type(TCPSOURCE_OTHER, "SJXH", content);
}

void RobotServerBase::sendSJWZ(LOCATEVISIONINDEX index, const Terminal &t){
    string content;
    add_next_number(content, index);
    for(int j=0; j<6; j++){
        add_next_number(content, t[TERMINALINDEX(j)]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "SJWZ", content);
}

void RobotServerBase::sendIOZT(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "IOZT", content);
}

void RobotServerBase::sendSRZT(const DigitalInputState &state){
    string content;
    for(int i=0; i<DIGITAL_INPUTPORT_NUM; i++){
        add_next_number(content, state[PORTINDEX(i)]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "SRZT", content);
}

void RobotServerBase::sendSCZT(const DigitalOutputState &state){
    string content;
    for(int i=0; i<DIGITAL_OUTPUTPORT_NUM; i++){
        add_next_number(content, state[PORTINDEX(i)]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "SCZT", content);
}

void RobotServerBase::sendMNSR(const AnalogInputState &state){
    string content;
    for(int i=0; i<ANALOG_INPUT_NUM; i++){
        add_next_number(content, state[PORTINDEX(i)]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "MNSR", content);
}

void RobotServerBase::sendMNSC(const AnalogOutputState &state){
    string content;
    for(int i=0; i<ANALOG_OUTPUT_NUM; i++){
        add_next_number(content, state[PORTINDEX(i)]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "MNSC", content);
}

void RobotServerBase::sendCSYX(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "CSYX", content);
}

void RobotServerBase::sendFSZT(SYSRUNSTATE state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "FSZT", content);
}

void RobotServerBase::sendSFZT(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "SFZT", content);
}

void RobotServerBase::sendTSZT(SWITCHSTATE state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "TSZT", content);
}

void RobotServerBase::sendYXMS(SYSPLAYSTATE state){
    string content;
    add_next_number(content, state);
    send_msg_type(TCPSOURCE_CLIENT, "YXMS", content);
}

void RobotServerBase::sendKZXT(int fd){
    string content;
    add_next_number(content, 1);
    send_msg(fd, "KZXT", content);
    send_msg_exceptFd(fd, "KZXT", "0|");
}

void RobotServerBase::sendCXSD(double vel){
    string content;
    add_next_number(content, vel);
    send_msg_type(TCPSOURCE_CLIENT, "CXSD", content);
}


void RobotServerBase::sendXGGJ(string frame){
    send_msg_type(TCPSOURCE_CLIENT, "XGGJ", frame);
}

void RobotServerBase::sendXGYH(string frame){
    send_msg_type(TCPSOURCE_CLIENT, "XGYH", frame);
}

void RobotServerBase::sendTCPF(const Terminal &t, double p){
    string content;
    for(int j=0; j<6; j++){
        add_next_number(content, t[TERMINALINDEX(j)]);
    }
    add_next_number(content, p);
    send_msg_type(TCPSOURCE_CLIENT, "TCPF", content);
}

void RobotServerBase::sendUSRF(const Terminal &t){
    string content;
    for(int j=0; j<6; j++){
        add_next_number(content, t[TERMINALINDEX(j)]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "USRF", content);
}

void RobotServerBase::sendHOME(SWITCHSTATE state[8])
{
    string content;
    for (int i=0; i<8; i++) {

        add_next_number(content, state[i]);
    }
    send_msg_type(TCPSOURCE_CLIENT, "HOME", content);
}

void RobotServerBase::nullApi()
{
    excuteCommand(int(), TCPSOURCETYPE(0), string(), string());
}
