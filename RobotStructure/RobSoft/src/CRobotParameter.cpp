#ifndef CROBOTPARAMETER_CPP
#define CROBOTPARAMETER_CPP

#include "CRobotParameter.hpp"
#include "CFileIO.hpp"

#include <iostream>

using namespace std;
using namespace robsoft;

RobotParameter::RobotParameter(){
    m_robotType = ROBSOFT_SERIAL_SIX_CONVENTION;
    m_samplePeriod = 0.002;

    m_jointMaxVelRatio = 1;
    m_jointMaxAccRatio = 1;
    m_jointMaxJerkRatio = 1;

    m_terminalMaxVelRatio = 1;
    m_terminalMaxAccRatio = 1;
    m_terminalMaxJerkRatio = 1;
}

RobotParameter::~RobotParameter(){

}

void RobotParameter::setRobotParameter(const RobotParameter& robot){
    *this = robot;
}

RobotParameter RobotParameter::getRobotParameter() const{
    return *this;
}

void RobotParameter::setRobotDOF(int robotDof){
    m_robotDOF = robotDof;
}

void RobotParameter::setExternDOF(int externDof){
    m_externDOF = externDof;
}

void RobotParameter::setRobotType(ROBOTTYPE robotType){
    m_robotType = robotType;
    switch(m_robotType){
    case ROBSOFT_SERIAL_SIX_CONVENTION: m_robotDOF = 6;
        break;
    case ROBSOFT_SERIAL_SIX_COOPERATION: m_robotDOF = 6;
        break;
    case ROBSOFT_SERIAL_FOUR_CONVENTION: m_robotDOF = 5;
        break;
    case ROBOTSOFT_SCARA_FOUR_ONERF: m_robotDOF = 4;
        break;
    case ROBOTSOFT_SCARA_FOUR_FOURRF: m_robotDOF = 4;
        break;
    case ROBOTSOFT_DELTA_FOUR: m_robotDOF = 4;
        break;
    case ROBOTSOFT_DELTA_SIX: m_robotDOF = 6;
        break;
    default:
        break;
    }
}

void RobotParameter::setSamplePeriod(double samplePeriod){
    m_samplePeriod = samplePeriod;
}

void RobotParameter::setEncoderResolution(const Joints& encoderResolution){
    m_encoderResolution = encoderResolution;
}

void RobotParameter::setRateTorque(const Joints& rateTorque){
    m_rateTorque = rateTorque;
}

void RobotParameter::setReduceRatio(const Joints& reduceRatio){
    m_reduceRatio = reduceRatio;
}

void RobotParameter::setJointRange(const Joints& jointRangeMinus, const Joints& jointRangePlus){
    m_jointRangeMinus = jointRangeMinus;
    m_jointRangePlus = jointRangePlus;
}

void RobotParameter::setJointMaxVelRange(const Joints& jointMaxVel){
    m_jointMaxVel = jointMaxVel;
}

void RobotParameter::setJointMaxAccRange(const Joints& jointMaxAcc){
    m_jointMaxAcc = jointMaxAcc;
}

void RobotParameter::setJointMaxJerkRange(const Joints& jointMaxJerk){
    m_jointMaxJerk = jointMaxJerk;
}

void RobotParameter::setJointMaxVelRatio(double jointMaxVelRatio){
    m_jointMaxVelRatio = jointMaxVelRatio;
}

void RobotParameter::setJointMaxAccRatio(double jointMaxAccRatio){
    m_jointMaxAccRatio = jointMaxAccRatio;
}

void RobotParameter::setJointMaxJerkRatio(double jointMaxJerkRatio){
    m_jointMaxJerkRatio = jointMaxJerkRatio;
}

void RobotParameter::setTerminalRange(const Terminal& terminalRangeMinus, const Terminal& terminalRangePlus){
    m_terminalRangeMinus = terminalRangeMinus;
    m_terminalRangePlus = terminalRangePlus;
}

void RobotParameter::setTerminalMaxVelRange(const Terminal& terminalMaxVel){
    m_terminalMaxVel = terminalMaxVel;
}

void RobotParameter::setTerminalMaxAccRange(const Terminal& terminalMaxAcc){
    m_terminalMaxAcc = terminalMaxAcc;
}

void RobotParameter::setTerminalMaxJerkRange(const Terminal& terminalMaxJerk){
    m_terminalMaxJerk = terminalMaxJerk;
}

void RobotParameter::setTerminalMaxVelRatio(double terminalMaxVelRatio){
    m_terminalMaxVelRatio = terminalMaxVelRatio;
}

void RobotParameter::setTerminalMaxAccRatio(double terminalMaxAccRatio){
    m_terminalMaxAccRatio = terminalMaxAccRatio;
}

void RobotParameter::setTerminalMaxJerkRatio(double terminalMaxJerkRatio){
    m_terminalMaxJerkRatio = terminalMaxJerkRatio;
}

void RobotParameter::setDHParameter(const Joints& DHParameterAlpha, const Joints& DHParameterA, const Joints& DHParameterD, const Joints& DHParameterTheta){
    m_DHParameterAlpha = DHParameterAlpha;
    m_DHParameterA = DHParameterA;
    m_DHParameterD = DHParameterD;
    m_DHParameterTheta = DHParameterTheta;
}

void RobotParameter::setToolFrame(const Terminal& toolFrame){
    m_toolFrame = toolFrame;
}

void RobotParameter::setWorkFrame(const Terminal& workFrame){
    m_workFrame = workFrame;
}


void RobotParameter::getRobotType(ROBOTTYPE& robotType) const{
    robotType = m_robotType;
}

void RobotParameter::getSamplePeriod(double& samplePeriod) const{
    samplePeriod = m_samplePeriod;
}

void RobotParameter::getEncoderResolution(Joints& encoderResolution) const{
    encoderResolution = m_encoderResolution;
}

void RobotParameter::getRateTorque(Joints& rateTorque) const{
    rateTorque = m_rateTorque;
}

void RobotParameter::getReduceRatio(Joints& reduceRatio) const{
    reduceRatio = m_reduceRatio;
}

void RobotParameter::getJointRange(Joints& jointRangeMinus, Joints& jointRangePlus) const{
    jointRangeMinus = m_jointRangeMinus;
    jointRangePlus = m_jointRangePlus;
}

void RobotParameter::getJointMaxVelRange(Joints &jointMaxVel) const{
    jointMaxVel = m_jointMaxVel;
}

void RobotParameter::getJointMaxAccRange(Joints &jointMaxAcc) const{
    jointMaxAcc = m_jointMaxAcc;
}

void RobotParameter::getJointMaxJerkRange(Joints &jointMaxJerk) const{
    jointMaxJerk = m_jointMaxJerk;
}

void RobotParameter::getJointMaxVel(Joints& jointMaxVel) const{
    jointMaxVel = m_jointMaxVel*m_jointMaxVelRatio;
}

void RobotParameter::getJointMaxAcc(Joints& jointMaxAcc) const{
    jointMaxAcc = m_jointMaxAcc*m_jointMaxAccRatio;
}

void RobotParameter::getJointMaxJerk(Joints& jointMaxJerk) const{
    jointMaxJerk = m_jointMaxJerk*m_jointMaxJerkRatio;
}

void RobotParameter::getJointMaxVelRatio(double& jointMaxVelRatio) const{
    jointMaxVelRatio = m_jointMaxVelRatio;
}

void RobotParameter::getJointMaxAccRatio(double& jointMaxAccRatio) const{
    jointMaxAccRatio = m_jointMaxAccRatio;
}

void RobotParameter::getJointMaxJerkRatio(double& jointMaxJerkRatio) const{
    jointMaxJerkRatio = m_jointMaxJerkRatio;
}

void RobotParameter::getTerminalRange(Terminal& terminalRangeMinus, Terminal& terminalRangePlus) const{
    terminalRangeMinus = m_terminalRangeMinus;
    terminalRangePlus = m_terminalRangePlus;
}

void RobotParameter::getTerminalMaxVelRange(Terminal& terminalMaxVel) const{
    terminalMaxVel = m_terminalMaxVel;
}

void RobotParameter::getTerminalMaxAccRange(Terminal& terminalMaxAcc) const{
    terminalMaxAcc = m_terminalMaxAcc;
}

void RobotParameter::getTerminalMaxJerkRange(Terminal& terminalMaxJerk) const{
    terminalMaxJerk = m_terminalMaxJerk;
}

void RobotParameter::getTerminalMaxVel(Terminal& terminalMaxVel) const{
    terminalMaxVel = m_terminalMaxVel*m_terminalMaxVelRatio;
}

void RobotParameter::getTerminalMaxAcc(Terminal& terminalMaxAcc) const{
    terminalMaxAcc = m_terminalMaxAcc*m_terminalMaxAccRatio;
}

void RobotParameter::getTerminalMaxJerk(Terminal& terminalMaxJerk) const{
    terminalMaxJerk = m_terminalMaxJerk*m_terminalMaxJerkRatio;
}

void RobotParameter::getTerminalMaxVelRatio(double& terminalMaxVelRatio) const{
    terminalMaxVelRatio = m_terminalMaxVelRatio;
}

void RobotParameter::getTerminalMaxAccRatio(double& terminalMaxAccRatio) const{
    terminalMaxAccRatio = m_terminalMaxAccRatio;
}

void RobotParameter::getTerminalMaxJerkRatio(double& terminalMaxJerkRatio) const{
    terminalMaxJerkRatio = m_terminalMaxJerkRatio;
}

void RobotParameter::getDHParameter(Joints& DHParameterAlpha, Joints& DHParameterA, Joints& DHParameterD, Joints& DHParameterTheta) const{
    DHParameterAlpha = m_DHParameterAlpha;
    DHParameterA = m_DHParameterA;
    DHParameterD = m_DHParameterD;
    DHParameterTheta = m_DHParameterTheta;
}

void RobotParameter::getToolFrame(Terminal& toolFrame) const{
    toolFrame = m_toolFrame;
}

void RobotParameter::getWorkFrame(Terminal& workFrame) const{
    workFrame = m_workFrame;
}

ROBOTTYPE RobotParameter::getRobotType() const{
    return m_robotType;
}

double RobotParameter::getSamplePeriod() const{
    return m_samplePeriod;
}

Joints RobotParameter::getEncoderResolution() const{
    return m_encoderResolution;
}

Joints RobotParameter::getRateTorque() const{
    return m_rateTorque;
}

Joints RobotParameter::getReduceRatio() const{
    return m_reduceRatio;
}

int RobotParameter::getRobotDOF() const{
    return m_robotDOF;
}

int RobotParameter::getExternDOF() const{
    return m_externDOF;
}

int RobotParameter::getWholeDOF() const{
    return m_robotDOF+m_externDOF;
}

Joints RobotParameter::getJointRangeMinus() const{
    return m_jointRangeMinus;
}

Joints RobotParameter::getJointRangePlus() const{
    return m_jointRangePlus;
}

Joints RobotParameter::getJointMaxVelRange() const{
    return m_jointMaxVel;
}

Joints RobotParameter::getJointMaxAccRange() const{
    return m_jointMaxAcc;
}

Joints RobotParameter::getJointMaxJerkRange() const{
    return m_jointMaxJerk;
}

Joints RobotParameter::getJointMaxVel() const{
    return m_jointMaxVel*m_jointMaxVelRatio;
}

Joints RobotParameter::getJointMaxAcc() const{
    return m_jointMaxAcc*m_jointMaxAccRatio;
}

Joints RobotParameter::getJointMaxJerk() const{
    return m_jointMaxJerk*m_jointMaxJerkRatio;
}

Terminal RobotParameter::getTerminalRangeMinus() const{
    return m_terminalRangeMinus;
}

Terminal RobotParameter::getTerminalRangePlus() const{
    return m_terminalRangePlus;
}

Terminal RobotParameter::getTerminalMaxVelRange() const{
    return m_terminalMaxVel;
}

Terminal RobotParameter::getTerminalMaxAccRange() const{
    return m_terminalMaxAcc;
}

Terminal RobotParameter::getTerminalMaxJerkRange() const{
    return m_terminalMaxJerk;
}

Terminal RobotParameter::getTerminalMaxVel() const{
    return m_terminalMaxVel*m_terminalMaxVelRatio;
}

Terminal RobotParameter::getTerminalMaxAcc() const{
    return m_terminalMaxAcc*m_terminalMaxAccRatio;
}

Terminal RobotParameter::getTerminalMaxJerk() const{
    return m_terminalMaxJerk*m_terminalMaxJerkRatio;
}

Terminal RobotParameter::getToolFrame() const{
    return m_toolFrame;
}

Terminal RobotParameter::getWorkFrame() const{
    return m_workFrame;
}

bool RobotParameter::operator ==(const RobotParameter &robot) const{
    if(m_robotDOF != robot.m_robotDOF){
        return false;
    }
    if(m_externDOF != robot.m_externDOF){
        return false;
    }

    if(m_robotType != robot.m_robotType){
        return false;
    }
    if(!num_is_zero(m_samplePeriod-robot.m_samplePeriod)){
        return false;
    }

    if(m_encoderResolution != robot.m_encoderResolution){
        return false;
    }
    if(m_rateTorque != robot.m_rateTorque){
        return false;
    }
    if(m_reduceRatio != robot.m_reduceRatio){
        return false;
    }

    if(m_jointRangeMinus != robot.m_jointRangeMinus){
        return false;
    }
    if(m_jointRangePlus != robot.m_jointRangePlus){
        return false;
    }
    if(m_jointMaxVel != robot.m_jointMaxVel){
        return false;
    }
    if(m_jointMaxAcc != robot.m_jointMaxAcc){
        return false;
    }
    if(m_jointMaxJerk != robot.m_jointMaxJerk){
        return false;
    }
    if(!num_is_zero(m_jointMaxVelRatio-robot.m_jointMaxVelRatio)){
        return false;
    }
    if(!num_is_zero(m_jointMaxAccRatio-robot.m_jointMaxAccRatio)){
        return false;
    }
    if(!num_is_zero(m_jointMaxJerkRatio-robot.m_jointMaxJerkRatio)){
        return false;
    }

    if(m_terminalRangeMinus != robot.m_terminalRangeMinus){
        return false;
    }
    if(m_terminalRangePlus != robot.m_terminalRangePlus){
        return false;
    }
    if(m_terminalMaxVel != robot.m_terminalMaxVel){
        return false;
    }
    if(m_terminalMaxAcc != robot.m_terminalMaxAcc){
        return false;
    }
    if(m_terminalMaxJerk != robot.m_terminalMaxJerk){
        return false;
    }
    if(!num_is_zero(m_terminalMaxVelRatio-robot.m_terminalMaxVelRatio)){
        return false;
    }
    if(!num_is_zero(m_terminalMaxAccRatio-robot.m_terminalMaxAccRatio)){
        return false;
    }
    if(!num_is_zero(m_terminalMaxJerkRatio-robot.m_terminalMaxJerkRatio)){
        return false;
    }

    if(m_DHParameterAlpha != robot.m_DHParameterAlpha){
        return false;
    }
    if(m_DHParameterA != robot.m_DHParameterA){
        return false;
    }
    if(m_DHParameterD != robot.m_DHParameterD){
        return false;
    }
    if(m_DHParameterTheta != robot.m_DHParameterTheta){
        return false;
    }

    return true;
}

bool RobotParameter::operator !=(const RobotParameter &robot) const{
    return !(*this == robot);
}

bool RobotParameter::isDriverSame(const RobotParameter& robot) const{
    if(m_robotDOF != robot.m_robotDOF){
        return false;
    }
    if(m_externDOF != robot.m_externDOF){
        return false;
    }

    if(m_robotType != robot.m_robotType){
        return false;
    }
    if(!num_is_zero(m_samplePeriod-robot.m_samplePeriod)){
        return false;
    }

    if(m_encoderResolution != robot.m_encoderResolution){
        return false;
    }
    if(m_rateTorque != robot.m_rateTorque){
        return false;
    }
    if(m_reduceRatio != robot.m_reduceRatio){
        return false;
    }

    if(m_DHParameterAlpha != robot.m_DHParameterAlpha){
        return false;
    }
    if(m_DHParameterA != robot.m_DHParameterA){
        return false;
    }
    if(m_DHParameterD != robot.m_DHParameterD){
        return false;
    }
    if(m_DHParameterTheta != robot.m_DHParameterTheta){
        return false;
    }

    return true;
}

void RobotParameter::print() const{
    cout << "Robot Type: ";
    switch(m_robotType){
    case 0: cout << "ROBSOFT_SERIAL_SIX_CONVENTION";
        break;
    case 1: cout << "ROBSOFT_SERIAL_SIX_COOPERATION";
        break;
    case 2: cout << "ROBSOFT_SERIAL_FOUR_CONVENTION";
        break;
    case 3: cout << "ROBOTSOFT_SCARA_FOUR_ONERF";
        break;
    case 4: cout << "ROBOTSOFT_SCARA_FOUR_FOURRF";
        break;
    case 5: cout << "ROBOTSOFT_DELTA_FOUR";
        break;
    case 6: cout << "ROBOTSOFT_DELTA_SIX";
        break;
    default:
        break;
    }
    cout << endl << endl;

    cout << "Sample Period(ms): " << m_samplePeriod << endl << endl;

    cout << "Robot DOF: " << m_robotDOF << endl;
    cout << "External DOF: " << m_externDOF << endl;
    cout << endl;

    m_encoderResolution.print("Encoder Resolution");
    cout << endl;

    m_rateTorque.print("Rate Torque");
    cout << endl;

    m_reduceRatio.print("Reduce Ratio");
    cout << endl;

    m_jointRangeMinus.print("Joints Range Minus");
    cout << endl;

    m_jointRangePlus.print("Joints Range Plus");
    cout << endl;

    m_jointMaxVel.print("Joints Max Vel");
    cout << endl;

    m_jointMaxAcc.print("Joints Max Acc");
    cout << endl;

    m_jointMaxJerk.print("Joints Max Jerk");
    cout << endl;

    cout << "Joint Max Vel Ratio: " << m_jointMaxVelRatio << endl;
    cout << "Joint Max Acc Ratio: " << m_jointMaxAccRatio << endl;
    cout << "Joint Max Jerk Ratio: " << m_jointMaxJerkRatio << endl;
    cout << endl;

    m_terminalRangeMinus.print("Terminal Range Minus");
    cout << endl;

    m_terminalRangePlus.print("Terminal Range Plus");
    cout << endl;

    m_terminalMaxVel.print("Terminal Max Vel");
    cout << endl;

    m_terminalMaxAcc.print("Terminal Max Acc");
    cout << endl;

    m_terminalMaxJerk.print("Terminal Max Jerk");
    cout << endl;

    cout << "Terminal Max Vel Ratio: " << m_terminalMaxVelRatio << endl;
    cout << "Terminal Max Acc Ratio: " << m_terminalMaxAccRatio << endl;
    cout << "Terminal Max Jerk Ratio: " << m_terminalMaxJerkRatio << endl;
    cout << endl;

    m_DHParameterAlpha.print("DH Parameter Alpha");
    cout << endl;

    m_DHParameterA.print("DH Parameter A");
    cout << endl;

    m_DHParameterD.print("DH Parameter D");
    cout << endl;

    m_DHParameterTheta.print("DH Parameter Theta");
    cout << endl;

    m_toolFrame.print("Tool Frame");
    cout << endl;

    m_workFrame.print("Work Frame");
    cout << endl;
}

void RobotParameter::readRobotParameter(const char* path){
    xmlDocPtr xmlDoc = xmlReadFile(path, "UTF-8", XML_PARSE_NOBLANKS);
    xmlNodePtr rootNode = xmlDocGetRootElement(xmlDoc);

    xmlNodePtr ptrNode = rootNode->children;
    while(ptrNode != NULL){
        int tmp;

        xml_judge_read_num("m_robotDOF", ptrNode, m_robotDOF);
        xml_judge_read_num("m_externDOF", ptrNode, m_externDOF);
        xml_judge_read_num("m_robotType", ptrNode, tmp);
        m_robotType = ROBOTTYPE(tmp);
        xml_judge_read_num("m_samplePeriod", ptrNode, m_samplePeriod);

        xml_judge_read_joints("m_encoderResolution", ptrNode, m_encoderResolution);
        xml_judge_read_joints("m_rateTorque", ptrNode, m_rateTorque);
        xml_judge_read_joints("m_reduceRatio", ptrNode, m_reduceRatio);

        xml_judge_read_joints("m_jointRangeMinus", ptrNode, m_jointRangeMinus);
        xml_judge_read_joints("m_jointRangePlus", ptrNode, m_jointRangePlus);
        xml_judge_read_joints("m_jointMaxVel", ptrNode, m_jointMaxVel);
        xml_judge_read_joints("m_jointMaxAcc", ptrNode, m_jointMaxAcc);
        xml_judge_read_joints("m_jointMaxJerk", ptrNode, m_jointMaxJerk);
        xml_judge_read_num("m_jointMaxVelRatio", ptrNode, m_jointMaxVelRatio);
        xml_judge_read_num("m_jointMaxAccRatio", ptrNode, m_jointMaxAccRatio);
        xml_judge_read_num("m_jointMaxJerkRatio", ptrNode, m_jointMaxJerkRatio);

        xml_judge_read_terminal("m_terminalRangeMinus", ptrNode, m_terminalRangeMinus);
        xml_judge_read_terminal("m_terminalRangePlus", ptrNode, m_terminalRangePlus);
        xml_judge_read_terminal("m_terminalMaxVel", ptrNode, m_terminalMaxVel);
        xml_judge_read_terminal("m_terminalMaxAcc", ptrNode, m_terminalMaxAcc);
        xml_judge_read_terminal("m_terminalMaxJerk", ptrNode, m_terminalMaxJerk);
        xml_judge_read_num("m_terminalMaxVelRatio", ptrNode, m_terminalMaxVelRatio);
        xml_judge_read_num("m_terminalMaxAccRatio", ptrNode, m_terminalMaxAccRatio);
        xml_judge_read_num("m_terminalMaxJerkRatio", ptrNode, m_terminalMaxJerkRatio);

        xml_judge_read_joints("m_DHParameterAlpha", ptrNode, m_DHParameterAlpha);
        xml_judge_read_joints("m_DHParameterA", ptrNode, m_DHParameterA);
        xml_judge_read_joints("m_DHParameterD", ptrNode, m_DHParameterD);
        xml_judge_read_joints("m_DHParameterTheta", ptrNode, m_DHParameterTheta);

        ptrNode = ptrNode->next;
    }

    xmlFreeDoc(xmlDoc);
}

void RobotParameter::writeRobotParameter(const char* path) const{
    xmlDocPtr xmlDoc = xmlNewDoc((const xmlChar*)"1.0");
    xmlNodePtr rootNode = xmlNewNode(0, BAD_CAST "robot");
    xmlDocSetRootElement(xmlDoc, rootNode);

    xml_addchild_num("m_robotDOF", m_robotDOF, rootNode);
    xml_addchild_num("m_externDOF", m_externDOF, rootNode);
    xml_addchild_num("m_robotType", m_robotType, rootNode);
    xml_addchild_num("m_samplePeriod", m_samplePeriod, rootNode);

    xml_addchild_joints("m_encoderResolution", m_encoderResolution, rootNode);
    xml_addchild_joints("m_rateTorque", m_rateTorque, rootNode);
    xml_addchild_joints("m_reduceRatio", m_reduceRatio, rootNode);

    xml_addchild_joints("m_jointRangeMinus", m_jointRangeMinus, rootNode);
    xml_addchild_joints("m_jointRangePlus", m_jointRangePlus, rootNode);
    xml_addchild_joints("m_jointMaxVel", m_jointMaxVel, rootNode);
    xml_addchild_joints("m_jointMaxAcc", m_jointMaxAcc, rootNode);
    xml_addchild_joints("m_jointMaxJerk", m_jointMaxJerk, rootNode);
    xml_addchild_num("m_jointMaxVelRatio", m_jointMaxVelRatio, rootNode);
    xml_addchild_num("m_jointMaxAccRatio", m_jointMaxAccRatio, rootNode);
    xml_addchild_num("m_jointMaxJerkRatio", m_jointMaxJerkRatio, rootNode);

    xml_addchild_terminal("m_terminalRangeMinus", m_terminalRangeMinus, rootNode);
    xml_addchild_terminal("m_terminalRangePlus", m_terminalRangePlus, rootNode);
    xml_addchild_terminal("m_terminalMaxVel", m_terminalMaxVel, rootNode);
    xml_addchild_terminal("m_terminalMaxAcc", m_terminalMaxAcc, rootNode);
    xml_addchild_terminal("m_terminalMaxJerk", m_terminalMaxJerk, rootNode);
    xml_addchild_num("m_terminalMaxVelRatio", m_terminalMaxVelRatio, rootNode);
    xml_addchild_num("m_terminalMaxAccRatio", m_terminalMaxAccRatio, rootNode);
    xml_addchild_num("m_terminalMaxJerkRatio", m_terminalMaxJerkRatio, rootNode);

    xml_addchild_joints("m_DHParameterAlpha", m_DHParameterAlpha, rootNode);
    xml_addchild_joints("m_DHParameterA", m_DHParameterA, rootNode);
    xml_addchild_joints("m_DHParameterD", m_DHParameterD, rootNode);
    xml_addchild_joints("m_DHParameterTheta", m_DHParameterTheta, rootNode);

    xmlSaveFormatFileEnc(path, xmlDoc, "UTF-8", 1);
    xmlFreeDoc(xmlDoc);
}

#endif
