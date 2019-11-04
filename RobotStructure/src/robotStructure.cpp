#include "RobSoft/CFileIO.hpp"
#include "robotStructure.h"
#include <stdio.h>
#include <cmath>
#include <sstream>
//#include "baselib.h"

using namespace std;
using namespace rclib;

void DigitalInputState::init(){
    for(int i=0; i<DIGITAL_INPUTPORT_NUM; i++){
        m_state[i] = SWITCHOFF;
    }
}

int DigitalInputState::size(){
    return DIGITAL_INPUTPORT_NUM;
}

SWITCHSTATE& DigitalInputState::operator [](PORTINDEX index){
    return m_state[index];
}

const SWITCHSTATE& DigitalInputState::operator [](PORTINDEX index) const{
    return m_state[index];
}

bool DigitalInputState::operator ==(const DigitalInputState &state) const{
    for(int i=0; i<DIGITAL_INPUTPORT_NUM; i++){
        if(m_state[i] != state.m_state[i]){
            return false;
        }
    }
    return true;
}

bool DigitalInputState::operator !=(const DigitalInputState &state) const{
    return !(*this == state);
}

void DigitalOutputState::init(){
    for(int i=0; i<DIGITAL_OUTPUTPORT_NUM; i++){
        m_state[i] = SWITCHOFF;
    }
}

int DigitalOutputState::size(){
    return DIGITAL_OUTPUTPORT_NUM;
}

SWITCHSTATE& DigitalOutputState::operator [](PORTINDEX index){
    return m_state[index];
}

const SWITCHSTATE& DigitalOutputState::operator [](PORTINDEX index) const{
    return m_state[index];
}

bool DigitalOutputState::operator ==(const DigitalOutputState &state) const{
    for(int i=0; i<DIGITAL_OUTPUTPORT_NUM; i++){
        if(m_state[i] != state.m_state[i]){
            return false;
        }
    }
    return true;
}

bool DigitalOutputState::operator !=(const DigitalOutputState &state) const{
    return !(*this == state);
}

void AnalogInputState::init(){
    for(int i=0; i<ANALOG_INPUT_NUM; i++){
        m_state[i] = 0;
    }
}

int AnalogInputState::size(){
    return ANALOG_INPUT_NUM;
}

double& AnalogInputState::operator [](PORTINDEX index){
    return m_state[index];
}

const double& AnalogInputState::operator [](PORTINDEX index) const{
    return m_state[index];
}

void AnalogOutputState::init(){
    for(int i=0; i<ANALOG_OUTPUT_NUM; i++){
        m_state[i] = 0;
    }
}

int AnalogOutputState::size(){
    return ANALOG_OUTPUT_NUM;
}

double& AnalogOutputState::operator [](PORTINDEX index){
    return m_state[index];
}

const double& AnalogOutputState::operator [](PORTINDEX index) const{
    return m_state[index];
}


RobotIO::RobotIO(){
    m_IOState = SWITCHOFF;
    m_DIState.init();
    m_DOState.init();
    m_AIState.init();
    m_AOState.init();
}

RobotIO::~RobotIO(){

}

SWITCHSTATE RobotIO::getIOConnectState() const{
    return m_IOState;
}

void RobotIO::setIOConnectState(SWITCHSTATE state){
    m_IOState = state;
}

DigitalInputState RobotIO::getDigitalInputState() const{
    return m_DIState;
}

SWITCHSTATE RobotIO::getDigitalInputState(PORTINDEX index) const
{
    return m_DIState[index];
}

void RobotIO::setDigitalInputState(const DigitalInputState& state){
    m_DIState = state;
}

DigitalOutputState RobotIO::getDigitalOutputState() const{
    return m_DOState;
}

SWITCHSTATE RobotIO::getDigitalOutputState(PORTINDEX index) const
{
    return m_DOState[index];
}

void RobotIO::setDigitalOutputState(const DigitalOutputState& state){
    m_DOState = state;
}

AnalogInputState RobotIO::getAnalogInputState() const{
    return m_AIState;
}

void RobotIO::setAnalogInputState(const AnalogInputState& state){
    m_AIState = state;
}

AnalogOutputState RobotIO::getAnalogOutputState() const{
    return m_AOState;
}

void RobotIO::setAnalogOutputState(const AnalogOutputState& state){
    m_AOState = state;
}


RobotState::RobotState(){
    m_runStateFlag = SYSRUN_STOP;
    m_playStateFlag = SYSPLAY_TEACH;
    m_servoStateFlag = SWITCHOFF;
    m_virtualStateFlag = SWITCHOFF;
    m_debugStateFlag = SWITCHOFF;
    m_vel = 0.05;
}

RobotState::~RobotState(){

}

SYSRUNSTATE RobotState::getRunState() const{
    return m_runStateFlag;
}

void RobotState::setRunState(SYSRUNSTATE state){
    m_runStateFlag = state;
}

SYSPLAYSTATE RobotState::getPlayState() const{
    return m_playStateFlag;
}

void RobotState::setPlayState(SYSPLAYSTATE state){
    m_playStateFlag = state;
}

SWITCHSTATE RobotState::getServoState() const{
    return m_servoStateFlag;
}

void RobotState::setServoState(SWITCHSTATE state){
    m_servoStateFlag = state;
}

SWITCHSTATE RobotState::getVirtualState() const{
    SWITCHSTATE state = m_virtualStateFlag;
    return state;
}

void RobotState::setVirtualState(SWITCHSTATE state){
    m_virtualStateFlag = state;
}

SWITCHSTATE RobotState::getDebugState() const{
    return m_debugStateFlag;
}

void RobotState::setDebugState(SWITCHSTATE state){
    m_debugStateFlag = state;
}

double RobotState::getVel() const{
    return m_vel;
}

void RobotState::setVel(double vel){
    m_vel = vel;
}


RobotInfo::RobotInfo(){

}

RobotInfo::~RobotInfo(){

}

void RobotInfo::getInfo(INFOSTATE &infoType, string &infoString){
    if(m_infoType.empty()){
        infoType = INFO_NONE;
    }
    if(m_infoString.empty()&&m_infoType.empty())
    {
        return ;
    }

    m_mutexInfo.lockMutex();
    infoType = m_infoType.front();
    infoString = m_infoString.front();
    m_infoType.pop_front();
    m_infoString.pop_front();
    m_mutexInfo.unlockMutex();
}

void RobotInfo::setInfo(INFOSTATE infoType, string infoString){
    m_mutexInfo.lockMutex();
    m_infoType.push_back(infoType);
    m_infoString.push_back(infoString);
    while(m_infoType.size() > 10){
        m_infoType.pop_front();
        m_infoString.pop_front();
    }
    m_mutexInfo.unlockMutex();
}


RobotFrame::RobotFrame(){

}

RobotFrame::~RobotFrame(){

}

void RobotFrame::readRobotFrame(const char *path){
    xmlDocPtr xmlDoc = xmlReadFile(path, "UTF-8", XML_PARSE_NOBLANKS);
    xmlNodePtr rootNode = xmlDocGetRootElement(xmlDoc);

    if(!rootNode)
        return;

    m_toolFrameList.clear();
    m_workFrameList.clear();

    xmlNodePtr ptrNode = rootNode->children;
    while(ptrNode != NULL){
        xml_judge_read_string("m_currentToolFrame", ptrNode, m_currentToolFrame);
        xml_judge_read_string("m_currentWorkFrame", ptrNode, m_currentWorkFrame);

        xml_judge_read_terminal_map("m_toolFrameList", ptrNode, m_toolFrameList);
        xml_judge_read_terminal_map("m_workFrameList", ptrNode, m_workFrameList);

        ptrNode = ptrNode->next;
    }

    xmlFreeDoc(xmlDoc);
}

void RobotFrame::writeRobotFrame(const char *path){
    xmlDocPtr xmlDoc = xmlNewDoc((const xmlChar*)"1.0");
    xmlNodePtr rootNode = xmlNewNode(0, BAD_CAST "robot_frame");
    xmlDocSetRootElement(xmlDoc, rootNode);

    xml_addchild_string("m_currentToolFrame", m_currentToolFrame, rootNode);
    xml_addchild_string("m_currentWorkFrame", m_currentWorkFrame, rootNode);
    xml_addchild_terminal_map("m_toolFrameList", m_toolFrameList, rootNode);
    xml_addchild_terminal_map("m_workFrameList", m_workFrameList, rootNode);

    xmlSaveFormatFileEnc(path, xmlDoc, "UTF-8", 1);
    xmlFreeDoc(xmlDoc);
}

map<string, Terminal> RobotFrame::getToolFrameList() const{
    return m_toolFrameList;
}

string RobotFrame::getCurrentToolFrame() const{
    return m_currentToolFrame;
}

Terminal RobotFrame::getToolFrame(string name){
    return m_toolFrameList[name];
}

int RobotFrame::addToolFrame(string name, const Terminal& frame){
    if(name.empty()){   // 坐标系名为空
        return 1;
    }

    if(m_toolFrameList.find(name) != m_toolFrameList.end()){    // 坐标系已存在
        return 1;
    }

    m_toolFrameList[name] = frame;
    return 0;
}

int RobotFrame::modifyToolFrame(string name, const Terminal& frame){
    if(name.empty()){   // 坐标系名为空
        return 1;
    }

    if(name == string("Defalut")){  // 默认坐标系不能修改
        return 1;
    }

    m_toolFrameList[name] = frame;
    return 0;
}

int RobotFrame::deleteToolFrame(string name){
    if(name.empty()){   // 坐标系名为空
        return 1;
    }

    if(m_toolFrameList.find(name) == m_toolFrameList.end()){    // 坐标系不存在
        return 0;
    }

    if(name == string("Defalut")){  // 默认坐标系不能修改
        return 1;
    }

    if(name == m_currentToolFrame){
        m_currentToolFrame = m_defaultName;
    }

    m_toolFrameList.erase(name);
    return 0;
}

void RobotFrame::clearToolFrame(){
    m_toolFrameList.clear();
    m_currentToolFrame = m_defaultName;
    setDefaultToolFrame();
}

int RobotFrame::setCurrentToolFrame(string name){
    if(m_toolFrameList.find(name) == m_toolFrameList.end()){    // 坐标系不存在
        return 1;
    }

    m_currentToolFrame = name;
    return 0;
}

void RobotFrame::setDefaultToolFrame(){
    m_toolFrameList[m_defaultName] = Terminal(0, 0, 0, 0, 0, 0);
}

map<string, Terminal> RobotFrame::getWorkFrameList() const{
    return m_workFrameList;
}

string RobotFrame::getCurrentWorkFrame() const{
    return m_currentWorkFrame;
}

Terminal RobotFrame::getWorkFrame(string name){
    return m_workFrameList[name];
}

int RobotFrame::addWorkFrame(string name, const Terminal& frame){
    if(name.empty()){   // 坐标系名为空
        return 1;
    }

    if(m_workFrameList.find(name) != m_workFrameList.end()){    // 坐标系已存在
        return 1;
    }

    m_workFrameList[name] = frame;
    return 0;
}

int RobotFrame::modifyWorkFrame(string name, const Terminal& frame){
    if(name.empty()){   // 坐标系名为空
        return 1;
    }

    if(name == m_defaultName){  // 默认坐标系不能修改
        return 1;
    }

    m_workFrameList[name] = frame;
    return 0;
}

int RobotFrame::deleteWorkFrame(string name){
    if(name.empty()){   // 坐标系名为空
        return 1;
    }

    if(m_workFrameList.find(name) == m_workFrameList.end()){    // 坐标系不存在
        return 0;
    }

    if(name == string("Defalut")){  // 默认坐标系不能修改
        return 1;
    }

    if(name == m_currentWorkFrame){
        m_currentWorkFrame = m_defaultName;
    }

    m_workFrameList.erase(name);
    return 0;
}

void RobotFrame::clearWorkFrame(){
    m_workFrameList.clear();
    m_currentWorkFrame = m_defaultName;
    setDefaultWorkFrame();
}

int RobotFrame::setCurrentWorkFrame(string name){
    if(m_workFrameList.find(name) == m_workFrameList.end()){    // 坐标系不存在
        return 1;
    }

    m_currentWorkFrame = name;
    return 0;
}

void RobotFrame::setDefaultWorkFrame(){
    m_workFrameList[m_defaultName] = Terminal(0, 0, 0, 0, 0, 0);
}

bool RobotFrame::operator ==(RobotFrame frame){
    for(map<string, Terminal>::iterator it = m_toolFrameList.begin(); it != m_toolFrameList.end(); it++){
        if(frame.m_toolFrameList.find(it->first) == frame.m_toolFrameList.end()){
            return false;
        }
        if(it->second != frame.m_toolFrameList[it->first]){
            return false;
        }
    }
    for(map<string, Terminal>::iterator it = m_workFrameList.begin(); it != m_workFrameList.end(); it++){
        if(frame.m_workFrameList.find(it->first) == frame.m_workFrameList.end()){
            return false;
        }
        if(it->second != frame.m_workFrameList[it->first]){
            return false;
        }
    }
    return true;
}

bool RobotFrame::operator !=(RobotFrame frame){
    return !(*this == frame);
}

RobotPreference::RobotPreference(){

}

RobotPreference::~RobotPreference(){

}

void RobotPreference::readRobotPreference(const char* path){
    xmlDocPtr xmlDoc = xmlReadFile(path, "UTF-8", XML_PARSE_NOBLANKS);
    xmlNodePtr rootNode = xmlDocGetRootElement(xmlDoc);
    if(!rootNode)
    {
        return ;
    }

    xmlNodePtr ptrNode = rootNode->children;
    while(ptrNode != NULL){
        xml_judge_read_joints("m_initJointPosition", ptrNode, m_initJointPosition);
        xml_judge_read_joints("m_jointCompensation", ptrNode, m_jointCompensation);
        xml_judge_read_joints("m_jointReturnSequence", ptrNode, m_jointReturnSequence);

        ptrNode = ptrNode->next;
    }

    xmlFreeDoc(xmlDoc);
}

void RobotPreference::writeRobotPreference(const char* path) const{
    xmlDocPtr xmlDoc = xmlNewDoc((const xmlChar*)"1.0");
    xmlNodePtr rootNode = xmlNewNode(0, BAD_CAST "robot_preference");
    xmlDocSetRootElement(xmlDoc, rootNode);

    xml_addchild_joints("m_initJointPosition", m_initJointPosition, rootNode);
    xml_addchild_joints("m_jointCompensation", m_jointCompensation, rootNode);
    xml_addchild_joints("m_jointReturnSequence", m_jointReturnSequence, rootNode);

    xmlSaveFormatFileEnc(path, xmlDoc, "UTF-8", 1);
    xmlFreeDoc(xmlDoc);
}

Joints RobotPreference::getInitJointPosition() const{
    return m_initJointPosition;
}

void RobotPreference::setInitJointPosition(const Joints& joint){
    m_initJointPosition = joint;
}

Joints RobotPreference::getJointCompensation() const{
    return m_jointCompensation;
}

void RobotPreference::setJointCompensation(const Joints& joint){
    m_jointCompensation = joint;
}

Joints RobotPreference::getJointReturnSequence() const{
    return m_jointReturnSequence;
}

void RobotPreference::setJointReturnSequence(const Joints& joint){
    m_jointReturnSequence = joint;
}

bool RobotPreference::operator ==(const RobotPreference &prefer) const{
    if(m_initJointPosition != prefer.m_initJointPosition){
        return false;
    }
    if(m_jointCompensation != prefer.m_jointCompensation){
        return false;
    }
    if(m_jointReturnSequence != prefer.m_jointReturnSequence){
        return false;
    }
    return true;
}

bool RobotPreference::operator !=(const RobotPreference &prefer) const{
    return !(*this == prefer);
}

MotionNode::MotionNode(){
    accRatio = 0.8;
    jerkRatio = 0.8;
    angleRatio = 0.8;
    precision = 0.01;
    referFrame = COORDINATE_BASE;
    circleType = PART_CIRCLE;
    height = 10;
    delayNum = 1;
}

MotionNode::~MotionNode(){

}

namespace rclib {

string num_to_string(double num){
    stringstream ss;
    ss.precision(15);
    ss << num;
    return ss.str();
}

string num_to_string(int num){
    stringstream ss;
    ss << num;
    return ss.str();
}

double string_to_double(const string& str){
    stringstream ss;
    ss << str;
    double num;
    ss >> num;
    return num;
}

int string_to_int(const string& str){
    stringstream ss;
    ss << str;
    double num;
    ss >> num;
    return num;
}

}
