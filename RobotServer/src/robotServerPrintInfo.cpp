#include "robotServerPrintInfo.h"

using namespace std;
using namespace rclib;

RobotServerPrintInfo::RobotServerPrintInfo(){
    languageType(CHINESE);
}

RobotServerPrintInfo::~RobotServerPrintInfo(){

}

void RobotServerPrintInfo::printServoOn(){
    printInfo(INFO_ERROR, m_errorInfo[SERVOON]);
}

void RobotServerPrintInfo::printServoOff(){
    printInfo(INFO_ERROR, m_errorInfo[SERVOOFF]);
}

void RobotServerPrintInfo::printDriverParameterChanged(){
    printInfo(INFO_ERROR, m_errorInfo[DRIVERPARAMETERCHANGED]);
}

void RobotServerPrintInfo::printEmergencySwitchOn(){
    printInfo(INFO_ERROR, m_errorInfo[EMERGENCYSWITCHON]);
}

void RobotServerPrintInfo::printDriverError(){
    printInfo(INFO_ERROR, m_errorInfo[DRIVERERROR]);
}

void RobotServerPrintInfo::printJointMoveToLimit(){
    printInfo(INFO_ERROR, m_errorInfo[JOINTMOVETOLIMIT]);
}

void RobotServerPrintInfo::printWaitCommandEnd(){
    printInfo(INFO_WARNING, m_errorInfo[WAITCOMMANDEND]);
}

void RobotServerPrintInfo::printNeverStartConTraj(){
    printInfo(INFO_WARNING, m_errorInfo[NEVERSTARTCONTRAJ]);
}

void RobotServerPrintInfo::printTCPPointsNumber(){
    printInfo(INFO_WARNING, m_errorInfo[TCPPOINTSNUMBER]);
}

void RobotServerPrintInfo::printMoveToZeroResult(bool state){
    if(state){
        printInfo(INFO_RECOM, m_errorInfo[MOVETOZERORESULT_TRUE]);
    }
    else{
        printInfo(INFO_ERROR, m_errorInfo[MOVETOZERORESULT_FALSE]);
    }
}

ERROR_INIF_LANGUAGE RobotServerPrintInfo::languageType(ERROR_INIF_LANGUAGE type)
{
    m_languageType = type;
    if(type==CHINESE)
    {
        m_errorInfo[SERVOON]="伺服状态已打开";
        m_errorInfo[SERVOOFF]="伺服状态未打开";
        m_errorInfo[DRIVERPARAMETERCHANGED]="本体参数已修改，请重启控制系统";
        m_errorInfo[EMERGENCYSWITCHON]="急停开关已按下";
        m_errorInfo[DRIVERERROR]="驱动器出现错误";
        m_errorInfo[JOINTMOVETOLIMIT]="关节运动到限位";
        m_errorInfo[WAITCOMMANDEND]="上一条命令未执行完成";
        m_errorInfo[NEVERSTARTCONTRAJ]="清先调用开始连续轨迹运行命令";
        m_errorInfo[TCPPOINTSNUMBER]="TCP标定需要3-10个点";
        m_errorInfo[MOVETOZERORESULT_FALSE]="限位回零过程失败，请重新回零";
        m_errorInfo[MOVETOZERORESULT_TRUE]= "限位回零过程成功";
    }else {
        m_errorInfo[SERVOON]="Servo is on";
        m_errorInfo[SERVOOFF]="Servo is off";
        m_errorInfo[DRIVERPARAMETERCHANGED]="Ontology parameters have been modified, please restart the control system";
        m_errorInfo[EMERGENCYSWITCHON]="The emergency stop switch has been pressed down";
        m_errorInfo[DRIVERERROR]="Driver error";
        m_errorInfo[JOINTMOVETOLIMIT]="The joints move to the limit";
        m_errorInfo[WAITCOMMANDEND]="The previous command was not completed";
        m_errorInfo[NEVERSTARTCONTRAJ]="Clear first call to start the continuous trajectory command";
        m_errorInfo[TCPPOINTSNUMBER]="TCP calibration requires 3-10 points";
        m_errorInfo[MOVETOZERORESULT_FALSE]="Limit reset failed. Please reset";
        m_errorInfo[MOVETOZERORESULT_TRUE]="Limit back to zero process successful";
    }
    return m_languageType;
}

ERROR_INIF_LANGUAGE RobotServerPrintInfo::languageType()
{
    return m_languageType;
}
