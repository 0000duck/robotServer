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
        m_errorInfo[SERVOON]="�ŷ�״̬�Ѵ�";
        m_errorInfo[SERVOOFF]="�ŷ�״̬δ��";
        m_errorInfo[DRIVERPARAMETERCHANGED]="����������޸ģ�����������ϵͳ";
        m_errorInfo[EMERGENCYSWITCHON]="��ͣ�����Ѱ���";
        m_errorInfo[DRIVERERROR]="���������ִ���";
        m_errorInfo[JOINTMOVETOLIMIT]="�ؽ��˶�����λ";
        m_errorInfo[WAITCOMMANDEND]="��һ������δִ�����";
        m_errorInfo[NEVERSTARTCONTRAJ]="���ȵ��ÿ�ʼ�����켣��������";
        m_errorInfo[TCPPOINTSNUMBER]="TCP�궨��Ҫ3-10����";
        m_errorInfo[MOVETOZERORESULT_FALSE]="��λ�������ʧ�ܣ������»���";
        m_errorInfo[MOVETOZERORESULT_TRUE]= "��λ������̳ɹ�";
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
