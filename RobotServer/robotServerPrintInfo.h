#ifndef ROBOTSERVERPRINTINFO_H
#define ROBOTSERVERPRINTINFO_H

#include <vector>
#include "robotStructure.h"

namespace rclib{

class RobotServerPrintInfo{
public:
    RobotServerPrintInfo();
    ~RobotServerPrintInfo();

    void printServoOn();
    void printServoOff();
    void printDriverParameterChanged();
    void printEmergencySwitchOn();
    void printDriverError();
    void printJointMoveToLimit();
    void printWaitCommandEnd();
    void printNeverStartConTraj();
    void printTCPPointsNumber();
    void printMoveToZeroResult(bool);

protected:
    ERROR_INIF_LANGUAGE languageType(ERROR_INIF_LANGUAGE type);
    ERROR_INIF_LANGUAGE languageType();
    enum ERROR_SERIAL{SERVOON=0, SERVOOFF, DRIVERPARAMETERCHANGED, EMERGENCYSWITCHON,
                     DRIVERERROR, JOINTMOVETOLIMIT, WAITCOMMANDEND, NEVERSTARTCONTRAJ,
                     TCPPOINTSNUMBER, MOVETOZERORESULT_FALSE, MOVETOZERORESULT_TRUE};
private:
    virtual void printInfo(INFOSTATE infoType, std::string infoString){}
    ERROR_INIF_LANGUAGE m_languageType;
    std::map<ERROR_SERIAL,std::string> m_errorInfo;

};

}

#endif
