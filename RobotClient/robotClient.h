#ifndef ROBOTCLIENT_H
#define ROBOTCLIENT_H

#include "robotClientBase.h"
#include "robotStructure.h"

namespace rclib{

MY_CLASS RobotClient
        :public RobotClientBase
{
private:
    RobotClient();
    ~RobotClient();
public:
    static RobotClient* m_robotClient;
    static RobotClient* initance();
    static void delInittance();

    bool initSystem(const char* server_ip = "127.0.0.1", int server_port = 8080);
    bool controlSystem();

    void setErrorClear();    // 清除错误
    bool setJointZero(JOINTINDEX joint); // 置零

    int returnZero(double vel = 0.1); // 回到初始位置
    int moveToZero(double vel = 0.05); // 传感器回零
    int jointJOG(JOINTINDEX index, MOVEDIRECTION dir, double vel);  // 关节点动
    int terminalJOG(TERMINALINDEX index, MOVEDIRECTION dir, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);   // 末端点动
    void stopJOG(); // 停止点动
    int jointStep(JOINTINDEX index, MOVEDIRECTION dir, double step, double vel);  // 关节步进
    int terminalStep(TERMINALINDEX index, MOVEDIRECTION dir, double step, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);   // 末端步进

    int moveABSJoint(const Joints& ps, double vel);
    int moveABSJoint(const JointsList& ps, double vel);
    int moveABSJoint(const JointsList& ps, std::vector<double>& tm);
    int moveABSJointR(const Joints& ps, double vel);
    int moveABSJointR(const JointsList &ps, double vel);
    int moveABSJointR(const JointsList &ps, std::vector<double> &tm);

    int moveJoint(const Terminal& p, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveJoint(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveJointR(const Terminal& p, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveJointR(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE);

    int moveLine(const HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveLine(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveLineR(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveCircle(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveCircleR(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE);

    int moveLineCon(const HomogeneousMatrix& m, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveLineCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveLineRCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveCircleCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveCircleRCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE);
    int moveStartCon(int delay = 2);
    int moveEndCon();

    int moveCurve(const TerminalList& ps, std::vector<double>& vel, double acc = 0.8, double jerk = 0.8, double angle = 0.8, double bpre = 0.01, COORDINATESYSTEM frame = COORDINATE_BASE);

    int moveJump(const HomogeneousMatrix& m, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3);    // 门运动
    int moveJump(const Terminal& p, double height, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3);

    int waitCommandEnd();

    void dragModeStart();
    void dragModeEnd();
    bool dragModeTimeStart(int period_ms, int timeLength_s);
    bool dragModePlayPrepare(double vel);
    bool dragModePlay(int period_ms);

    bool programVelocity(double vel);
    void programLoad(const char* path);
    void programRun();
    void programPause(bool bPause=true);
    void programStop();
    void programStepRun();
    void programStepPause();
    void setProgramPointer(const ProgramPointer &pointer);  // 设置程序指针

    void testFunctionButton();
    ProgramPointer m_breakPointer;          //断点
};

}

#endif
