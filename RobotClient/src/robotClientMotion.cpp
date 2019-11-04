#include "robotClient.h"

using namespace std;
using namespace rclib;

int RobotClient::returnZero(double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    moveABSJoint(getRobotPreference().getInitJointPosition(), vel);

    return waitCMDRightFlag();
}

int RobotClient::moveToZero(double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    sendXWHL(vel);

    return waitCMDRightFlag();
}

int RobotClient::jointJOG(JOINTINDEX joint, MOVEDIRECTION dir, double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    sendGJDD(joint, dir, vel);

    return waitCMDRightFlag();
}

int RobotClient::terminalJOG(TERMINALINDEX terminal, MOVEDIRECTION dir, double vel, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    sendMDDD(terminal, dir, vel, frame);

    return waitCMDRightFlag();
}

void RobotClient::stopJOG(){
    sendDDTZ();
}

int RobotClient::jointStep(JOINTINDEX joint, MOVEDIRECTION dir, double step, double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    sendGJBJ(joint, dir, step, vel);

    return waitCMDRightFlag();
}

int RobotClient::terminalStep(TERMINALINDEX terminal, MOVEDIRECTION dir, double step, double vel, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    sendMDBJ(terminal, dir, step, vel, frame);

    return waitCMDRightFlag();
}

int RobotClient::moveABSJoint(const Joints &ps, double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    MotionNode node;
    node.motionType = MOTION_MOVEABSJ;
    node.jointList.push_back(ps);
    node.velList.push_back(vel);
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveABSJoint(const JointsList &ps, double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    MotionNode node;
    node.motionType = MOTION_MOVEABSJ;
    node.jointList = ps;
    node.velList.push_back(vel);
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveABSJoint(const JointsList &ps, vector<double>& tm){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(ps.size() != tm.size()){
        printf("Error: conflicting parameter number\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEABSJ_TIME;
    node.jointList = ps;
    node.velList = tm;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveABSJointR(const Joints &ps, double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    MotionNode node;
    node.motionType = MOTION_MOVEABSJR;
    node.jointList.push_back(ps);
    node.velList.push_back(vel);
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveABSJointR(const JointsList &ps, double vel){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    MotionNode node;
    node.motionType = MOTION_MOVEABSJR;
    node.jointList = ps;
    node.velList.push_back(vel);
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveABSJointR(const JointsList &ps, vector<double> &tm){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(ps.size() != tm.size()){
        printf("Error: conflicting parameter number\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEABSJR_TIME;
    node.jointList = ps;
    node.velList = tm;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveJoint(const Terminal &p, double vel, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEJ;
    node.terminalList.push_back(p);
    node.velList.push_back(vel);
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveJoint(const TerminalList &ps, double vel, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEJ;
    node.terminalList = ps;
    node.velList.push_back(vel);
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveJointR(const Terminal &p, double vel, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEJR;
    node.terminalList.push_back(p);
    node.velList.push_back(vel);
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveJointR(const TerminalList &ps, double vel, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEJR;
    node.terminalList = ps;
    node.velList.push_back(vel);
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveLine(const HomogeneousMatrix &m, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    Terminal p = m.getTerminal();
    moveLine(p, vel, acc, jerk, frame);
	return 0;
}

int RobotClient::moveLine(const Terminal& p, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEL;
    node.terminalList.push_back(p);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveLineR(const Terminal &p, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVELR;
    node.terminalList.push_back(p);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveCircle(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEC;
    node.terminalList.push_back(p1);
    node.terminalList.push_back(p2);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.referFrame = frame;
    node.circleType = cir;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveCircleR(CIRCLETYPE cir, const Terminal &p1, const Terminal &p2, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVECR;
    node.terminalList.push_back(p1);
    node.terminalList.push_back(p2);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.referFrame = frame;
    node.circleType = cir;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveLineCon(const HomogeneousMatrix &m, double vel, double acc, double jerk, double turn, COORDINATESYSTEM frame){
    Terminal p = m.getTerminal();
    moveLineCon(p, vel, acc, jerk, turn, frame);
	return 0;
}

int RobotClient::moveLineCon(const Terminal& p, double vel, double acc, double jerk, double turn, COORDINATESYSTEM frame){
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEL_CON;
    node.terminalList.push_back(p);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.precision = turn;
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveLineRCon(const Terminal &p, double vel, double acc, double jerk, double turn, COORDINATESYSTEM frame){
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVELR_CON;
    node.terminalList.push_back(p);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.precision = turn;
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveCircleCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc, double jerk, double turn, COORDINATESYSTEM frame){
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEC_CON;
    node.terminalList.push_back(p1);
    node.terminalList.push_back(p2);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.precision = turn;
    node.referFrame = frame;
    node.circleType = cir;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveCircleRCon(CIRCLETYPE cir, const Terminal &p1, const Terminal &p2, double vel, double acc, double jerk, double turn, COORDINATESYSTEM frame){
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVECR_CON;
    node.terminalList.push_back(p1);
    node.terminalList.push_back(p2);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.precision = turn;
    node.referFrame = frame;
    node.circleType = cir;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveStartCon(int delay){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(delay < 2){
        printf("Error: the parameter must larger than 1\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_START_CON;
    node.delayNum = delay;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveEndCon(){
    setCMDRightFlag(-1);

    MotionNode node;
    node.motionType = MOTION_END_CON;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveCurve(const TerminalList& ps, vector<double>& vel, double acc, double jerk, double angle, double bpre, COORDINATESYSTEM frame){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    if(frame == COORDINATE_TOOL){
        printf("Error: cannot choose end frame\n");
        exit(0);
    }
    MotionNode node;
    node.motionType = MOTION_MOVEB;
    node.terminalList = ps;
    node.velList = vel;
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.angleRatio = angle;
    node.precision = bpre;
    node.referFrame = frame;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::moveJump(const HomogeneousMatrix &m, double height, double vel, double acc, double jerk, double turn){
    Terminal p = m.getTerminal();
    moveJump(p, height, vel, turn, acc, jerk);
	return 0;
}

int RobotClient::moveJump(const Terminal &p, double height, double vel, double acc, double jerk, double turn){
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);

    MotionNode node;
    node.motionType = MOTION_MOVEJUMP;
    node.terminalList.push_back(p);
    node.velList.push_back(vel);
    node.accRatio = acc;
    node.jerkRatio = jerk;
    node.referFrame = COORDINATE_BASE;
    node.precision = turn;
    node.height = height;
    sendGJYX(node);

    return waitCMDRightFlag();
}

int RobotClient::waitCommandEnd(){
    return waitCMDFinishFlag();
}


void RobotClient::dragModeStart(){
    sendKSTD();
}

void RobotClient::dragModeEnd(){
    sendJSTD();
}

bool RobotClient::dragModeTimeStart(int period_ms, int timeLength_s){
    sendJLTD(period_ms, timeLength_s);
	return true;
}

bool RobotClient::dragModePlayPrepare(double vel){
    sendZXZB(vel);
	return true;
}

bool RobotClient::dragModePlay(int period_ms){
    sendZXTD(period_ms);
	return true;
}
