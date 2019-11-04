#include "robotServer.h"
#include "RobSoft/CMotionPlanning.hpp"
#include "RobSoft/CErrorCode.hpp"

using namespace std;
using namespace rclib;

int RobotServer::returnZero(double vel){
    return moveABSJoint(m_robotPreference.getInitJointPosition(), vel);
}

static double s_moveToZeroVel;
int RobotServer::moveToZero(double vel){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    s_moveToZeroVel = vel;

#ifndef SIMULATION
    if(ret = m_threadSendTraj.create(threadMoveToZero, NULL) !=0){
        printf("create send traj thread failed %d\n", ret);
        exit(-1);
    }
#else
    ret = 0;
    setCMDFinishFlag(ret);
#endif

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

THREAD_ROUTINE_RETURN RobotServer::threadMoveToZero(void *lpParameter){
    thread_detach();
    thread_name("RobotServer::threadMoveToZero");

    double vel = s_moveToZeroVel;

    robotServer->setRunState(SYSRUN_RUN);

    // 回零过程
    int ret = 0;
    Robotics robot = robotServer->m_robot;
    Joints jrangem(robot.getWholeDOF());
    Joints jrangep(robot.getWholeDOF());
    for(int i=0; i<robot.getWholeDOF(); i++){
        jrangem[JOINTINDEX(i)] = -1000;
        jrangep[JOINTINDEX(i)] = 1000;
    }
    robot.setJointRange(jrangem, jrangep);  // 扩大关节限位，防止软限位导致回零失败

    for(int i=0; i<robot.getWholeDOF(); i++){   // 循环逐个关节回零
        int index = robotServer->m_robotPreference.getJointReturnSequence()[JOINTINDEX(i)]; // 获取回零的顺序
        if(abs(index) <= 0 || abs(index) > robot.getWholeDOF()){    // 忽略不是有效的关节序号
            continue;
        }
        index = index-1;

        Timer timerSend;
        SWITCHSTATE homeState;
        int jogCount;
        // 步骤一，点动回零
        robotServer->setHomeEnabled(JOINTINDEX(index)); // 设置回零关节的回零开关使能
        robot.setCurrentJointPosition(robotServer->getLastRobotMotion().getCurrentJointPosition());
        robotServer->m_jogMove.setRobotics(robot);
        homeState = robotServer->getHomeState(JOINTINDEX(index));
        if(homeState == SWITCHON){
            robotServer->m_jogMove.setJogMode(JOINTINDEX(index), vel);
        }
        else{
            robotServer->m_jogMove.setJogMode(JOINTINDEX(index), -vel);
        }
        robotServer->m_jogMove.resetState();

        timerSend.start(robotServer->m_robotParameter.getSamplePeriod());
        jogCount = 0;
        while(!robotServer->m_jogMove.isFinish()){
            if(robotServer->m_errorFlag & 0x0f){    // 判断硬件是否出错
                ret = 1;
                break;
            }
            if(robotServer->m_errorFlag & 0x10){    // 判断零位开关是否触发
                robotServer->m_jogMove.setFinish();
            }
            else{
                if(jogCount >= 1/robotServer->m_robot.getSamplePeriod()){
                    jogCount = 0;
                    bool tmpHomeState = robotServer->getHomeState(JOINTINDEX(index));
                    if(homeState != tmpHomeState){   // 表示传感器状态已经切换，但是并没有停止发脉冲
                        ret = 1;
                        break;
                    }
                }
                else{
                    jogCount++;
                }
            }

            if(robotServer->getRobotMotionListNum()<3){
                JointsMotionState jms;
                ret = robotServer->m_jogMove.getJogPoint(jms);  // 获取新的点动点
                if(ret){
                    robotServer->printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
                    break;
                }

                RobotMotion rm;
                rm.setCurrentJointPosition(jms.getPosValue());
                rm.setCurrentJointVelocity(jms.getVelValue());
                rm.setCurrentJointAcceleration(jms.getAccValue());
                robotServer->addMotor(rm);
            }
            else{
//                sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000); // 如果缓存区有足够点，则等待
                timerSend.wait();
            }
        }
        sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000*5);
        if(ret){    // 如果出错就退出
            break;
        }

        // 步骤二，关节步进，预备二次回零
        robotServer->setHomeEnabled(JOINT_WHOLE);
        robot.setCurrentJointPosition(robotServer->getLastRobotMotion().getCurrentJointPosition());
        MOVESTEP move(robot);
        move.setStepMode(JOINTINDEX(index), 5, -vel);   // 固定向负方向回一定距离
        ret = move.start();
        if(ret){
            robotServer->printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
            break;
        }
        JointsMotionStateList jms = move.getTraj();
        ret = robotServer->startSendPointList(jms);
        sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000*5);
        if(ret){    // 如果出错就退出
            break;
        }

        // 步骤三，二次点动回零
        robotServer->setHomeEnabled(JOINTINDEX(index)); // 设置回零关节的回零开关使能
        robot.setCurrentJointPosition(robotServer->getLastRobotMotion().getCurrentJointPosition());
        robotServer->m_jogMove.setRobotics(robot);
        homeState = robotServer->getHomeState(JOINTINDEX(index));
        if(homeState == SWITCHON){
            robotServer->m_jogMove.setJogMode(JOINTINDEX(index), vel*0.2);  // 以设置速度的20％，二次回零
        }
        else{
            robotServer->m_jogMove.setJogMode(JOINTINDEX(index), -vel*0.2);
        }
        robotServer->m_jogMove.resetState();

        timerSend.start(robotServer->m_robotParameter.getSamplePeriod());
        jogCount = 0;
        while(1){
            if(robotServer->m_errorFlag & 0x0f){
                ret = 1;
                break;
            }
            if(robotServer->m_errorFlag & 0x10){
                break;
            }
            else{
                if(jogCount >= 1/robotServer->m_robot.getSamplePeriod()){
                    jogCount = 0;
                    bool tmpHomeState = robotServer->getHomeState(JOINTINDEX(index));
                    if(homeState != tmpHomeState){   // 表示传感器状态已经切换，但是并没有停止发脉冲
                        ret = 1;
                        break;
                    }
                }
                else{
                    jogCount++;
                }
            }

            if(robotServer->getRobotMotionListNum()<3){
                JointsMotionState jms;
                ret = robotServer->m_jogMove.getJogPoint(jms);
                if(ret){
                    robotServer->printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
                    break;
                }

                RobotMotion rm;
                rm.setCurrentJointPosition(jms.getPosValue());
                rm.setCurrentJointVelocity(jms.getVelValue());
                rm.setCurrentJointAcceleration(jms.getAccValue());
                robotServer->addMotor(rm);
            }
            else{
//                sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000);
                timerSend.wait();
            }
        }
        sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000*5);
        if(ret){    // 如果出错就退出
            break;
        }

        robotServer->setZero(JOINTINDEX(index));
    }

    robotServer->setHomeEnabled(JOINT_WHOLE);

    // 步骤四，当所有关节都运动到限位后，进行关节补偿运动
    if(!ret){
        Joints joint(robotServer->m_robotParameter.getWholeDOF());
        robotServer->setCurrentJointPosition(joint);    // 关节角度置零

        Joints jointCompensation = robotServer->m_robotPreference.getJointCompensation();
        JointsList js;
        js.push_back(jointCompensation);
        robot.setCurrentJointPosition(robotServer->getLastRobotMotion().getCurrentJointPosition());
        MOVEJOINT move(robot);
        move.setWayPointsWithVel(js, vel);
        ret = move.start();
        if(ret){
            robotServer->printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        }
        else{
            JointsMotionStateList jms = move.getTraj();
            ret = robotServer->startSendPointList(jms);
        }
    }

    if(ret){
        robotServer->printMoveToZeroResult(false);
    }
    else{
//        Joints joint(robotServer->m_robotParameter.getWholeDOF());
//        robotServer->setCurrentJointPosition(joint);    // 关节角度置零
        robotServer->setZero(JOINT_WHOLE);
        robotServer->printMoveToZeroResult(true);
    }

    robotServer->setRunState(SYSRUN_STOP);

    robotServer->setCMDFinishFlag(ret);
	return 0;
}

int RobotServer::jointJOG(JOINTINDEX index, MOVEDIRECTION dir, double vel){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare() !=0){
        return ret;
    }

    // 运动规划
    if(dir == MOVE_POSITIVE){
        vel = fabs(vel);
    }
    else{
        vel = -fabs(vel);
    }
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    m_jogMove.setRobotics(m_robot);
    m_jogMove.setJogMode(index, vel);
    m_jogMove.resetState();

    if(ret = m_threadSendTraj.create(threadSendMoveJOG, NULL)!=0){
        printf("create send traj thread failed %d\n", ret);
        exit(-1);
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::terminalJOG(TERMINALINDEX index, MOVEDIRECTION dir, double vel, COORDINATESYSTEM frame){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    if(dir == MOVE_POSITIVE){
        vel = fabs(vel);
    }
    else{
        vel = -fabs(vel);
    }
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    m_jogMove.setRobotics(m_robot);
    m_jogMove.setJogMode(index, vel, frame);
    m_jogMove.resetState();

    if(ret = m_threadSendTraj.create(threadSendMoveJOG, NULL)!=0){
        printf("create send traj thread failed %d\n", ret);
        exit(-1);
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

THREAD_ROUTINE_RETURN RobotServer::threadSendMoveJOG(void* lpParameter){
    thread_detach();
    thread_name("RobotServer::threadSendMoveJOG");

//    robotServer->setRunState(SYSRUN_RUN);

    int ret = 0;
    Timer timerSend;
    timerSend.start(robotServer->m_robotParameter.getSamplePeriod());
    while(!robotServer->m_jogMove.isFinish()){
        if(robotServer->m_errorFlag & 0x0f){
            ret = 1;
            break;
        }

        if(robotServer->getRobotMotionListNum()<3){
            JointsMotionState jms;
            ret = robotServer->m_jogMove.getJogPoint(jms);
            if(ret){
                robotServer->printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
                break;
            }

            RobotMotion rm;
            rm.setCurrentJointPosition(jms.getPosValue());
            rm.setCurrentJointVelocity(jms.getVelValue());
            rm.setCurrentJointAcceleration(jms.getAccValue());
            robotServer->addMotor(rm);
        }
        else{
//            sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000);
            timerSend.wait();
        }
    }

//    if(robotServer->m_robotState.getPlayState() == SYSPLAY_TEACH){
//        robotServer->setRunState(SYSRUN_STOP);
//    }

    robotServer->setCMDFinishFlag(ret);
	return 0;
}

void RobotServer::stopJOG(){
    m_jogMove.setFinish();
}

int RobotServer::jointStep(JOINTINDEX index, MOVEDIRECTION dir, double step, double vel){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    if(dir == MOVE_POSITIVE){
        vel = fabs(vel);
    }
    else{
        vel = -fabs(vel);
    }
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    MOVESTEP stepMove(m_robot);
    stepMove.setStepMode(index, step, vel);
    ret = stepMove.start();
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        setCMDFinishFlag(ret);
    }
    else{
        m_trajList = stepMove.getTraj();
        if(ret = m_threadSendTraj.create(threadSendTraj, NULL)!=0){
            printf("create send traj thread failed %d\n", ret);
            exit(-1);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::terminalStep(TERMINALINDEX index, MOVEDIRECTION dir, double step, double vel, COORDINATESYSTEM frame){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    if(dir == MOVE_POSITIVE){
        vel = fabs(vel);
    }
    else{
        vel = -fabs(vel);
    }
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    MOVESTEP stepMove(m_robot);
    stepMove.setStepMode(index, step, vel, frame);
    ret = stepMove.start();
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        setCMDFinishFlag(ret);
    }
    else{
        m_trajList = stepMove.getTraj();
        if(ret = m_threadSendTraj.create(threadSendTraj, NULL)!=0){
            printf("create send traj thread failed %d\n", ret);
            exit(-1);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}


int RobotServer::moveABSJoint(const Joints& ps, double vel){
    JointsList js;
    js.push_back(ps);
    return moveABSJoint(js, vel);
}

int RobotServer::moveABSJoint(const JointsList& ps, double vel){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    MOVEJOINT move(m_robot);
    move.setWayPointsWithVel(ps, vel);
    ret = move.start();
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        setCMDFinishFlag(ret);
    }
    else{
        m_trajList = move.getTraj();
        if(ret = m_threadSendTraj.create(threadSendTraj, NULL)!=0){
            printf("create send traj thread failed %d\n", ret);
            exit(-1);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::moveABSJoint(const JointsList& ps, std::vector<double>& tm){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    MOVEJOINT move(m_robot);
    move.setWayPointsWithTime(ps, tm);
    ret = move.start();
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        setCMDFinishFlag(ret);
    }
    else{
        m_trajList = move.getTraj();
        if(ret = m_threadSendTraj.create(threadSendTraj, NULL)!=0){
            printf("create send traj thread failed %d\n", ret);
            exit(-1);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::moveABSJointR(const Joints& ps, double vel){
    Joints joint = getLastRobotMotion().getCurrentJointPosition();
    joint = joint+ps;
    return moveABSJoint(joint, vel);
}

int RobotServer::moveABSJointR(const JointsList &ps, double vel){
    Joints joint = getLastRobotMotion().getCurrentJointPosition();
    JointsList js;
    js.push_back(joint);
    for(int i=0; i<ps.size(); i++){
        joint = joint + ps[i];
        js.push_back(joint);
    }

    return moveABSJoint(js, vel);
}

int RobotServer::moveABSJointR(const JointsList &ps, std::vector<double> &tm){
    Joints joint = getLastRobotMotion().getCurrentJointPosition();
    JointsList js;
    js.push_back(joint);
    for(int i=0; i<ps.size(); i++){
        joint = joint + ps[i];
        js.push_back(joint);
    }

    return moveABSJoint(js, tm);
}


int RobotServer::moveJoint(const Terminal& p, double vel, COORDINATESYSTEM frame){
    Joints joint = getLastRobotMotion().getCurrentJointPosition();
    int ret = m_robot.inverseKinematics(joint, p, joint, frame);
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        return ret;
    }
    return moveABSJoint(joint, vel);
}

int RobotServer::moveJoint(const TerminalList& ps, double vel, COORDINATESYSTEM frame){
    Joints joint = getLastRobotMotion().getCurrentJointPosition();
    JointsList js;
    int ret = m_robot.inverseKinematics(js, ps, joint, frame);
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        return ret;
    }
    return moveABSJoint(js, vel);
}

int RobotServer::moveJointR(const Terminal& p, double vel, COORDINATESYSTEM frame){
    Terminal terminal;
    if(frame == COORDINATE_BASE){
        terminal = getLastRobotMotion().getCurrentTerminal();
    }
    else{
        terminal = getLastRobotMotion().getCurrentWorkTerminal();
    }
    terminal = terminal + p;

    return moveJoint(terminal, vel, frame);
}

int RobotServer::moveJointR(const TerminalList& ps, double vel, COORDINATESYSTEM frame){
    Terminal terminal;
    if(frame == COORDINATE_BASE){
        terminal = getLastRobotMotion().getCurrentTerminal();
    }
    else{
        terminal = getLastRobotMotion().getCurrentWorkTerminal();
    }
    TerminalList ts;
    ts.push_back(terminal);
    for(int i=0; i<ps.size(); i++){
        terminal = terminal + ps[i];
        ts.push_back(terminal);
    }

    return moveJoint(ts, vel, frame);
}


int RobotServer::moveLine(const HomogeneousMatrix& m, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    Terminal terminal = m.getTerminal();
    return moveLine(terminal, vel, acc, jerk, frame);
}

int RobotServer::moveLine(const Terminal& p, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    MOVELC move(m_robot);
    move.setWayPoints(p, vel, acc, jerk, frame);
    ret = move.start();
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        setCMDFinishFlag(ret);
    }
    else{
        m_trajList = move.getTraj();
        if(ret = m_threadSendTraj.create(threadSendTraj, NULL)!=0){
            printf("create send traj thread failed %d\n", ret);
            exit(-1);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::moveLineR(const Terminal& p, double vel, double acc, double jerk, COORDINATESYSTEM frame){
    Terminal terminal;
    if(frame == COORDINATE_BASE){
        terminal = getLastRobotMotion().getCurrentTerminal();
    }
    else{
        terminal = getLastRobotMotion().getCurrentWorkTerminal();
    }

    return moveLine(terminal+p, vel, acc, jerk, frame);
}

int RobotServer::moveCircle(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc, double jerk , COORDINATESYSTEM frame){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    MOVELC move(m_robot);
    move.setWayPoints(cir, p1, p2, vel, acc, jerk, frame);
    ret = move.start();
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
        setCMDFinishFlag(ret);
    }
    else{
        m_trajList = move.getTraj();
        while(ret = m_threadSendTraj.create(threadSendTraj, NULL)!=0){
            printf("create send traj thread failed %d\n", ret);
            exit(-1);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::moveCircleR(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc , double jerk , COORDINATESYSTEM frame){
    Terminal terminal;
    if(frame == COORDINATE_BASE){
        terminal = getLastRobotMotion().getCurrentTerminal();
    }
    else{
        terminal = getLastRobotMotion().getCurrentWorkTerminal();
    }

    return moveCircle(cir, terminal+p1, terminal+p2, vel, acc, jerk, frame);
}


int RobotServer::moveLineCon(const HomogeneousMatrix& m, double vel, double acc , double jerk , double turn , COORDINATESYSTEM frame){
    Terminal terminal = m.getTerminal();
    return moveLineCon(terminal, vel, acc, jerk, turn, frame);
}

int RobotServer::moveLineCon(const Terminal& p, double vel, double acc , double jerk , double turn , COORDINATESYSTEM frame){
    setCMDRightFlag(-1);

    // 判断是否没有开始连续轨迹
    if(m_conMove.isEnd()){
        printNeverStartConTraj();
        setCMDRightFlag(1);
        return 1;
    }

    // 运动规划
    m_robot.setCurrentJointPosition(m_conMove.getLastRobotMotion().getCurrentJointPosition());
    MOVELC move(m_robot);
    move.setWayPoints(p, vel, acc, jerk, turn, frame);

    int ret = m_conMove.addPath(move);
    if(ret){
        m_conMove.endPath();
        m_trajListCon.push_back(m_conMove.getTraj());
        m_conMove.setEnd();
        waitCMDFinishFlag();
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
    }
    else{
        m_trajListCon.push_back(m_conMove.getTraj());
        while(m_trajListCon.size() >= m_conTrajWaitNum){
            sleep_ms(m_robotParameter.getSamplePeriod()*1000);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::moveLineRCon(const Terminal& p, double vel, double acc , double jerk , double turn , COORDINATESYSTEM frame){
    Terminal terminal;
    if(frame == COORDINATE_BASE){
        terminal = m_conMove.getLastRobotMotion().getCurrentTerminal();
    }
    else{
        terminal = m_conMove.getLastRobotMotion().getCurrentWorkTerminal();
    }

    return moveLineCon(terminal+p, vel, acc, jerk, turn, frame);
}

int RobotServer::moveCircleCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc , double jerk , double turn , COORDINATESYSTEM frame){
    setCMDRightFlag(-1);

    // 判断是否没有开始连续轨迹
    if(m_conMove.isEnd()){
        printNeverStartConTraj();
        setCMDRightFlag(1);
        return 1;
    }

    // 运动规划
    m_robot.setCurrentJointPosition(m_conMove.getLastRobotMotion().getCurrentJointPosition());
    MOVELC move(m_robot);
    move.setWayPoints(cir, p1, p2, vel, acc, jerk, turn, frame);

    int ret = m_conMove.addPath(move);
    if(ret){
        m_conMove.endPath();
        m_trajListCon.push_back(m_conMove.getTraj());
        m_conMove.setEnd();
        waitCMDFinishFlag();
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
    }
    else{
        m_trajListCon.push_back(m_conMove.getTraj());
        if(m_trajListCon.size() >= m_conTrajWaitNum){
            sleep_ms(m_robotParameter.getSamplePeriod()*1000);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::moveCircleRCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc , double jerk , double turn , COORDINATESYSTEM frame){
    Terminal terminal;
    if(frame == COORDINATE_BASE){
        terminal = m_conMove.getLastRobotMotion().getCurrentTerminal();
    }
    else{
        terminal = m_conMove.getLastRobotMotion().getCurrentWorkTerminal();
    }

    return moveCircleCon(cir, terminal+p1, terminal+p2, vel, acc, jerk, turn, frame);
}

int RobotServer::moveStartCon(int delay){
    if(!robotServer->m_conMove.isEnd()){
        return ERROR_TRUE;
    }

    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    m_conTrajWaitNum = delay;
    //clear list
    m_trajListCon.clear();
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    m_conMove.setRobotics(m_robot);
    m_conMove.start();

    // 开始发送轨迹线程
    if(ret = m_threadSendTraj.create(threadSendConTraj, NULL)!=0){
        printf("create send traj thread failed %d\n", ret);
        exit(-1);
    }

    // 返回执行结果
    setCMDRightFlag(ret);
    return ret;
}

int RobotServer::moveEndCon(){
    m_conMove.endPath();
    m_trajListCon.push_back(m_conMove.getTraj());
    m_conMove.setEnd();
    // 返回执行结果
    setCMDRightFlag(0);
    return 0;
}


int RobotServer::moveCurve(const TerminalList& ps, std::vector<double>& vel, double acc, double jerk, double angle, double bpre, COORDINATESYSTEM frame){
    // 运动前判断准备
    int ret;
    if(ret = judgeMovePrepare()){
        return ret;
    }

    // 运动规划
    m_robot.setCurrentJointPosition(getLastRobotMotion().getCurrentJointPosition());
    MOVEBCURVE move(m_robot);
    move.setWayPoints(ps, vel, acc, jerk, angle, bpre, frame);
    ret = move.start();
    if(ret){
        printInfo(INFO_ERROR, get_error_string(ret,robotServer->languageType()));
    }
    else{
        m_trajList = move.getTraj();
        if(ret = m_threadSendTraj.create(threadSendTraj, NULL)!=0){
            printf("create send traj thread failed %d\n", ret);
            exit(-1);
        }
    }

    // 返回规划结果
    setCMDRightFlag(ret);
    return ret;
}


int RobotServer::moveJump(const HomogeneousMatrix& m, double height, double vel, double acc , double jerk , double turn ){
    Terminal terminal = m.getTerminal();
    return moveJump(terminal, height, vel, acc, jerk, turn);
}

int RobotServer::moveJump(const Terminal& p, double height, double vel, double acc , double jerk , double turn ){
	return 0;
}


int RobotServer::waitCommandEnd(){
    return waitCMDFinishFlag();
}


void RobotServer::dragModeStart(){

}

void RobotServer::dragModeEnd(){

}

bool RobotServer::dragModeTimeStart(int period_ms, int timeLength_s){
	return true;
}

bool RobotServer::dragModePlayPrepare(double vel){
	return true;
}

bool RobotServer::dragModePlay(int period_ms){
	return true;
}

list<Joints> RobotServer::getDragPoint(){
    return m_dragPointList;
}

int RobotServer::startSendPointList(const JointsMotionStateList &js){
    int sendIndex = 0;
    int sendNum = js.size();
    Timer timerSend;
    timerSend.start(m_robotParameter.getSamplePeriod());
    while(sendIndex < sendNum){
        if(m_errorFlag & 0x0f){
            return 1;
        }
        if(m_robotState.getRunState() == SYSRUN_RUN){
            if(getRobotMotionListNum()<3){
                RobotMotion rm;
                rm.setCurrentJointPosition(js[sendIndex].getPosValue());
                rm.setCurrentJointVelocity(js[sendIndex].getVelValue());
                rm.setCurrentJointAcceleration(js[sendIndex].getAccValue());
                addMotor(rm);
                sendIndex++;
            }
            else{
                timerSend.wait();
//                sleep_ms(m_robotParameter.getSamplePeriod()*1000);
            }
        }
        else if(m_robotState.getRunState() == SYSRUN_PAUSE){
            sleep_ms(m_robotParameter.getSamplePeriod()*1000);
        }
        else if(m_robotState.getRunState() == SYSRUN_STOP){
            return 1;
        }
    }
    return 0;
}

THREAD_ROUTINE_RETURN RobotServer::threadSendTraj(void* lpParameter){
    thread_detach();
    thread_name("RobotServer::threadSendTraj");

    if(robotServer->m_robotState.getPlayState() == SYSPLAY_TEACH){
        robotServer->setRunState(SYSRUN_RUN);
    }

    int ret = 0;
    ret = robotServer->startSendPointList(robotServer->m_trajList);

    if(robotServer->m_robotState.getPlayState() == SYSPLAY_TEACH){
        robotServer->setRunState(SYSRUN_STOP);
    }

    robotServer->setCMDFinishFlag(ret);
	return 0;
}

THREAD_ROUTINE_RETURN RobotServer::threadSendConTraj(void* lpParameter){
    thread_detach();
    thread_name("RobotServer::threadSendConTraj");

    if(robotServer->m_robotState.getPlayState() == SYSPLAY_TEACH){
        robotServer->setRunState(SYSRUN_RUN);
    }

    while(robotServer->m_trajListCon.size()<2){
        if(robotServer->m_conMove.isEnd()){
            break;
        }
        sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000);
    }

    int ret = 0;
    while(1){
        if(robotServer->m_trajListCon.size()<2 && !robotServer->m_conMove.isEnd()){
            sleep_ms(robotServer->m_robotParameter.getSamplePeriod()*1000);
            continue;
        }
        if(robotServer->m_trajListCon.empty() && robotServer->m_conMove.isEnd()){
            break;
        }

        ret = robotServer->startSendPointList(robotServer->m_trajListCon.front());
        robotServer->m_trajListCon.pop_front();
        if(ret){
            robotServer->m_conMove.setEnd();
			robotServer->m_trajListCon.clear();
            break;
        }
    }

    if(robotServer->m_robotState.getPlayState() == SYSPLAY_TEACH){
        robotServer->setRunState(SYSRUN_STOP);
    }

    robotServer->setCMDFinishFlag(ret);
	return 0;
}
