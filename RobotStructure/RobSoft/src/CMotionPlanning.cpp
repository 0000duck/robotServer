#ifndef CMOTIONPLANNING_CPP
#define CMOTIONPLANNING_CPP

#include "CMotionPlanning.hpp"
#include "CMathBasic.hpp"
#include "CErrorCode.hpp"

#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace robsoft;

MOVEJOG::MOVEJOG(){

}

MOVEJOG::MOVEJOG(const Robotics &robot){
    setRobotics(robot);
}

MOVEJOG::~MOVEJOG(){

}

void MOVEJOG::setRobotics(const Robotics &robot){
    m_robot = robot;
    m_jogPolynomial.setPeriod(robot.getSamplePeriod());

    m_initJointMotionState.setPosValue(m_robot.getCurrentJointPosition());
    m_initJointMotionState.setVelValue(m_robot.getCurrentJointVelocity());
    m_initJointMotionState.setAccValue(m_robot.getCurrentJointAcceleration());

    m_initJoint = m_robot.getCurrentJointPosition();
    m_initTerminal = m_robot.getCurrentTerminal();
    m_initWorkTerminal = m_robot.getCurrentWorkTerminal();

    m_lastJointMotionState = m_initJointMotionState;
}

void MOVEJOG::setJogMode(JOINTINDEX index, double vel){
    if(fabs(vel) <= 0 || fabs(vel) > 1){
        throw string("Error: MOVEJOG::the vel should belong to [-1,1)&(0, 1]");
    }

    m_jointTerminalFlag = 0;
    m_jointIndex = index;

    // 全局速度作用于速度、加速度和冲击
//    double v = m_robot.getJointMaxVel().getValue(index)*vel;
//    double a = m_robot.getJointMaxAcc().getValue(index)*vel;
//    double j = m_robot.getJointMaxJerk().getValue(index)*vel;
    //　全局速度仅作用于速度
    double v = m_robot.getJointMaxVel().getValue(index)*vel;
    double a = m_robot.getJointMaxAcc().getValue(index)*vel/fabs(vel);
    double j = m_robot.getJointMaxJerk().getValue(index)*vel/fabs(vel);

    m_jogPolynomial.setMotionCoefficient(v, a, j);
}

void MOVEJOG::setJogMode(TERMINALINDEX index, double vel, COORDINATESYSTEM coord){
    if(fabs(vel) <= 0 || fabs(vel) > 1){
        throw string("Error: MOVEJOG::the vel should belong to [-1,1)&(0, 1]");
    }

    m_jointTerminalFlag = 1;
    m_terminalIndex = index;
    m_coord = coord;

    // 全局速度作用于速度、加速度和冲击
//    double v = m_robot.getTerminalMaxVel().getValue(index)*vel;
//    double a = m_robot.getTerminalMaxAcc().getValue(index)*vel;
//    double j = m_robot.getTerminalMaxJerk().getValue(index)*vel;
    //　全局速度仅作用于速度
    double v = m_robot.getTerminalMaxVel().getValue(index)*vel;
    double a = m_robot.getTerminalMaxAcc().getValue(index)*vel/fabs(vel);
    double j = m_robot.getTerminalMaxJerk().getValue(index)*vel/fabs(vel);

    m_jogPolynomial.setMotionCoefficient(v, a, j);
}

void MOVEJOG::resetState(){
    m_lastJointMotionState = m_initJointMotionState;
    m_jogPolynomial.resetState();
}

bool MOVEJOG::isFinish(){
    return m_jogPolynomial.isEnd();
}

int MOVEJOG::getJogPoint(JointsMotionState &jointMotionState){
    CMatrix<double> samplePoint;
    samplePoint = m_jogPolynomial.getNextPoint();

    if(m_jointTerminalFlag == 0){
        CMatrix<double> mat(m_robot.getWholeDOF(), 1);

        mat = m_lastJointMotionState.getPosValue().getValue();
        mat.at(m_jointIndex, 0) += samplePoint.at(0, 0);
        jointMotionState.setPosValue(Joints(m_robot.getWholeDOF(), mat));

        mat = m_lastJointMotionState.getVelValue().getValue();
        mat.at(m_jointIndex, 0) = samplePoint.at(0, 1);
        jointMotionState.setVelValue(Joints(m_robot.getWholeDOF(), mat));

        mat = m_lastJointMotionState.getAccValue().getValue();
        mat.at(m_jointIndex, 0) = samplePoint.at(0, 2);
        jointMotionState.setAccValue(Joints(m_robot.getWholeDOF(), mat));
    }
    else{
        CMatrix<double> mat(6, 1);
        mat.at(m_terminalIndex, 0) = samplePoint.at(0, 0);
        Terminal terminal(mat);

        Joints joint;
        switch(m_coord){
        case COORDINATE_BASE:{
            terminal = m_initTerminal+terminal;
            int ret = m_robot.inverseKinematics(joint, terminal, m_lastJointMotionState.getPosValue(), COORDINATE_BASE);
            if(ret){
                return ret;
            }
        }
            break;
        case COORDINATE_WORK:{
            terminal = m_initWorkTerminal+terminal;
            int ret = m_robot.inverseKinematics(joint, terminal, m_lastJointMotionState.getPosValue(), COORDINATE_WORK);
            if(ret){
                return ret;
            }
        }
            break;
        case COORDINATE_TOOL:{
            terminal = m_initTerminal * terminal;
            int ret = m_robot.inverseKinematics(joint, terminal, m_lastJointMotionState.getPosValue(), COORDINATE_BASE);
            if(ret){
                return ret;
            }
        }
            break;
        default:
            break;
        }

        jointMotionState.setPosValue(joint);
        jointMotionState.calVelAcc(m_lastJointMotionState, m_robot.getSamplePeriod());
        m_lastJointMotionState = jointMotionState;
    }

    int ret = m_robot.checkJointWithinPerformance(jointMotionState);
    if(ret){
        return ret;
    }

    return 0;
}

void MOVEJOG::setFinish(){
    m_jogPolynomial.setEnd();
}


MOVESTEP::MOVESTEP(){

}

MOVESTEP::MOVESTEP(const Robotics &robot){
    setRobotics(robot);
}

MOVESTEP::~MOVESTEP(){

}

void MOVESTEP::setRobotics(const Robotics &robot){
    m_robot = robot;
    m_sPolynomial.setPeriod(robot.getSamplePeriod());

    m_initJointMotionState.setPosValue(m_robot.getCurrentJointPosition());
    m_initJointMotionState.setVelValue(m_robot.getCurrentJointVelocity());
    m_initJointMotionState.setAccValue(m_robot.getCurrentJointAcceleration());

    m_initJoint = m_robot.getCurrentJointPosition();
    m_initTerminal = m_robot.getCurrentTerminal();
    m_initWorkTerminal = m_robot.getCurrentWorkTerminal();

    m_lastJointMotionState = m_initJointMotionState;
}

void MOVESTEP::setStepMode(JOINTINDEX index, double len, double vel){
    if(fabs(vel) <= 0 || fabs(vel) > 1){
        throw string("Error: MOVESTEP::the vel should belong to [-1,1)&(0, 1]");
    }

    m_jointTerminalFlag = 0;    // 关节步进
    m_jointIndex = index;

    // 全局速度作用于速度、加速度和冲击
//    double v = m_robot.getJointMaxVel().getValue(index)*vel;
//    double a = m_robot.getJointMaxAcc().getValue(index)*vel;
//    double j = m_robot.getJointMaxJerk().getValue(index)*vel;
    //　全局速度仅作用于速度
    double v = m_robot.getJointMaxVel().getValue(index)*vel;
    double a = m_robot.getJointMaxAcc().getValue(index)*vel/fabs(vel);
    double j = m_robot.getJointMaxJerk().getValue(index)*vel/fabs(vel);

    m_sPolynomial.setMotionCoefficientWithVel(vel / fabs(vel) * len, v, a, j);
}

void MOVESTEP::setStepMode(TERMINALINDEX index, double len, double vel, COORDINATESYSTEM coord){
    if(fabs(vel) <= 0 || fabs(vel) > 1){
        throw string("Error: MOVESTEP::the vel should belong to [-1,1)&(0, 1]");
    }

    m_jointTerminalFlag = 1;    // 末端步进
    m_terminalIndex = index;
    m_coord = coord;

    // 全局速度作用于速度、加速度和冲击
//    double v = m_robot.getTerminalMaxVel().getValue(index)*vel;
//    double a = m_robot.getTerminalMaxAcc().getValue(index)*vel;
//    double j = m_robot.getTerminalMaxJerk().getValue(index)*vel;
    //　全局速度仅作用于速度
    double v = m_robot.getTerminalMaxVel().getValue(index)*vel;
    double a = m_robot.getTerminalMaxAcc().getValue(index)*vel/fabs(vel);
    double j = m_robot.getTerminalMaxJerk().getValue(index)*vel/fabs(vel);

    m_sPolynomial.setMotionCoefficientWithVel(vel / fabs(vel) * len, v, a, j);
}

void MOVESTEP::resetState(){
    m_lastJointMotionState = m_initJointMotionState;
    m_sPolynomial.resetState();
}

int MOVESTEP::start(){
    m_jointTraj.clear();
    CMatrix<double> samplePoints;
    samplePoints = m_sPolynomial.getAllSegment();

    if(m_jointTerminalFlag == 0){
        Joints pos = m_robot.getCurrentJointPosition();
        Joints vel = m_robot.getCurrentJointVelocity();
        Joints acc = m_robot.getCurrentJointAcceleration();

        for(int i=0; i<samplePoints.rows(); i++){
            pos = m_robot.getCurrentJointPosition();
            pos[m_jointIndex] += samplePoints.at(i, 0);
            vel[m_jointIndex] = samplePoints.at(i, 1);
            acc[m_jointIndex] = samplePoints.at(i, 2);

            JointsMotionState jointMotion;
            jointMotion.setPosValue(pos);
            jointMotion.setVelValue(vel);
            jointMotion.setAccValue(acc);

            m_jointTraj.push_back(jointMotion);

            int ret = m_robot.checkJointWithinPerformance(jointMotion);
            if(ret){
                return ret;
            }
        }
    }
    else{
        for(int i=0; i<samplePoints.rows(); i++){
            CMatrix<double> mat(6, 1);
            mat.at(m_terminalIndex, 0) = samplePoints.at(i, 0);
            Terminal terminal(mat);

            Joints joint;
            switch(m_coord){
            case COORDINATE_BASE:{
                terminal = m_initTerminal+terminal;
                int ret = m_robot.inverseKinematics(joint, terminal, m_lastJointMotionState.getPosValue(), COORDINATE_BASE);
                if(ret){
                    return ret;
                }
            }
                break;
            case COORDINATE_WORK:{
                terminal = m_initWorkTerminal+terminal;
                int ret = m_robot.inverseKinematics(joint, terminal, m_lastJointMotionState.getPosValue(), COORDINATE_WORK);
                if(ret){
                    return ret;
                }
            }
                break;
            case COORDINATE_TOOL:{
                terminal = m_initTerminal * terminal;
                int ret = m_robot.inverseKinematics(joint, terminal, m_lastJointMotionState.getPosValue(), COORDINATE_BASE);
                if(ret){
                    return ret;
                }
            }
                break;
            default:
                break;
            }


            JointsMotionState jointMotion;
            jointMotion.setPosValue(joint);
            jointMotion.calVelAcc(m_lastJointMotionState, m_robot.getSamplePeriod());
            m_lastJointMotionState = jointMotion;

            m_jointTraj.push_back(jointMotion);

            int ret = m_robot.checkJointWithinPerformance(jointMotion);
            if(ret){
                return ret;
            }
        }
    }

    return 0;
}

JointsMotionStateList MOVESTEP::getTraj() const{
    return m_jointTraj;
}

void MOVESTEP::getTraj(JointsMotionStateList &jointTraj) const{
    jointTraj = m_jointTraj;
}


MOVEJOINT::MOVEJOINT(const Robotics &robot){
    m_robot = robot;

    m_quinticContinuousPolynominal.resize(m_robot.getWholeDOF());
    for(int i=0; i<m_quinticContinuousPolynominal.size(); i++){
        m_quinticContinuousPolynominal[i].setPeriod(robot.getSamplePeriod());
    }
}

MOVEJOINT::~MOVEJOINT(){

}

void MOVEJOINT::setWayPointsWithTime(const JointsList &joints, const std::vector<double> &times){
    if(joints.empty()){
        throw string("Error: MOVEJ::the number of way points should be 1 at least");
    }
    if(joints.size() != times.size()){
        throw string("Error: MOVEJ::invalid time interval number");
    }

    m_initJointMotionState.setPosValue(m_robot.getCurrentJointPosition());
    m_initJointMotionState.setVelValue(m_robot.getCurrentJointVelocity());
    m_initJointMotionState.setAccValue(m_robot.getCurrentJointAcceleration());
    resetState();

    vector<double> x;
    x.push_back(0);
    JointsList js;
    js.push_back(m_robot.getCurrentJointPosition());
    for(int i=0; i<times.size(); i++){
        x.push_back(x[x.size()-1]+times[i]);
        js.push_back(joints[i]);
    }
    for(int i=0; i<m_quinticContinuousPolynominal.size(); i++){
        m_quinticContinuousPolynominal[i].setPoints(CMatrix<double>(x.size(), 1, x), js.getCertainJointList(JOINTINDEX(i)));
    }
}

void MOVEJOINT::setWayPointsWithTime(const JointsList &joints, const JointsList &vel, const JointsList &acc, const std::vector<double> &times){
    if(joints.empty()){
        throw string("Error: MOVEJ::the number of way points should be 1 at least");
    }
    if(joints.size() != times.size()){
        throw string("Error: MOVEJ::invalid time interval number");
    }

    m_initJointMotionState.setPosValue(m_robot.getCurrentJointPosition());
    m_initJointMotionState.setVelValue(m_robot.getCurrentJointVelocity());
    m_initJointMotionState.setAccValue(m_robot.getCurrentJointAcceleration());
    resetState();

    vector<double> x;
    x.push_back(0);
    JointsList js, vs, as;
    js.push_back(m_robot.getCurrentJointPosition());
    vs.push_back(m_robot.getCurrentJointVelocity());
    as.push_back(m_robot.getCurrentJointAcceleration());
    for(int i=0; i<times.size(); i++){
        x.push_back(x[x.size()-1]+times[i]);
        js.push_back(joints[i]);
        vs.push_back(vel[i]);
        as.push_back(acc[i]);
    }
    for(int i=0; i<m_quinticContinuousPolynominal.size(); i++){
        m_quinticContinuousPolynominal[i].setPoints(CMatrix<double>(x.size(), 1, x), js.getCertainJointList(JOINTINDEX(i)), vs.getCertainJointList(JOINTINDEX(i)), as.getCertainJointList(JOINTINDEX(i)));
    }
}

void MOVEJOINT::setWayPointsWithTime(const TerminalList &terminals, const std::vector<double> &times, COORDINATESYSTEM coord){
    JointsList joints;
    m_robot.inverseKinematics(joints, terminals, m_robot.getCurrentJointPosition(), coord);

    this->setWayPointsWithTime(joints, times);
}

void MOVEJOINT::setWayPointsWithVel(const JointsList &joints, const double &vel){
    if(joints.empty()){
        throw string("Error: MOVEJ::the number of way points should be 1 at least");
    }
    if(vel <= 0 || vel > 1){
        throw string("Error: MOVEJ::the vel should belong to (0, 1]");
    }

    SPolynomial sp;
    vector<double> times;
    JointsList js;
    Joints joint = m_robot.getCurrentJointPosition();
    for(int i=0; i<joints.size(); i++){
        double time = 0;

        Joints jointDiff = joints[i]-joint;
        for(int j=0; j<jointDiff.getJointsDOF(); j++){
            double diff = fabs(jointDiff.getValue(JOINTINDEX(j)));
            sp.setMotionCoefficientWithVel(diff, m_robot.getJointMaxVel().getValue(JOINTINDEX(j))*vel, m_robot.getJointMaxAcc().getValue(JOINTINDEX(j)), m_robot.getJointMaxJerk().getValue(JOINTINDEX(j)));
            double temp =sp.getTime(diff);

            if(fabs(temp) > time){
                time = fabs(temp);
            }
        }
//        time *= 2;

        if(num_is_zero(time)){  // 如果时间为０，说明两个点相同，则合并
            if(!js.empty()){
                js.pop_back();
                time =times.back();
                times.pop_back();
            }
            else{
                continue;
            }
        }
        times.push_back(time);
        js.push_back(joints[i]);
        joint = joints[i];
    }

    if(js.empty()){ // 极端情况，所有的点都时相同点，则保留至少一个目标点，且时间间隔为０
        js.push_back(joints[joints.size()-1]);
        times.push_back(0);
    }

    this->setWayPointsWithTime(js, times);
}

void MOVEJOINT::setWayPointsWithVel(const TerminalList &terminals, const double &vel, COORDINATESYSTEM coord){
    JointsList joints;
    m_robot.inverseKinematics(joints, terminals, m_robot.getCurrentJointPosition(), coord);

    this->setWayPointsWithVel(joints, vel);
}

void MOVEJOINT::resetState(){
    m_lastJointMotionState = m_initJointMotionState;
    for(int i=0; i<m_quinticContinuousPolynominal.size(); i++){
        m_quinticContinuousPolynominal[i].resetState();
    }
}

int MOVEJOINT::getTrajNum() const{
    return m_quinticContinuousPolynominal[0].getSegmentNum();
}

int MOVEJOINT::startNext(){
    m_jointTraj.clear();
    vector<CMatrix<double>> samplePoints;
    for(int i=0; i<m_quinticContinuousPolynominal.size(); i++){
        samplePoints.push_back(m_quinticContinuousPolynominal[i].getNextSegment());
    }

    int dof = samplePoints.size();

    for(int j=0; j<samplePoints[0].rows(); j++){
        JointsMotionState jointMotion;
        jointMotion.setValue(dof);
        vector<double> pos, vel, acc;
        for(int i=0; i<dof; i++){
            pos.push_back(samplePoints[i].at(j, 0));
            vel.push_back(samplePoints[i].at(j, 1));
            acc.push_back(samplePoints[i].at(j, 2));
        }
        jointMotion.setPosValue(Joints(dof, pos));
        jointMotion.setVelValue(Joints(dof, vel));
        jointMotion.setAccValue(Joints(dof, acc));

        m_jointTraj.push_back(jointMotion);

        int ret = m_robot.checkJointWithinPerformance(jointMotion);
        if(ret){
            return ret;
        }
    }

    return 0;
}

int MOVEJOINT::start(){
    m_jointTraj.clear();
    vector<CMatrix<double>> samplePoints;
    for(int i=0; i<m_quinticContinuousPolynominal.size(); i++){
        samplePoints.push_back(m_quinticContinuousPolynominal[i].getAllSegment());
    }

    int dof = samplePoints.size();

    for(int j=0; j<samplePoints[0].rows(); j++){
        JointsMotionState jointMotion;
        jointMotion.setValue(dof);
        vector<double> pos, vel, acc;
        for(int i=0; i<dof; i++){
            pos.push_back(samplePoints[i].at(j, 0));
            vel.push_back(samplePoints[i].at(j, 1));
            acc.push_back(samplePoints[i].at(j, 2));
        }
        jointMotion.setPosValue(Joints(dof, pos));
        jointMotion.setVelValue(Joints(dof, vel));
        jointMotion.setAccValue(Joints(dof, acc));

        m_jointTraj.push_back(jointMotion);

        int ret = m_robot.checkJointWithinPerformance(jointMotion);
        if(ret){
            return ret;
        }
    }

    return 0;
}

JointsMotionStateList MOVEJOINT::getTraj() const{
    return m_jointTraj;
}

void MOVEJOINT::getTraj(JointsMotionStateList &jointTraj) const{
    jointTraj = m_jointTraj;
}


MOVELC::MOVELC(const Robotics &robot){
    m_robot = robot;
    m_sPolynominalP.setPeriod(m_robot.getSamplePeriod());
    m_sPolynominalA.setPeriod(m_robot.getSamplePeriod());
}

MOVELC::~MOVELC(){

}

void MOVELC::setWayPoints(const Terminal &t1, double vel, double acc, double jerk, COORDINATESYSTEM coord){
    m_type = 0;
    m_acc = acc;
    m_jerk = jerk;

    Terminal startTer, endTer;
    if(coord == COORDINATE_BASE){
        startTer = m_robot.getCurrentTerminal();
        endTer = t1;
    }
    else if(coord == COORDINATE_WORK){
        startTer = m_robot.getCurrentTerminal();
        endTer = t1.getTerminalFromWorkFrame(m_robot.getWorkFrame());
    }
    else{
        cout << "MOVELC::Error COORDINATESYSTEM Type!" << endl;
        exit(1);
    }

    m_line.setPoints(startTer.getPoint(), endTer.getPoint());
    m_sPolynominalP.setMotionCoefficientWithVel(m_line.getLength(), vel, m_robot.getTerminalMaxAcc()[TERMINAL_X]*m_acc, m_robot.getTerminalMaxJerk()[TERMINAL_X]*m_jerk);
    m_slerp1.setAttitudes(startTer.getAttitudeAngle().getQuaternion(), endTer.getAttitudeAngle().getQuaternion());

    m_len1 = m_line.getLength();
    m_len2 = m_len1;
    m_len3 = m_len1;
}

void MOVELC::setWayPoints(CIRCLETYPE cir, const Terminal &t1, const Terminal &t2, double vel, double acc, double jerk, COORDINATESYSTEM coord){
    m_type = 1;
    m_acc = acc;
    m_jerk = jerk;

    Terminal startTer, middleTer, endTer;
    if(coord == COORDINATE_BASE){
        startTer = m_robot.getCurrentTerminal();
        middleTer = t1,
        endTer = t2;
    }
    else if(coord == COORDINATE_WORK){
        startTer = m_robot.getCurrentTerminal();
        middleTer = t1.getTerminalFromWorkFrame(m_robot.getWorkFrame());
        endTer = t2.getTerminalFromWorkFrame(m_robot.getWorkFrame());
    }
    else{
        cout << "MOVELC::Error COORDINATESYSTEM Type!" << endl;
        exit(1);
    }

    m_cir.setPoints(startTer.getPoint(), middleTer.getPoint(), endTer.getPoint(), cir);
    m_sPolynominalP.setMotionCoefficientWithVel(m_cir.getLength(), vel, m_robot.getTerminalMaxAcc()[TERMINAL_X]*m_acc, m_robot.getTerminalMaxJerk()[TERMINAL_X]*m_jerk);

    if(cir == WHOLE_CIRCLE){
        m_slerp1.setAttitudes(startTer.getAttitudeAngle().getQuaternion(), startTer.getAttitudeAngle().getQuaternion());
    }
    else{
        m_slerp1.setAttitudes(startTer.getAttitudeAngle().getQuaternion(), endTer.getAttitudeAngle().getQuaternion());
    }
//    m_slerp1.setAttitudes(startTer.getAttitudeAngle().getQuaternion(), middleTer.getAttitudeAngle().getQuaternion());
//    m_slerp2.setAttitudes(middleTer.getAttitudeAngle().getQuaternion(), endTer.getAttitudeAngle().getQuaternion());
//    m_slerp3.setAttitudes(endTer.getAttitudeAngle().getQuaternion(), startTer.getAttitudeAngle().getQuaternion());

    m_len1 = m_cir.getLenFromStartWithPoint(middleTer.getPoint());
    m_len2 = m_cir.getLenFromStartWithPoint(endTer.getPoint());
    m_len3 = m_cir.getLength();
}

void MOVELC::setWayPoints(const Terminal &t1, double vel, double acc, double jerk, double turn, COORDINATESYSTEM coord){
    this->setWayPoints(t1, vel, acc, jerk, coord);

    m_turn = turn;
    if(m_turn*2 >= m_len3-EPSLON){
        m_turn = m_len3/2;
    }
}

void MOVELC::setWayPoints(CIRCLETYPE cir, const Terminal &t1, const Terminal &t2, double vel, double acc, double jerk, double turn, COORDINATESYSTEM coord){
    this->setWayPoints(cir, t1, t2, vel, acc, jerk, coord);

    m_turn = turn;
    if(m_turn*2 >= m_len3-EPSLON){
        m_turn = m_len3/2;
    }
}

int MOVELC::start(){
    m_jointTraj.clear();
    CMatrix<double> samplePoints;
    samplePoints = m_sPolynominalP.getAllSegment();

    JointsMotionState m_lastJointMotionState;
    m_lastJointMotionState.setPosValue(m_robot.getCurrentJointPosition());
    m_lastJointMotionState.setVelValue(Joints(m_robot.getWholeDOF()));
    m_lastJointMotionState.setAccValue(Joints(m_robot.getWholeDOF()));

    for(int i=0; i<samplePoints.rows(); i++){
        Point p;
        Quaternion q;
        if(m_type == 0){
            p = m_line.getPointFormStartWithLen(samplePoints.at(i, 0));
        }
        else{
            p = m_cir.getPointFormStartWithLen(samplePoints.at(i, 0));
        }
        q = m_slerp1.getAttitudeWithRatio(samplePoints.at(i, 0) / m_len3);
//        if(samplePoints.at(i, 0) <= m_len1+EPSLON){
//            q = m_slerp1.getAttitudeWithRatio(samplePoints.at(i, 0) / m_len1);
//        }
//        else if(samplePoints.at(i, 0) <= m_len2+EPSLON){
//            q = m_slerp2.getAttitudeWithRatio((samplePoints.at(i, 0)-m_len1) / (m_len2 - m_len1));
//        }
//        else{
//            q = m_slerp3.getAttitudeWithRatio((samplePoints.at(i, 0)-m_len2) / (m_len3 - m_len2));
//        }
        Terminal ter(p, q.getAttitudeAngle());

        Joints joint;
        int ret = m_robot.inverseKinematics(joint, ter, m_lastJointMotionState.getPosValue(), COORDINATE_BASE);
        if(ret){
            return ret;
        }

        JointsMotionState jointMotion;
        jointMotion.setPosValue(joint);
        jointMotion.calVelAcc(m_lastJointMotionState, m_robot.getSamplePeriod());
        m_lastJointMotionState = jointMotion;

        m_jointTraj.push_back(jointMotion);

        ret = m_robot.checkJointWithinPerformance(jointMotion);
        if(ret){
            return ret;
        }
    }

    return 0;
}

void MOVELC::clear(){
    m_jointTraj.clear();
}

JointsMotionStateList MOVELC::getTraj() const{
    return m_jointTraj;
}

void MOVELC::getTraj(JointsMotionStateList &jointTraj) const{
    jointTraj = m_jointTraj;
}

void MOVELC::getLeftTurnTraj(JointsMotionStateList &jointTraj){
    jointTraj = m_jointTraj;
    double time = getLeftTurnTime();
    int index = time / m_robot.getSamplePeriod();
    for(int i=0; i<index; i++){
        if(jointTraj.size() < 2){
            cout << "MOVELC::getLeftSideTurn no enough trajectory point" << endl;
            break;
        }
        jointTraj.pop_front();
    }
}

void MOVELC::getRightTurnTraj(JointsMotionStateList &jointTraj){
    jointTraj = m_jointTraj;
    double time = getRightTurnTime();
    int index = time / m_robot.getSamplePeriod();
    for(int i=0; i<index; i++){
        if(jointTraj.size() < 2){
            cout << "MOVELC::getRightSideTurn no enough trajectory point" << endl;
            break;
        }
        jointTraj.pop_back();
    }
}

void MOVELC::getBothTureTraj(JointsMotionStateList &jointTraj){
    jointTraj = m_jointTraj;
    double time = getRightTurnTime();
    int index = time / m_robot.getSamplePeriod();
    for(int i=0; i<index; i++){
        if(jointTraj.size() < 2){
            cout << "MOVELC::getBothSideTure no enough trajectory point" << endl;
            break;
        }
        jointTraj.pop_back();
    }

    time = getLeftTurnTime();
    index = time / m_robot.getSamplePeriod();
    for(int i=0; i<index; i++){
        if(jointTraj.size() < 2){
            cout << "MOVELC::getBothSideTure no enough trajectory point" << endl;
            break;
        }
        jointTraj.pop_front();
    }
}

double MOVELC::getLeftTurnTime(){
    if(m_turn < -EPSLON){
        m_turn = m_sPolynominalP.getAccLength();
    }
    return m_sPolynominalP.getTime(m_turn);
}

double MOVELC::getRightTurnTime(){
    if(m_turn < -EPSLON){
        m_turn = m_sPolynominalP.getAccLength();
    }
    return m_sPolynominalP.getTime(m_len3)-m_sPolynominalP.getTime(m_len3 - m_turn);
}

JointsMotionState MOVELC::getLeftTurnPoint(){
    double time = getLeftTurnTime();
    int index = time / m_robot.getSamplePeriod();
    return m_jointTraj[index];
}

JointsMotionState MOVELC::getRightTurnPoint(){
    double time = getRightTurnTime();
    int index = time / m_robot.getSamplePeriod();
    return m_jointTraj[m_jointTraj.size()-1-index];
}


MOVETCON::MOVETCON()
	: m_endState (true)
{

}

MOVETCON::MOVETCON(const Robotics &robot)
	: m_endState(true) 
{
    setRobotics(robot);
    start();
}

MOVETCON::~MOVETCON(){

}

void MOVETCON::setRobotics(const Robotics &robot){
    m_robot = robot;
    m_robotMotion.setCurrentJointPosition(m_robot.getCurrentJointPosition());
//    m_robotMotion.setCurrentJointVelocity(m_robot.getCurrentJointVelocity());
//    m_robotMotion.setCurrentJointAcceleration(m_robot.getCurrentJointAcceleration());
    m_robotMotion.setCurrentTerminal(m_robot.getCurrentTerminal());
    m_robotMotion.setCurrentWorkTerminal(m_robot.getCurrentWorkTerminal());
}

int MOVETCON::addPath(const MOVELC &lc){
    MOVELC movelc = lc;
    int ret = movelc.start();
    if(ret){
        return ret;
    }

    if(!m_moveList.empty()){
        JointsMotionStateList jmsl;
        if(m_moveList.size() == 1){ // 已经存在轨迹，且为第一段轨迹，则取左侧段加中间段
            m_moveList.back().getRightTurnTraj(jmsl);
        }
        else{   // 已经存在轨迹，且不为第一段，则取中间
            m_moveList.back().getBothTureTraj(jmsl);
        }

        JointsMotionState jms1 = m_moveList.back().getRightTurnPoint();
        JointsMotionState jms2 = movelc.getLeftTurnPoint();

        double time = (m_moveList.back().getRightTurnTime() + movelc.getLeftTurnTime())/2;

        Robotics robot = m_robot;
        robot.setCurrentJointPosition(jms1.getPosValue());
        robot.setCurrentJointVelocity(jms1.getVelValue());
        robot.setCurrentJointAcceleration(jms1.getAccValue());
        MOVEJOINT mj(robot);
        JointsList js, vs, as;
        vector<double> ts;
        js.push_back(jms2.getPosValue());
        vs.push_back(jms2.getVelValue());
        as.push_back(jms2.getAccValue());
        ts.push_back(time);
        mj.setWayPointsWithTime(js, vs, as, ts);
        ret = mj.start();
        if(ret){
            return ret;
        }
        JointsMotionStateList jmsc;
        mj.getTraj(jmsc);
        if (!jmsc.empty()){
            jmsc.pop_front();
		}
        if (!jmsc.empty()){
            jmsc.pop_back();
        }
        jmsl.append(jmsc);

        m_jointTraj.push_back(jmsl);
    }

    JointsMotionState jms = movelc.getTraj().back();
    m_robotMotion.setCurrentJointPosition(jms.getPosValue());
//    m_robotMotion.setCurrentJointVelocity(jms.getVelValue());
//    m_robotMotion.setCurrentJointAcceleration(jms.getAccValue());
    Robotics robot = m_robot;
    robot.setCurrentJointPosition(jms.getPosValue());
    m_robotMotion.setCurrentTerminal(robot.getCurrentTerminal());
    m_robotMotion.setCurrentWorkTerminal(robot.getCurrentWorkTerminal());

    m_moveList.push_back(movelc);
    while(m_moveList.size() > 2){
        m_moveList.pop_front();
    }
    return 0;
}

void MOVETCON::endPath(){
    if(!m_moveList.empty()){
        JointsMotionStateList jmsl;
        if(m_moveList.size() == 1){
            m_moveList.back().getTraj(jmsl);
        }
        else{
            m_moveList.back().getLeftTurnTraj(jmsl);
        }
        m_jointTraj.push_back(jmsl);
    }
    m_moveList.clear();
}

void MOVETCON::setEnd(){
    m_endState = true;
}

bool MOVETCON::isEnd() const{
    return m_endState;
}

RobotMotion MOVETCON::getLastRobotMotion() const{
    return m_robotMotion;
}

void MOVETCON::start(){
    m_jointTraj.clear();
    m_moveList.clear();
    m_endState = false;
}

int MOVETCON::getTrajNum() const{
    return m_jointTraj.size();
}

JointsMotionStateList MOVETCON::getTraj(){
    JointsMotionStateList jointTraj;
    if(!m_jointTraj.empty()){
        jointTraj = m_jointTraj.front();
        m_jointTraj.pop_front();
    }
    return jointTraj;
}

void MOVETCON::getTraj(JointsMotionStateList &jointTraj){
    if(!m_jointTraj.empty()){
        jointTraj = m_jointTraj.front();
        m_jointTraj.pop_front();
    }
}


MOVEBCURVE::MOVEBCURVE(const Robotics &robot){
    m_robot = robot;
}

MOVEBCURVE::~MOVEBCURVE(){

}

void MOVEBCURVE::setWayPoints(const TerminalList &terminals, std::vector<double> vel, double acc, double jerk, double angle, double precision, COORDINATESYSTEM coord){

}

int MOVEBCURVE::start(){
    return 0;
}

JointsMotionStateList MOVEBCURVE::getTraj() const{
    return m_jointTraj;
}

void MOVEBCURVE::getTraj(JointsMotionStateList& jointTraj) const{
    jointTraj = m_jointTraj;
}

#endif
