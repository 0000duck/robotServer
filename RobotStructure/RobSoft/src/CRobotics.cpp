#ifndef CROBOTICS_CPP
#define CROBOTICS_CPP

#include "CRobotics.hpp"
#include "CMathBasic.hpp"
#include "CErrorCode.hpp"

using namespace std;
using namespace robsoft;

Robotics::Robotics(){

}

Robotics::~Robotics(){

}

void Robotics::setCurrentJointPosition(const Joints& currentJointPosition){
    m_currentJointPosition = currentJointPosition;
}

void Robotics::setCurrentJointVelocity(const Joints& currentJointVelocity){
    m_currentJointVelocity = currentJointVelocity;
}

void Robotics::setCurrentJointAcceleration(const Joints& currentJointAcceleration){
    m_currentJointAcceleration = currentJointAcceleration;
}

void Robotics::setCurrentJointTorque(const Joints& currentJointTorque){
    m_currentJointTorque = currentJointTorque;
}

Joints Robotics::getCurrentJointPosition() const{
    return m_currentJointPosition;
}

Joints Robotics::getCurrentJointVelocity() const{
    return m_currentJointVelocity;
}

Joints Robotics::getCurrentJointAcceleration() const{
    return m_currentJointAcceleration;
}

Joints Robotics::getCurrentJointTorque() const{
    return m_currentJointTorque;
}

Terminal Robotics::getCurrentTerminal(){
    m_currentTerminal = this->forwardKinematics(m_currentJointPosition);
    return m_currentTerminal;
}

Terminal Robotics::getCurrentWorkTerminal(){
    m_currentTerminal = this->forwardKinematics(m_currentJointPosition);
    m_currentWorkTerminal = m_currentTerminal.getTerminalInWorkFrame(m_workFrame);
    return m_currentWorkTerminal;
}

Terminal Robotics::forwardKinematics(const Joints& joints, bool toolEnabled) const{
    CMatrix<double> mat;
    mat.eye(4, 4);

    HomogeneousMatrix homo(mat);

    for(int i=0; i<joints.getJointsDOF(); i++){
        JOINTINDEX index = JOINTINDEX(i);
        double dhalp = m_DHParameterAlpha.getValue(index);
        double dha = m_DHParameterA.getValue(index);
        double dhd = m_DHParameterD.getValue(index);
        double dht = m_DHParameterTheta.getValue(index);
        double jv = joints.getValue(index);
        if(m_robotType == ROBSOFT_SERIAL_FOUR_CONVENTION && index == JOINT_4){
            jv = -(joints.getValue(JOINT_2)+joints.getValue(JOINT_3));
        }

        if(m_robotType == ROBOTSOFT_SCARA_FOUR_FOURRF && index == JOINT_4){
            HomogeneousMatrix tempHomo = this->DHParameterToHomogeneousMatrix(dhalp, dha, jv+dhd, dht);
            homo = homo * tempHomo;
            break;
        }

        if(m_robotType == ROBOTSOFT_SCARA_FOUR_ONERF && index == JOINT_1){
            HomogeneousMatrix tempHomo = this->DHParameterToHomogeneousMatrix(dhalp, dha, jv+dhd, dht);
            homo = homo * tempHomo;
            continue;
        }

        HomogeneousMatrix tempHomo = this->DHParameterToHomogeneousMatrix(dhalp, dha, dhd, jv+dht);
        homo = homo * tempHomo;
    }

    if(toolEnabled){
        homo = homo * this->getToolFrame().getHomogeneousMatrix();
    }

    return homo.getTerminal();
}

int Robotics::inverseKinematics(Joints& joints, const Terminal& terminal, const Joints& lastJoints, COORDINATESYSTEM coord) const{
    Terminal baseTerminal = terminal;
    if(coord == COORDINATE_WORK){
        baseTerminal = baseTerminal.getTerminalFromWorkFrame(this->getWorkFrame());
    }
    JointsList jointslist;
    switch(this->getRobotType()){
    case ROBSOFT_SERIAL_SIX_CONVENTION: jointslist = this->inverse_kinematics_serial_six_convention(baseTerminal, lastJoints);
        break;
    case ROBSOFT_SERIAL_SIX_COOPERATION: jointslist = this->inverse_kinematics_serial_six_cooperation(baseTerminal, lastJoints);
        break;
    case ROBSOFT_SERIAL_FOUR_CONVENTION: jointslist = this->inverse_kinematics_serial_four_convention(baseTerminal, lastJoints);
        break;
    case ROBOTSOFT_SCARA_FOUR_ONERF: jointslist = this->inverse_kinematics_scara_four_onerf(baseTerminal, lastJoints);
        break;
    case ROBOTSOFT_SCARA_FOUR_FOURRF: jointslist = this->inverse_kinematics_scara_four_fourrf(baseTerminal, lastJoints);
        break;
    case ROBOTSOFT_DELTA_FOUR: jointslist = this->inverse_kinematics_delta_four(baseTerminal, lastJoints);
        break;
    case ROBOTSOFT_DELTA_SIX: jointslist = this->inverse_kinematics_delta_six(baseTerminal, lastJoints);
        break;
    default:
        break;
    }

    if(jointslist.empty()){
        return 1;
    }

//    for(int i=0; i<jointslist.size(); i++){
//        jointslist[i].print();
//    }

    joints = chooseBestJoints(jointslist, lastJoints);

    return 0;
}

int Robotics::inverseKinematics(JointsList& jointslist, const TerminalList &terminalList, const Joints &lastJoints, COORDINATESYSTEM coord) const{
    jointslist.clear();
    Joints currentJoint = lastJoints;
    for(int i=0; i<terminalList.size(); i++){
        if(this->inverseKinematics(currentJoint, terminalList[i], currentJoint, coord)){
            return 1;
        }
        jointslist.push_back(currentJoint);
    }

    return 0;
}

HomogeneousMatrix Robotics::DHParameterToHomogeneousMatrix(double alpha, double a, double d, double theta) const{
    double r11 = cos_deg(theta);
    double r21 = sin_deg(theta)*cos_deg(alpha);
    double r31 = sin_deg(theta)*sin_deg(alpha);
    double r12 = -sin_deg(theta);
    double r22 = cos_deg(theta) * cos_deg(alpha);
    double r32 = cos_deg(theta) * sin_deg(alpha);
    double r13 = 0;
    double r23 = -sin_deg(alpha);
    double r33 = cos_deg(alpha);
    double r14 = a;
    double r24 = -sin_deg(alpha)*d;
    double r34 = cos_deg(alpha)*d;

    double value[] = {r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34, 0, 0, 0, 1};

    return HomogeneousMatrix(CMatrix<double>(4, 4, value));
}

JointsList Robotics::inverse_kinematics_serial_six_convention(const Terminal &terminal, const Joints& joints) const{
    double a1 = m_DHParameterA.getValue(JOINT_2);
    double a2 = m_DHParameterA.getValue(JOINT_3);
    double a3 = m_DHParameterA.getValue(JOINT_4);
    double d1 = m_DHParameterD.getValue(JOINT_1);
    double d3 = m_DHParameterD.getValue(JOINT_3);
    double d4 = m_DHParameterD.getValue(JOINT_4);
    double d6 = m_DHParameterD.getValue(JOINT_6);

    Terminal tempBase(0, 0, d1, 0, 0, 0);
    HomogeneousMatrix T = (terminal-tempBase).getHomogeneousMatrix();
    T = T*m_toolFrame.getHomogeneousMatrix().inv();
    T = T*this->DHParameterToHomogeneousMatrix(0, 0, d6, 0).inv();

    double r11 = T.getValue(0, 0);
    double r12 = T.getValue(0, 1);
    double r13 = T.getValue(0, 2);
    double r21 = T.getValue(1, 0);
    double r22 = T.getValue(1, 1);
    double r23 = T.getValue(1, 2);
    double r31 = T.getValue(2, 0);
    double r32 = T.getValue(2, 1);
    double r33 = T.getValue(2, 2);
    double px = T.getValue(0, 3);
    double py = T.getValue(1, 3);
    double pz = T.getValue(2, 3);

    JointsList jlist;

    double rho = sqrt(px*px+py*py);
    double cphi1 = d3/rho;
    if(num_is_zero(rho) || cphi1>1){
        return jlist;
    }

    double tempTheta1[2];
    tempTheta1[0] = atan2_deg(py, px)-atan2_deg(cphi1, sqrt(1-cphi1*cphi1));
    tempTheta1[1] = atan2_deg(py, px)-atan2_deg(cphi1, -sqrt(1-cphi1*cphi1));

    for(int i=0; i<2; i++){
        double theta1 = tempTheta1[i];
        fitJointRange(theta1, JOINT_1);

        double K = (px*px+py*py+pz*pz+a1*a1-2*a1*px*cos_deg(theta1)-2*a1*py*sin_deg(theta1)-a2*a2-a3*a3-d3*d3-d4*d4)/(2*a2);
        K = K / sqrt(a3*a3+d4*d4);
        if(K>1 || K<-1){
            continue;
        }

        double tempTheta3[2];
        tempTheta3[0] = atan2_deg(a3, d4)-atan2_deg(K, sqrt(1-K*K));
        tempTheta3[1] = atan2_deg(a3, d4)-atan2_deg(K, -sqrt(1-K*K));
        for(int j=0; j<2; j++){
            double theta3 = tempTheta3[j];
            fitJointRange(theta3, JOINT_3);

            double c1 = cos_deg(theta1);
            double s1 = sin_deg(theta1);
            double c3 = cos_deg(theta3);
            double s3 = sin_deg(theta3);

            double theta23 = atan2_deg(((a3+a2*c3)*(px*c1+py*s1-a1)-(d4-a2*s3)*pz)/((px*c1+py*s1)*(px*c1+py*s1)+pz*pz),
                                   ((a3+a2*c3)*pz+(d4-a2*s3)*(px*c1+py*s1-a1))/((px*c1+py*s1)*(px*c1+py*s1)+pz*pz));
            double theta2 = theta23 - theta3;
            fitJointRange(theta2, JOINT_2);

            double s23 = sin_deg(theta2 + theta3);
            double c23 = cos_deg(theta2 + theta3);

            double s4 = -r13*s1+r23*c1;
            double c4 = -(r13*c1*s23+r23*s1*s23+r33*c23);

            double theta4;
            if(num_is_zero(s4) && num_is_zero(c4)){
                theta4 = joints.getValue(JOINT_4);
            }
            else{
                theta4 = atan2_deg(s4, c4);
            }
            fitJointRange(theta4, JOINT_4);
            s4 = sin_deg(theta4);
            c4 = cos_deg(theta4);

            double s5 = -r13*c1*c23-r23*s1*c23+r33*s23;
            double c5 = -r13*(c1*s23*c4+s1*s4)-r23*(s1*s23*c4-c1*s4)-r33*c23*c4;
            double theta5 = atan2_deg(s5, c5);
            fitJointRange(theta5, JOINT_5);

            double s6 = r11*(-c1*s23*s4+s1*c4)+r21*(-s1*s23*s4-c1*c4)-r31*c23*s4;
            double c6 = r12*(-c1*s23*s4+s1*c4)+r22*(-s1*s23*s4-c1*c4)-r32*c23*s4;
            double theta6 = atan2_deg(s6, c6);
            fitJointRange(theta6, JOINT_6);

            double tempV[] = {theta1, theta2, theta3, theta4, theta5, theta6};
            Joints tempJ1(6, tempV);
            jlist.push_back(tempJ1);

            tempV[3] += 180;
            fitJointRange(tempV[3], JOINT_4);
            tempV[4] = -tempV[4]-180;
            fitJointRange(tempV[4], JOINT_5);
            tempV[5] += 180;
            fitJointRange(tempV[5], JOINT_6);
            Joints tempJ2(6, tempV);
            jlist.push_back(tempJ2);
        }
    }

    for(int i=0; i<jlist.size(); i++){
        if(!isAvailableJoints(jlist[i])){
            jlist.erase(i);
        }
    }
    return jlist;
}

JointsList Robotics::inverse_kinematics_serial_six_cooperation(const Terminal &terminal, const Joints& joints) const{
    double a2 = m_DHParameterA.getValue(JOINT_3);
    double a3 = m_DHParameterA.getValue(JOINT_4);
    double d1 = m_DHParameterD.getValue(JOINT_1);
    double d4 = m_DHParameterD.getValue(JOINT_4);
    double d5 = m_DHParameterD.getValue(JOINT_5);
    double d6 = m_DHParameterD.getValue(JOINT_6);

    Terminal tempBase(0, 0, d1, 0, 0, 0);
    HomogeneousMatrix T = (terminal-tempBase).getHomogeneousMatrix();
    T = T*m_toolFrame.getHomogeneousMatrix().inv();
    T = T*this->DHParameterToHomogeneousMatrix(0, 0, d6, 0).inv();

    double r11 = T.getValue(0, 0);
    double r12 = T.getValue(0, 1);
    double r13 = T.getValue(0, 2);
    double r21 = T.getValue(1, 0);
    double r22 = T.getValue(1, 1);
    double r23 = T.getValue(1, 2);
    double r31 = T.getValue(2, 0);
    double r32 = T.getValue(2, 1);
    double r33 = T.getValue(2, 2);
    double px = T.getValue(0, 3);
    double py = T.getValue(1, 3);
    double pz = T.getValue(2, 3);

    JointsList jlist;

    double rho = sqrt(px*px+py*py);
    double cphi1 = d4/rho;
    if(num_is_zero(rho) || cphi1>1){
        return jlist;
    }

    double tempTheta1[2];
    tempTheta1[0] = atan2_deg(py, px)-atan2_deg(cphi1, sqrt(1-cphi1*cphi1));
    tempTheta1[1] = atan2_deg(py, px)-atan2_deg(cphi1, -sqrt(1-cphi1*cphi1));

    for(int i=0; i<2; i++){
        double theta1 = tempTheta1[i];
        fitJointRange(theta1, JOINT_1);
        double s1 = sin_deg(theta1);
        double c1 = cos_deg(theta1);

        double s5 = -r13*s1+r23*c1;
        double tempTheta5[2];
        tempTheta5[0] = atan2_deg(s5, sqrt(1-s5*s5));
        tempTheta5[1] = atan2_deg(s5, -sqrt(1-s5*s5));

        for(int j=0; j<2; j++){
            double theta5 = tempTheta5[j];
            fitJointRange(theta5, JOINT_5);
            double c5 = cos_deg(theta5);

            double theta6;
            if(num_is_zero(c5)){
                theta6 = joints.getValue(JOINT_6);
            }
            else{
                theta6 = atan2_deg(r12*s1-r22*c1, -r11*s1+r21*c1);
            }
            fitJointRange(theta6, JOINT_6);
            double s6 = sin_deg(theta6);
            double c6 = cos_deg(theta6);

            double px2 = -d5*s6*(r11*c1+r21*s1)-d5*c6*(r12*c1+r22*s1)+px*c1+py*s1;
            double py2 = -d5*s6*(-r11*s1+r21*c1)-d5*c6*(-r12*s1+r22*c1)-px*s1+py*c1;
            double pz2 = -r31*d5*s6-r32*d5*c6+pz;

            double c3 = (px2*px2+py2*py2+pz2*pz2-a2*a2-a3*a3-d4*d4)/(2*a2*a3);

            if(c3<-1 || c3>1){
                continue;
            }

            double tempTheta3[2];
            tempTheta3[0] = atan2_deg(sqrt(1-c3*c3), c3);
            tempTheta3[1] = atan2_deg(-sqrt(1-c3*c3), c3);

            for(int k=0; k<2; k++){
                double theta3 = tempTheta3[k];
                fitJointRange(theta3, JOINT_3);
                double s3 = sin_deg(theta3);

                double s2 = (px2*(a3*c3+a2)-pz2*a3*s3)/((a3*c3+a2)*(a3*c3+a2)+a3*a3*s3*s3);
                double c2 = (px2*a3*s3+pz2*(a3*c3+a2))/((a3*c3+a2)*(a3*c3+a2)+a3*a3*s3*s3);

                double theta2 = atan2_deg(s2, c2);
                fitJointRange(theta2, JOINT_2);

                double c234 = r31*s6+r32*c6;
                double s234 = s6*(r11*c1+r21*s1)+c6*(r12*c1+r22*s1);

                double theta4 = atan2_deg(s234, c234)-theta2-theta3;
                fitJointRange(theta4, JOINT_4);

                double tempV[] = {theta1, theta2, theta3, theta4, theta5, theta6};
                Joints tempJ1(6, tempV);
                jlist.push_back(tempJ1);
            }
        }
    }
    for(int i=0; i<jlist.size(); i++){
        if(!isAvailableJoints(jlist[i])){
            jlist.erase(i);
        }
    }
    return jlist;
}

JointsList Robotics::inverse_kinematics_serial_four_convention(const Terminal &terminal, const Joints& joints) const{
    double a1 = m_DHParameterA.getValue(JOINT_2);
    double a2 = m_DHParameterA.getValue(JOINT_3);
    double a3 = m_DHParameterA.getValue(JOINT_4);
    double a4 = m_DHParameterA.getValue(JOINT_5);
    double d1 = m_DHParameterD.getValue(JOINT_1);
    double d4 = m_DHParameterD.getValue(JOINT_4);
    double d5 = m_DHParameterD.getValue(JOINT_5);

    Terminal tempBase(0, 0, d1, 0, 0, 0);
    HomogeneousMatrix T = (terminal-tempBase).getHomogeneousMatrix();
    T = T*m_toolFrame.getHomogeneousMatrix().inv();
    T = T*this->DHParameterToHomogeneousMatrix(0, 0, d5, 0).inv();

    double r11 = T.getValue(0, 0);
    double r12 = T.getValue(0, 1);
    double r13 = T.getValue(0, 2);
    double r21 = T.getValue(1, 0);
    double r22 = T.getValue(1, 1);
    double r23 = T.getValue(1, 2);
    double r31 = T.getValue(2, 0);
    double r32 = T.getValue(2, 1);
    double r33 = T.getValue(2, 2);
    double px = T.getValue(0, 3);
    double py = T.getValue(1, 3);
    double pz = T.getValue(2, 3);

    JointsList jlist;

    if(!num_is_zero(r33+1)){ // r33 != -1
        return jlist;
    }

    double rho = sqrt(px*px+py*py);
    double cphi1 = d4/rho;
    if(num_is_zero(rho) || cphi1>1){
        return jlist;
    }

    double tempTheta1[2];
    tempTheta1[0] = atan2_deg(py, px)-atan2_deg(cphi1, sqrt(1-cphi1*cphi1));
    tempTheta1[1] = atan2_deg(py, px)-atan2_deg(cphi1, -sqrt(1-cphi1*cphi1));

    for(int i=0; i<2; i++){
        double theta1 = tempTheta1[i];
        fitJointRange(theta1, JOINT_1);
        double s1 = sin_deg(theta1);
        double c1 = cos_deg(theta1);

        double c5 = r11*c1+r21*s1;
        double s5 = -r12*c1-r22*s1;

        double theta5 = atan2_deg(s5, c5);
        fitJointRange(theta5, JOINT_5);

        double s3 = (a3*a3+a2*a2-pow(px*c1+py*s1-a4-a1, 2)-pz*pz)/(2*a2*a3);
        if(fabs(s3) > 1){
            continue;
        }

        double tempTheta3[2];
        tempTheta3[0] = atan2_deg(s3, sqrt(1-s3*s3));
        tempTheta3[1] = atan2_deg(s3, -sqrt(1-s3*s3));

        for(int j=0; j<2; j++){
            double theta3 = tempTheta3[j];
            fitJointRange(theta3, JOINT_3);

            double s3 = sin_deg(theta3);
            double c3 = cos_deg(theta3);

            double c2 = ((px*c1+py*s1-a4-a1)*a3*c3+pz*(a2-a3*s3))/(pow(a3*c3,2)+pow((a2-a3*s3),2));
            double s2 = ((px*c1+py*s1-a4-a1)*(a2-a3*s3)-pz*a3*c3)/(pow((a2-a3*s3),2)+pow(a3*c3,2));

            double theta2 = atan2_deg(s2, c2);
            fitJointRange(theta2, JOINT_2);

            double theta4 = -(theta2+theta3);
            fitJointRange(theta4, JOINT_4);

            double tempV[] = {theta1, theta2, theta3, theta4, theta5};
            Joints tempJ1(5, tempV);
            jlist.push_back(tempJ1);
        }
    }

    for(int i=0; i<jlist.size(); i++){
        if(!isAvailableJoints(jlist[i])){
            jlist.erase(i);
        }
    }
    return jlist;
}

JointsList Robotics::inverse_kinematics_scara_four_onerf(const Terminal &terminal, const Joints& joints) const{
    double a1 = m_DHParameterA.getValue(JOINT_2);
    double a2 = m_DHParameterA.getValue(JOINT_3);
    double a3 = m_DHParameterA.getValue(JOINT_4);
    double d1 = m_DHParameterD.getValue(JOINT_1);

    Terminal tempBase(0, 0, -d1, 0, 0, 0);
    HomogeneousMatrix T = (terminal-tempBase).getHomogeneousMatrix();
    T = T*m_toolFrame.getHomogeneousMatrix().inv();
//    T = T*this->DHParameterToHomogeneousMatrix(0, 0, d5, 0).inv();

    double r11 = T.getValue(0, 0);
    double r12 = T.getValue(0, 1);
    double r13 = T.getValue(0, 2);
    double r21 = T.getValue(1, 0);
    double r22 = T.getValue(1, 1);
    double r23 = T.getValue(1, 2);
    double r31 = T.getValue(2, 0);
    double r32 = T.getValue(2, 1);
    double r33 = T.getValue(2, 2);
    double px = T.getValue(0, 3);
    double py = T.getValue(1, 3);
    double pz = T.getValue(2, 3);

    JointsList jlist;

    if(!num_is_zero(r33+1)){ // r33 != -1
        return jlist;
    }

    double theta1 = -pz;

    double c3 = (pow(px-a1,2)+py*py-a2*a2-a3*a3)/(2*a2*a3);
    if(c3 > 1){
        return jlist;
    }
    double tempTheta3[2];
    tempTheta3[0] = atan2_deg(sqrt(1-c3*c3), c3);
    tempTheta3[1] = atan2_deg(-sqrt(1-c3*c3), c3);

    for(int i=0; i<2; i++){
        double theta3 = tempTheta3[i];
        fitJointRange(theta3, JOINT_3);
        double s3 = sin_deg(theta3);

        double c2 = ((px-a1)*(a2+a3*c3)-py*a3*s3)/(pow(a2+a3*c3,2)+pow(a3*s3,2));
        double s2 = -((px-a1)*a3*s3+py*(a2+a3*c3))/(pow(a2+a3*c3,2)+pow(a3*s3,2));

        double theta2 = atan2_deg(s2, c2);
        fitJointRange(theta2, JOINT_2);

        double c234 = r11;
        double s234 = -r12;

        double theta234 = atan2_deg(s234, c234);

        double theta4 = theta234-theta2-theta3;
        fitJointRange(theta4, JOINT_4);

        double tempV[] = {theta1, theta2, theta3, theta4};
        Joints tempJ1(4, tempV);
        jlist.push_back(tempJ1);
    }

    for(int i=0; i<jlist.size(); i++){
        if(!isAvailableJoints(jlist[i])){
            jlist.erase(i);
        }
    }
    return jlist;
}

JointsList Robotics::inverse_kinematics_scara_four_fourrf(const Terminal &terminal, const Joints& joints) const{
    double a1 = m_DHParameterA.getValue(JOINT_2);
    double a2 = m_DHParameterA.getValue(JOINT_3);
    double d1 = m_DHParameterD.getValue(JOINT_1);

    Terminal tempBase(0, 0, -d1, 0, 0, 0);
    HomogeneousMatrix T = (terminal-tempBase).getHomogeneousMatrix();
    T = T*m_toolFrame.getHomogeneousMatrix().inv();
//    T = T*this->DHParameterToHomogeneousMatrix(0, 0, d5, 0).inv();

    double r11 = T.getValue(0, 0);
    double r12 = T.getValue(0, 1);
    double r13 = T.getValue(0, 2);
    double r21 = T.getValue(1, 0);
    double r22 = T.getValue(1, 1);
    double r23 = T.getValue(1, 2);
    double r31 = T.getValue(2, 0);
    double r32 = T.getValue(2, 1);
    double r33 = T.getValue(2, 2);
    double px = T.getValue(0, 3);
    double py = T.getValue(1, 3);
    double pz = T.getValue(2, 3);

    JointsList jlist;

    if(!num_is_zero(r33+1)){ // r33 != -1
        return jlist;
    }

    double theta4 = -pz;

    double c2 = (px*px+py*py-a1*a1-a2*a2)/(2*a1*a2);
    if(c2 > 1){
        return jlist;
    }
    double tempTheta2[2];
    tempTheta2[0] = atan2_deg(sqrt(1-c2*c2), c2);
    tempTheta2[1] = atan2_deg(-sqrt(1-c2*c2), c2);

    for(int i=0; i<2; i++){
        double theta2 = tempTheta2[i];
        fitJointRange(theta2, JOINT_2);
        double s2 = sin_deg(theta2);

        double c1 = (px*(a1+a2*c2)-py*a2*s2)/(pow(a1+a2*c2,2)+pow(a2*s2,2));
        double s1 = -(px*a2*s2+py*(a1+a2*c2))/(pow(a1+a2*c2,2)+pow(a2*s2,2));

        double theta1 = atan2_deg(s1, c1);
        fitJointRange(theta1, JOINT_1);

        double c123 = r11;
        double s123 = -r12;

        double theta123 = atan2_deg(s123, c123);

        double theta3 = theta123-theta1-theta2;
        fitJointRange(theta3, JOINT_3);

        double tempV[] = {theta1, theta2, theta3, theta4};
        Joints tempJ1(4, tempV);
        jlist.push_back(tempJ1);
    }

    for(int i=0; i<jlist.size(); i++){
        if(!isAvailableJoints(jlist[i])){
            jlist.erase(i);
        }
    }
    return jlist;
}

JointsList Robotics::inverse_kinematics_delta_four(const Terminal &terminal, const Joints& joints) const{
	return JointsList();
}

JointsList Robotics::inverse_kinematics_delta_six(const Terminal &terminal, const Joints& joints) const{
	return JointsList();
}

int Robotics::checkJointWithinPerformance(const JointsMotionState& jointMotion){
//    if(jointMotion.getPosValue().judgeOverMinimum(this->getJointRangeMinus()) && ){
//        return OVERRANGE;
//    }
//    if(jointMotion.getPosValue().judgeOverMaximum(this->getJointRangePlus())){
//        return OVERRANGE;
//    }
    for(int i=0; i<jointMotion.getJointsDOF(); i++){
        if(jointMotion.getPosValue()[JOINTINDEX(i)]<this->getJointRangeMinus()[JOINTINDEX(i)]){
            if(jointMotion.getVelValue()[JOINTINDEX(i)]<-EPSLON){
                return OVERRANGE;
            }
        }
    }
    for(int i=0; i<jointMotion.getJointsDOF(); i++){
        if(jointMotion.getPosValue()[JOINTINDEX(i)]>this->getJointRangePlus()[JOINTINDEX(i)]){
            if(jointMotion.getVelValue()[JOINTINDEX(i)]>EPSLON){
                return OVERRANGE;
            }
        }
    }

    if(jointMotion.getVelValue().judgeOverMaximum(this->getJointMaxVelRange())){
        return OVERVELOCITY;
    }
    if(jointMotion.getAccValue().judgeOverMaximum(this->getJointMaxAccRange())){
        return OVERACCELERATION;
    }
    return NOERROR_ROB;
}

Joints Robotics::chooseBestJoints(JointsList jointslist, Joints joints) const{
    int index;
    double min;
    for(int i=0; i<jointslist.size(); i++){
//        jointslist[i].print();
        if(i == 0){
            min = jointslist[i].norm(joints);
            index = i;
        }
        else{
            double temp = jointslist[i].norm(joints);
            if(min > temp){
                min = temp;
                index = i;
            }
        }
    }
    return jointslist[index];
}

bool Robotics::isAvailableJoints(Joints joints) const{
    for(int i=0; i<joints.getJointsDOF(); i++){
        JOINTINDEX index = JOINTINDEX(i);
        if(joints.getValue(index)<m_jointRangeMinus.getValue(index) || joints.getValue(index)>m_jointRangePlus.getValue(index)){
            return false;
        }
    }
    return true;
}

void Robotics::fitJointRange(double &angle, JOINTINDEX index) const{
    if(angle >= m_jointRangeMinus.getValue(index) && angle <= m_jointRangePlus.getValue(index)){
        return;
    }

    while(angle < m_jointRangeMinus.getValue(index)){
        angle += 2*180;
    }

    while(angle > m_jointRangePlus.getValue(index)){
        angle -= 2*180;
    }
}

double Robotics::TCPCalibrate(const JointsList &jointslist, Terminal &frame){
    if(jointslist.size() < 3 || jointslist.size() > 10){
        cout << "Robotics::3-10 joint points are needed for TCP calibration" << endl;
        return -1;
    }

    vector<RotateMatrix> vecR;
    vector<Point> vecP;
    for(int i=0; i<jointslist.size(); i++){
        Terminal end = forwardKinematics(jointslist[i], false);
        vecR.push_back(end.getRotateMatrix());
        vecP.push_back(end.getPoint());
    }

    RotateMatrix sumR;
    Point sumP;
    sumR = sumR - sumR; // 清零
    sumP = sumP - sumP; // 清零
    for(int i=0; i<jointslist.size()-1; i++){
        RotateMatrix eyeR;
        sumR = sumR + (eyeR * 2 - (vecR[i].trans() * vecR[i+1] + vecR[i+1].trans() * vecR[i]));
        sumP = sumP + (vecR[i] - vecR[i+1]).trans() * (vecP[i+1] - vecP[i]);
    }

    Point tcpP = sumR.inv() * sumP;

    double pre = 0;
    for(int i=0; i<jointslist.size()-1; i++){
        Point p = (vecR[i] - vecR[i+1]) * tcpP - (vecP[i+1] - vecP[i]);
        pre += pow(p.norm(), 2);
    }
    pre = sqrt(pre);

    frame.setValue(tcpP, RotateMatrix());
    return pre;
}

double Robotics::TCPCalibrateZ(const JointsList &jointslist, const Joints &jo, const Joints &jz, Terminal &frame){
    double pre = TCPCalibrate(jointslist, frame);
    if(pre < 0){
        return pre;
    }

    HomogeneousMatrix hm = frame.getHomogeneousMatrix();
    HomogeneousMatrix hmo, hmz;
    RotateMatrix rmo, rmz;
    Point po, pz;
    hmo = forwardKinematics(jo, false).getHomogeneousMatrix()/* * hm*/; // 腕部相对于基地
    hmz = forwardKinematics(jz, false).getHomogeneousMatrix()/* * hm*/; // 腕部相对于基地
    rmo = hmo.getRotateMatrix();
    po = hmo.getPoint();
    rmz = hmz.getRotateMatrix();
    pz = hmz.getPoint();

    Point poz = rmz * frame.getPoint() - rmo * frame.getPoint() + pz - po; // oz向量，相对于基底的方向向量
    UnitVector uvoz = rmo.inv() * poz; // 计算相对于腕部的方向向量
//    if(num_is_zero(Point(1, 0, 0).cross(uvoz).norm())){
//        cout << "Robotics::invalid z axis data" << endl;
//        return -1;
//    }

    DualVector dv;
    dv.setDirVector(uvoz);

    frame.setValue(frame.getPoint(), dv.getAttitudeAngle());

    return pre;
}

double Robotics::TCPCalibrateXZ(const JointsList &jointslist, const Joints &jo, const Joints &jx, const Joints &jz, Terminal &frame){
    double pre = TCPCalibrate(jointslist, frame);
    if(pre < 0){
        return pre;
    }

    HomogeneousMatrix hm = frame.getHomogeneousMatrix();
    HomogeneousMatrix hmo, hmz, hmx;
    RotateMatrix rmo, rmz, rmx;
    Point po, pz, px;
    hmo = forwardKinematics(jo, false).getHomogeneousMatrix()/* * hm*/;
    hmz = forwardKinematics(jz, false).getHomogeneousMatrix()/* * hm*/;
    hmx = forwardKinematics(jx, false).getHomogeneousMatrix()/* * hm*/;
    rmo = hmo.getRotateMatrix();
    po = hmo.getPoint();
    rmz = hmz.getRotateMatrix();
    pz = hmz.getPoint();
    rmx = hmx.getRotateMatrix();
    px = hmx.getPoint();

    Point pox = rmx * frame.getPoint() - rmo * frame.getPoint() + px - po;
    UnitVector uvox = rmo.inv() * pox;
    Point poz = rmz * frame.getPoint() - rmo * frame.getPoint() + pz - po;
    UnitVector uvoz = rmo.inv() * poz;

    DualVector dv;
    dv.setValue(uvoz, uvox);

    frame.setValue(frame.getPoint(), dv.getAttitudeAngle());
    return pre;
}

void Robotics::workFrameCalibrate(const Terminal &to, const Terminal &tx, const Terminal &ty, Terminal &frame){
    CMatrix<double> mat(4, 4);
    for(int i=0; i<3; i++){
        mat.at(i, 3) = to.getValue(TERMINALINDEX(i));
    }
    mat.at(3, 3) = 1;

    UnitVector uvox = (tx - to).getPoint();
    UnitVector uvoy = (ty - to).getPoint(); // 此时的ty仅要求在oxy平面上，所以求得的单位向量uvoy并不是真正的ｙ轴
    UnitVector uvoz = uvox.cross(uvoy);
    uvoy = uvoz.cross(uvox);

    // 根据旋转矩阵的定义赋值
    for(int i=0; i<3; i++){
        mat.at(i, 0) = uvox.getValue(POINTINDEX(i));
        mat.at(i, 1) = uvoy.getValue(POINTINDEX(i));
        mat.at(i, 2) = uvoz.getValue(POINTINDEX(i));
    }

    frame = HomogeneousMatrix(mat).getTerminal();
}

#endif
