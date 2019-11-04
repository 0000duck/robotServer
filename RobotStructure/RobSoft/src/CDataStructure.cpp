#ifndef CDATASTRUCTURES_CPP
#define CDATASTRUCTURES_CPP

#include "CDataStructure.hpp"
#include "CMathBasic.hpp"

#include <iostream>

using namespace std;
using namespace robsoft;

// class Joints
Joints::Joints(){
    this->setValue(6);
}

Joints::~Joints(){

}

Joints::Joints(int dof){
    this->setValue(dof);
}

Joints::Joints(int dof, double *value){
    this->setValue(dof, value);
}

Joints::Joints(int dof, const vector<double> &value){
    this->setValue(dof, value);
}

Joints::Joints(int dof, const CMatrix<double>& value){
    this->setValue(dof, value);
}

void Joints::setValue(int dof){
    m_dof = dof;
    m_value.zeros(m_dof, 1);
}

void Joints::setValue(int dof, double *value){
    m_dof = dof;
    m_value.setValue(m_dof, 1, value);
}

void Joints::setValue(int dof, const vector<double> &value){
    m_dof = dof;
    m_value.setValue(m_dof, 1, value);
}

void Joints::setValue(int dof, const CMatrix<double>& value){
    m_dof = dof;
    m_value = value;
}

void Joints::append(const Joints &joint){
    m_dof += joint.getJointsDOF();
    m_value.appendRow(joint.m_value);
}

int Joints::getJointsDOF() const{
    return m_dof;
}

double Joints::getValue(JOINTINDEX index) const{
    return m_value.at(index, 0);
}

CMatrix<double> Joints::getValue() const{
    return m_value;
}

void Joints::getValue(double* value) const{
    for(int i=0; i<m_dof; i++){
        value[i] = m_value.at(i, 0);
    }
}

void Joints::getValue(vector<double>& value) const{
    value.clear();
    for(int i=0; i<m_dof; i++){
        value.push_back(m_value.at(i, 0));
    }
}

void Joints::getValue(CMatrix<double>& value) const{
    value = m_value;
}

double Joints::norm(const Joints &joint) const{
    Joints temp = *this - joint;
    return temp.m_value.norm();
}

double& Joints::operator [](JOINTINDEX index){
    return m_value.at(index, 0);
}

const double& Joints::operator [](JOINTINDEX index) const{
    return m_value.at(index, 0);
}

Joints Joints::operator +(const Joints& joint) const{
    CMatrix<double> res;
    res = m_value + joint.m_value;
    return Joints(m_dof, res);
}

Joints Joints::operator -(const Joints& joint) const{
    CMatrix<double> res;
    res = m_value - joint.m_value;
    return Joints(m_dof, res);
}

Joints Joints::operator *(double value) const{
    CMatrix<double> res;
    res = m_value * value;
    return Joints(m_dof, res);
}

Joints Joints::operator /(double value) const{
    CMatrix<double> res;
    res = m_value * (1/value);
    return Joints(m_dof, res);
}

bool Joints::operator ==(const Joints& joint) const{
    if(getJointsDOF() != joint.getJointsDOF()){
        return false;
    }
    for(int i=0; i<getJointsDOF(); i++){
        double diff = getValue(JOINTINDEX(i))-joint.getValue(JOINTINDEX(i));
        if(!num_is_zero(diff)){
            return false;
        }
    }
    return true;
}

bool Joints::operator !=(const Joints& joint) const{
    return !(*this == joint);
}

bool Joints::judgeOverMinimum(const Joints &joint) const{
    bool state = false;
    for(int i=0; i<m_dof; i++){
        if(this->getValue(JOINTINDEX(i))<joint.getValue(JOINTINDEX(i))){
            state = true;
            break;
        }
    }
    return state;
}

bool Joints::judgeOverMaximum(const Joints &joint) const{
    bool state = false;
    for(int i=0; i<m_dof; i++){
        if(this->getValue(JOINTINDEX(i))>joint.getValue(JOINTINDEX(i))){
            state = true;
            break;
        }
    }
    return state;
}

void Joints::print(const char* str) const{
    m_value.print(str);
}

// class JointsList
JointsList::JointsList(){

}

JointsList::JointsList(const std::vector<Joints> &js){
    m_vector = js;
}

JointsList::~JointsList(){

}

void JointsList::clear(){
    m_vector.clear();
}

void JointsList::push_back(const Joints& element){
    m_vector.push_back(element);
}

void JointsList::push_front(const Joints &element){
    m_vector.insert(m_vector.begin(), element);
}

void JointsList::pop_back(){
    m_vector.pop_back();
}

void JointsList::erase(int index){
    m_vector.erase(m_vector.begin()+index);
}

Joints& JointsList::operator [](int index){
    return m_vector[index];
}

const Joints& JointsList::operator [](int index) const{
    return m_vector[index];
}

bool JointsList::empty() const{
    return m_vector.empty();
}

int JointsList::size() const{
    return m_vector.size();
}

CMatrix<double> JointsList::getCertainJointList(JOINTINDEX index) const{
    if(this->size() == 0){
        throw string("Error: get_certain_joint_list::joint list size is zero");
    }

    if(index >= (*this)[0].getJointsDOF()){
        throw string("Error: get_certain_joint_list::joint index is larger than dof");
    }

    CMatrix<double> matrix(this->size(), 1);
    for(int i=0; i<this->size(); i++){
        matrix.at(i, 0) = (*this)[i].getValue(index);
    }
    return matrix;
}


// class JointsMotionState
JointsMotionState::JointsMotionState(){

}

JointsMotionState::JointsMotionState(int dof){
    this->setValue(dof);
}

JointsMotionState::~JointsMotionState(){

}

void JointsMotionState::setValue(int dof){
    m_pos.setValue(dof);
    m_vel.setValue(dof);
    m_acc.setValue(dof);
}

void JointsMotionState::setPosValue(const Joints &pos){
    m_pos = pos;
}

void JointsMotionState::setVelValue(const Joints &vel){
    m_vel = vel;
}

void JointsMotionState::setAccValue(const Joints &acc){
    m_acc =acc;
}

int JointsMotionState::getJointsDOF() const{
    return m_pos.getJointsDOF();
}

Joints JointsMotionState::getPosValue() const{
    return m_pos;
}

Joints JointsMotionState::getVelValue() const{
    return m_vel;
}

Joints JointsMotionState::getAccValue() const{
    return m_acc;
}

void JointsMotionState::append(const JointsMotionState &jMotion){
    m_pos.append(jMotion.m_pos);
    m_vel.append(jMotion.m_vel);
    m_acc.append(jMotion.m_acc);
}

void JointsMotionState::calVelAcc(const JointsMotionState &jMotion, double time){
    m_vel = (m_pos-jMotion.m_pos) * (1/time);
    m_acc = (m_vel-jMotion.m_vel) * (1/time);
}

// class JointsMotionStateList
JointsMotionStateList::JointsMotionStateList(){

}

JointsMotionStateList::~JointsMotionStateList(){

}

void JointsMotionStateList::clear(){
    m_vector.clear();
}

void JointsMotionStateList::push_back(const JointsMotionState& element){
    m_vector.push_back(element);
}

void JointsMotionStateList::pop_back(){
    m_vector.pop_back();
}

void JointsMotionStateList::pop_front(){
    m_vector.erase(m_vector.begin());
}

void JointsMotionStateList::erase(int index){
    m_vector.erase(m_vector.begin()+index);
}

void JointsMotionStateList::append(const JointsMotionStateList &jMotionList){
    for(int i=0; i<jMotionList.size(); i++){
        this->push_back(jMotionList[i]);
    }
}

JointsMotionState JointsMotionStateList::front() const{
    return m_vector.front();
}

JointsMotionState JointsMotionStateList::back() const{
    return m_vector.back();
}

JointsMotionState& JointsMotionStateList::operator [](int index){
    return m_vector[index];
}

const JointsMotionState& JointsMotionStateList::operator [](int index) const{
    return m_vector[index];
}

bool JointsMotionStateList::empty() const{
    return m_vector.empty();
}

int JointsMotionStateList::size() const{
    return m_vector.size();
}

void JointsMotionStateList::print() const{
    cout << "JointsMotionStateList Position: " << endl;
    for(int i=0; i<this->size(); i++){
        (*this)[i].getPosValue().getValue().trans().print("");
    }
    cout << "JointsMotionStateList Velocity: " << endl;
    for(int i=0; i<this->size(); i++){
        (*this)[i].getVelValue().getValue().trans().print("");
    }
    cout << "JointsMotionStateList Acceleration: " << endl;
    for(int i=0; i<this->size(); i++){
        (*this)[i].getAccValue().getValue().trans().print("");
    }
}

// class Point
Point::Point(){
    this->setValue(0, 0, 1);
}

Point::~Point(){

}

Point::Point(double valueX, double valueY, double valueZ){
    this->setValue(valueX, valueY, valueZ);
}

Point::Point(const CMatrix<double>& value){
    this->setValue(value);
}

void Point::setValue(double valueX, double valueY, double valueZ){
    m_value.zeros(3, 1);
    m_value.at(0, 0) = valueX;
    m_value.at(1, 0) = valueY;
    m_value.at(2, 0) = valueZ;
}

void Point::setValue(const CMatrix<double>& value){
    m_value = value;
}

double Point::getValue(POINTINDEX index) const{
    return m_value.at(index, 0);
}

CMatrix<double> Point::getValue() const{
    return m_value;
}

void Point::getValue(double &valueX, double &valueY, double &valueZ) const{
    valueX = m_value.at(0, 0);
    valueY = m_value.at(1, 0);
    valueZ = m_value.at(2, 0);
}

void Point::getValue(CMatrix<double> &value) const{
    value = m_value;
}

double& Point::operator [](POINTINDEX index){
    return m_value.at(index, 0);
}

const double& Point::operator [](POINTINDEX index) const{
    return m_value.at(index, 0);
}

Point Point::operator +(const Point& point) const{
    return Point(m_value + point.m_value);
}

Point Point::operator -(const Point& point) const{
    return Point(m_value - point.m_value);
}

Point Point::operator *(double value) const{
    return Point(m_value * value);
}

double Point::dot(const Point& point) const{
    return m_value.dot(point.m_value);
}
Point Point::cross(const Point& point) const{
    return Point(m_value.cross(point.m_value));
}

double Point::norm() const{
    return m_value.norm();
}

Point Point::normalization() const{
    return Point(m_value.normalization());
}

void Point::print(const char* str) const{
    m_value.print(str);
}


// class UnitVector
UnitVector::UnitVector(){
    this->setValue(0, 0, 1);
}

UnitVector::UnitVector(double valueX, double valueY, double valueZ){
    this->setValue(valueX, valueY, valueZ);
}

UnitVector::UnitVector(const Point &point){
    this->setValue(point);
}

UnitVector::UnitVector(const CMatrix<double> &value){
    this->setValue(value);
}

UnitVector::~UnitVector(){

}

void UnitVector::setValue(double valueX, double valueY, double valueZ){
    m_value.zeros(3, 1);
    m_value.at(0, 0) = valueX;
    m_value.at(1, 0) = valueY;
    m_value.at(2, 0) = valueZ;

    m_value = m_value.normalization();
}

void UnitVector::setValue(const Point &point){
    CMatrix<double> value;
    point.getValue(value);
    this->setValue(value);
}

void UnitVector::setValue(const CMatrix<double> &value){
    m_value = value.normalization();
}

void UnitVector::operator =(const Point& point){
    this->setValue(point);
}

void UnitVector::print(const char* str) const{
    m_value.print(str);
}



// class AttitudeAngle

AttitudeAngle::AttitudeAngle(){
    this->setValue(0, 0, 0);
}

AttitudeAngle::AttitudeAngle(double valueA, double valueB, double valueC){
    this->setValue(valueA, valueB, valueC);
}

AttitudeAngle::AttitudeAngle(const CMatrix<double> &value){
    this->setValue(value);
}

AttitudeAngle::~AttitudeAngle(){

}

void AttitudeAngle::setValue(double valueA, double valueB, double valueC){
    m_value.zeros(3, 1);
    m_value.at(0, 0) = valueA;
    m_value.at(1, 0) = valueB;
    m_value.at(2, 0) = valueC;
}

void AttitudeAngle::setValue(const CMatrix<double> &value){
    m_value = value;
}

double AttitudeAngle::getValue(ATTITUDEINDEX index) const{
    return m_value.at(index, 0);
}

CMatrix<double> AttitudeAngle::getValue() const{
    return m_value;
}

void AttitudeAngle::getValue(double &valueA, double &valueB, double &valueC) const{
    if(valueA < -180 || valueA > 180){
        cout << "angle A belongs to [-180, 180]" << endl;
        exit(0);
    }

    if(valueB < -180/2 || valueB > 180/2){
        cout << "angle B belongs to [-180/2, 180/2]" << endl;
        exit(0);
    }

    if(valueC < -180 || valueC > 180){
        cout << "angle C belongs to [-180, 180]" << endl;
        exit(0);
    }

    valueA = m_value.at(0, 0);
    valueB = m_value.at(1, 0);
    valueC = m_value.at(2, 0);
}

void AttitudeAngle::getValue(CMatrix<double> &value) const{
    value = m_value;
}

RotateMatrix AttitudeAngle::getRotateMatrix() const{
    double alpha = m_value.at(0, 0);
    double beta = m_value.at(1, 0);
    double gamma = m_value.at(2, 0);

    double r11 = cos_deg(alpha)*cos_deg(beta);
    double r21 = sin_deg(alpha)*cos_deg(beta);
    double r31 = -sin_deg(beta);
    double r12 = cos_deg(alpha)*sin_deg(beta)*sin_deg(gamma)-sin_deg(alpha)*cos_deg(gamma);
    double r22 = sin_deg(alpha)*sin_deg(beta)*sin_deg(gamma)+cos_deg(alpha)*cos_deg(gamma);
    double r32 = cos_deg(beta)*sin_deg(gamma);
    double r13 = cos_deg(alpha)*sin_deg(beta)*cos_deg(gamma)+sin_deg(alpha)*sin_deg(gamma);
    double r23 = sin_deg(alpha)*sin_deg(beta)*cos_deg(gamma)-cos_deg(alpha)*sin_deg(gamma);
    double r33 = cos_deg(beta)*cos_deg(gamma);

    double value[] = {r11, r12, r13, r21, r22, r23, r31, r32, r33};

    return RotateMatrix(CMatrix<double>(3, 3, value));
}

DualVector AttitudeAngle::getDualVector() const{
    return this->getRotateMatrix().getDualVector();
}

Quaternion AttitudeAngle::getQuaternion() const{
    return this->getRotateMatrix().getQuaternion();
}

double& AttitudeAngle::operator [](ATTITUDEINDEX index){
    return m_value.at(index, 0);
}

const double& AttitudeAngle::operator [](ATTITUDEINDEX index) const{
    return m_value.at(index, 0);
}

AttitudeAngle AttitudeAngle::operator +(const AttitudeAngle& angle) const{
    return AttitudeAngle(m_value + angle.m_value);
}

AttitudeAngle AttitudeAngle::operator -(const AttitudeAngle& angle) const{
    return AttitudeAngle(m_value - angle.m_value);
}

AttitudeAngle AttitudeAngle::operator *(double value) const{
    return AttitudeAngle(m_value * value);
}

void AttitudeAngle::print(const char* str) const{
    m_value.print(str);
}



// class RotateMatrix

RotateMatrix::RotateMatrix(){
    CMatrix<double> value;
    value.eye(3, 3);
    this->setValue(value);
}

RotateMatrix::RotateMatrix(const CMatrix<double> &value){
    this->setValue(value);
}

RotateMatrix::~RotateMatrix(){

}

void RotateMatrix::setValue(const CMatrix<double> &value){
    m_value = value;
}

CMatrix<double> RotateMatrix::getValue() const{
    return m_value;
}

AttitudeAngle RotateMatrix::getAttitudeAngle() const{
    double r11 = m_value.at(0, 0);
    double r21 = m_value.at(1, 0);
    double r31 = m_value.at(2, 0);
    double r12 = m_value.at(0, 1);
    double r22 = m_value.at(1, 1);
    double r32 = m_value.at(2, 1);
    double r13 = m_value.at(0, 2);
    double r23 = m_value.at(1, 2);
    double r33 = m_value.at(2, 2);

    double alpha, beta, gamma;

    double cosbeta = sqrt(r11*r11+r21*r21);
    if(num_is_zero(cosbeta)){
        alpha = 0;
        if(r31 > 0){
            beta = -180/2;
            gamma = -atan2_deg(r12, r22);
        }
        else{
            beta = 180/2;
            gamma = atan2_deg(r12, r22);
        }
    }
    else{
        beta = atan2_deg(-r31, cosbeta);
        alpha = atan2_deg(r21/cosbeta, r11/cosbeta);
        gamma = atan2_deg(r32/cosbeta, r33/cosbeta);
    }

    return AttitudeAngle(alpha, beta, gamma);
}

DualVector RotateMatrix::getDualVector() const{
    double r11 = m_value.at(0, 0);
    double r21 = m_value.at(1, 0);
    double r31 = m_value.at(2, 0);
    double r13 = m_value.at(0, 2);
    double r23 = m_value.at(1, 2);
    double r33 = m_value.at(2, 2);
    UnitVector dirVector(r13, r23, r33);
    UnitVector normVector(r11, r21, r31);

    return DualVector(dirVector, normVector);
}

Quaternion RotateMatrix::getQuaternion() const{
    double r11 = m_value.at(0, 0);
    double r21 = m_value.at(1, 0);
    double r31 = m_value.at(2, 0);
    double r12 = m_value.at(0, 1);
    double r22 = m_value.at(1, 1);
    double r32 = m_value.at(2, 1);
    double r13 = m_value.at(0, 2);
    double r23 = m_value.at(1, 2);
    double r33 = m_value.at(2, 2);

    double q0 = (1 + r11 + r22 + r33)/4;
    double q1 = (1 + r11 - r22 - r33)/4;
    double q2 = (1 - r11 + r22 - r33)/4;
    double q3 = (1 - r11 - r22 + r33)/4;
    if(q0 >= (q1-EPSLON) && q0 >= (q2-EPSLON) && q0 >= (q3-EPSLON)){
        q0 = sqrt(q0);
        q1 = (r32 - r23)/(4*q0);
        q2 = (r13 - r31)/(4*q0);
        q3 = (r21 - r12)/(4*q0);
    }
    else if(q1 >= (q0-EPSLON) && q1 >= (q2-EPSLON) && q1 >= (q3-EPSLON)){
        q1 = sqrt(q1);
        q0 = (r32 - r23)/(4*q1);
        q2 = (r21 + r12)/(4*q1);
        q3 = (r13 + r31)/(4*q1);
    }
    else if(q2 >= (q0-EPSLON) && q2 >= (q1-EPSLON) && q2 >= (q3-EPSLON)){
        q2 = sqrt(q2);
        q0 = (r13 - r31)/(4*q2);
        q1 = (r21 + r12)/(4*q2);
        q3 = (r32 + r23)/(4*q2);
    }
    else if(q3 >= (q0-EPSLON) && q3 >= (q1-EPSLON) && q3 >= (q2-EPSLON)){
        q3 = sqrt(q3);
        q0 = (r21 - r12)/(4*q3);
        q1 = (r13 + r31)/(4*q3);
        q2 = (r32 + r23)/(4*q3);
    }

    return Quaternion(q0, q1, q2, q3);
}

RotateAxis RotateMatrix::getRotateAxis() const{
    double r11 = m_value.at(0, 0);
    double r21 = m_value.at(1, 0);
    double r31 = m_value.at(2, 0);
    double r12 = m_value.at(0, 1);
    double r22 = m_value.at(1, 1);
    double r32 = m_value.at(2, 1);
    double r13 = m_value.at(0, 2);
    double r23 = m_value.at(1, 2);
    double r33 = m_value.at(2, 2);

    double theta = acos_deg((r11+r22+r33-1)/2);
    UnitVector uv((r32-r23)/(2*sin_deg(theta)), (r13-r31)/(2*sin_deg(theta)), (r21-r12)/(2*sin_deg(theta)));

    return RotateAxis(uv, theta);
}

RotateMatrix RotateMatrix::operator *(double value) const{
    return RotateMatrix(m_value * value);
}

RotateMatrix RotateMatrix::operator +(const RotateMatrix& rotate) const{
    return RotateMatrix(m_value + rotate.m_value);
}

RotateMatrix RotateMatrix::operator -(const RotateMatrix& rotate) const{
    return RotateMatrix(m_value - rotate.m_value);
}

RotateMatrix RotateMatrix::operator *(const RotateMatrix& rotate) const{
    return RotateMatrix(m_value * rotate.m_value);
}

Point RotateMatrix::operator *(const Point& point) const{
    CMatrix<double> value;
    value = m_value * point.getValue();
    return Point(value);
}

RotateMatrix RotateMatrix::trans() const{
    return RotateMatrix(m_value.trans());
}

RotateMatrix RotateMatrix::inv() const{
    return RotateMatrix(m_value.inv());
}

void RotateMatrix::print(const char* str) const{
    m_value.print(str);
}



// class DualVector

DualVector::DualVector(){
    UnitVector dirVector(0, 0, 1);
    UnitVector normVector(1, 0, 0);
    this->setValue(dirVector, normVector);
}

DualVector::DualVector(const UnitVector &dirVector, const UnitVector &normVector){
    this->setValue(dirVector, normVector);
}

DualVector::~DualVector(){

}

void DualVector::setValue(const UnitVector &dirVector, const UnitVector &normVector){
    m_dirVector = dirVector;
    m_normVector = normVector;
}

void DualVector::setDirVector(const UnitVector &dirVector){ // 仅指定方向向量时，保持alpha角为０
    m_dirVector = dirVector;
    double x = m_dirVector.getValue(POINT_X);
    double z = m_dirVector.getValue(POINT_Z);
    double beta = atan2(x, z);
    m_normVector = UnitVector(cos(beta), 0, -sin(beta));
}

void DualVector::setNormVector(const UnitVector &normVector){
    m_normVector = normVector;
}

UnitVector DualVector::getDirVector() const{
    return m_dirVector;
}

UnitVector DualVector::getNormVector() const{
    return m_normVector;
}

RotateMatrix DualVector::getRotateMatrix() const{
    UnitVector tempVector = m_dirVector.cross(m_normVector);

    CMatrix<double> value(3, 3);
    value.at(0, 0) = m_normVector.getValue(POINT_X);
    value.at(1, 0) = m_normVector.getValue(POINT_Y);
    value.at(2, 0) = m_normVector.getValue(POINT_Z);
    value.at(0, 1) = tempVector.getValue(POINT_X);
    value.at(1, 1) = tempVector.getValue(POINT_Y);
    value.at(2, 1) = tempVector.getValue(POINT_Z);
    value.at(0, 2) = m_dirVector.getValue(POINT_X);
    value.at(1, 2) = m_dirVector.getValue(POINT_Y);
    value.at(2, 2) = m_dirVector.getValue(POINT_Z);

    return RotateMatrix(value);
}

AttitudeAngle DualVector::getAttitudeAngle() const{
    return this->getRotateMatrix().getAttitudeAngle();
}

Quaternion DualVector::getQuaternion() const{
    return this->getRotateMatrix().getQuaternion();
}

void DualVector::print(const char* str) const{
    cout << str << ": " << endl;
    m_dirVector.print("Dir Vector");
    m_normVector.print("Norm Vector");
}



// class Quaternion

Quaternion::Quaternion(){
    this->setValue(1, 0, 0, 0);
}

Quaternion::Quaternion(const CMatrix<double> &value){
    this->setValue(value);
}

Quaternion::Quaternion(double valueW, double valueX, double valueY, double valueZ){
    this->setValue(valueW, valueX, valueY, valueZ);
}

Quaternion::~Quaternion(){

}

void Quaternion::setValue(const CMatrix<double> &value){
    m_value = value;
}

void Quaternion::setValue(double valueW, double valueX, double valueY, double valueZ){
    m_value.zeros(4, 1);

    m_value.at(0, 0) = valueW;
    m_value.at(1, 0) = valueX;
    m_value.at(2, 0) = valueY;
    m_value.at(3, 0) = valueZ;
}

double Quaternion::getValue(QUATERNIONINDEX index) const{
    return m_value.at(index, 0);
}

RotateMatrix Quaternion::getRotateMatrix() const{
    double q0 = m_value.at(0, 0);
    double q1 = m_value.at(1, 0);
    double q2 = m_value.at(2, 0);
    double q3 = m_value.at(3, 0);

    double r11 = 1-2*q2*q2-2*q3*q3;
    double r21 = 2*q1*q2+2*q0*q3;
    double r31 = 2*q1*q3-2*q0*q2;
    double r12 = 2*q1*q2-2*q0*q3;
    double r22 = 1-2*q1*q1-2*q3*q3;
    double r32 = 2*q2*q3+2*q0*q1;
    double r13 = 2*q1*q3+2*q0*q2;
    double r23 = 2*q2*q3-2*q0*q1;
    double r33 = 1-2*q1*q1-2*q2*q2;

    double value[] = {r11, r12, r13, r21, r22, r23, r31, r32, r33};

    return RotateMatrix(CMatrix<double>(3, 3, value));
}

AttitudeAngle Quaternion::getAttitudeAngle() const{
    return this->getRotateMatrix().getAttitudeAngle();
}

DualVector Quaternion::getDualVector() const{
    return this->getRotateMatrix().getDualVector();
}

Quaternion Quaternion::operator *(double value) const{
    return Quaternion(m_value * value);
}

double Quaternion::operator *(const Quaternion &q) const{
    return m_value.dot(q.m_value);
}

Quaternion Quaternion::operator +(const Quaternion &q) const{
    return Quaternion(m_value + q.m_value);
}

double Quaternion::dot(const Quaternion &q) const{
    return this->getValue(QUATERNION_W)*q.getValue(QUATERNION_W) +
            this->getValue(QUATERNION_X)*q.getValue(QUATERNION_X) +
            this->getValue(QUATERNION_Y)*q.getValue(QUATERNION_Y) +
            this->getValue(QUATERNION_Z)*q.getValue(QUATERNION_Z);
}

Quaternion Quaternion::cross(const Quaternion &q) const{
    double w = this->getValue(QUATERNION_W)*q.getValue(QUATERNION_W)-
            this->getValue(QUATERNION_X)*q.getValue(QUATERNION_X)-
            this->getValue(QUATERNION_Y)*q.getValue(QUATERNION_Y)-
            this->getValue(QUATERNION_Z)*q.getValue(QUATERNION_Z);
    double x = this->getValue(QUATERNION_W)*q.getValue(QUATERNION_X)+
            this->getValue(QUATERNION_X)*q.getValue(QUATERNION_W)+
            this->getValue(QUATERNION_Z)*q.getValue(QUATERNION_Y)-
            this->getValue(QUATERNION_Y)*q.getValue(QUATERNION_Z);
    double y = this->getValue(QUATERNION_W)*q.getValue(QUATERNION_Y)+
            this->getValue(QUATERNION_Y)*q.getValue(QUATERNION_W)+
            this->getValue(QUATERNION_X)*q.getValue(QUATERNION_Z)-
            this->getValue(QUATERNION_Z)*q.getValue(QUATERNION_X);
    double z = this->getValue(QUATERNION_W)*q.getValue(QUATERNION_Z)+
            this->getValue(QUATERNION_Z)*q.getValue(QUATERNION_W)+
            this->getValue(QUATERNION_Y)*q.getValue(QUATERNION_X)-
            this->getValue(QUATERNION_X)*q.getValue(QUATERNION_Y);
    return Quaternion(w, x, y, z);
}

double Quaternion::norm() const{
    return m_value.norm();
}

void Quaternion::print(const char *str) const{
    m_value.print(str);
}



// class RotateAxis

RotateAxis::RotateAxis(){
    UnitVector uv(0, 0, 1);
    this->setValue(uv, 0);
}

RotateAxis::RotateAxis(const UnitVector &axis, double angle){
    this->setValue(axis, angle);
}

RotateAxis::~RotateAxis(){

}

void RotateAxis::setValue(const UnitVector &axis, double angle){
    m_axis = axis;
    m_angle = angle;
}

UnitVector RotateAxis::getRotateAxis() const{
    return m_axis;
}

double RotateAxis::getRotateAngle() const{
    return m_angle;
}

void RotateAxis::print(const char *str) const{
    cout << str << ": " << endl;
    cout << "Angle: " << m_angle << endl;
    m_axis.print("Axis");
}

RotateMatrix RotateAxis::getRotateMatrix() const{
    double ct = cos_deg(m_angle);
    double st = sin_deg(m_angle);
    double vt = 1-ct;

    double kx = m_axis.getValue(POINT_X);
    double ky = m_axis.getValue(POINT_Y);
    double kz = m_axis.getValue(POINT_Z);

    double r11 = kx*kx*vt+ct;
    double r12 = kx*ky*vt-kz*st;
    double r13 = kx*kz*vt+ky*st;
    double r21 = kx*ky*vt+kz*st;
    double r22 = ky*ky*vt+ct;
    double r23 = ky*kz*vt-kx*st;
    double r31 = kx*kz*vt-ky*st;
    double r32 = ky*kz*vt+kx*st;
    double r33 = kz*kz*vt+ct;

    double value[] = {r11, r12, r13, r21, r22, r23, r31, r32, r33};

    return RotateMatrix(CMatrix<double>(3, 3, value));
}

// class Terminal

Terminal::Terminal(){
    this->setValue(0, 0, 0, 0, 0, 0);
}

Terminal::Terminal(const CMatrix<double> &value){
    this->setValue(value);
}

Terminal::Terminal(double valueX, double valueY, double valueZ, double valueA, double valueB, double valueC){
    this->setValue(valueX, valueY, valueZ, valueA, valueB, valueC);
}

Terminal::Terminal(const Point &point, const AttitudeAngle &attitude){
    this->setValue(point, attitude);
}

Terminal::Terminal(const Point &point, const RotateMatrix &rotate){
    this->setValue(point, rotate);
}

Terminal::Terminal(const HomogeneousMatrix &homo){
    this->setValue(homo);
}

Terminal::~Terminal(){

}

void Terminal::setValue(const CMatrix<double> &value){
    m_point.setValue(value.at(0, 0), value.at(1, 0), value.at(2, 0));
    m_attitude.setValue(value.at(3, 0), value.at(4, 0), value.at(5, 0));
}

void Terminal::setValue(double valueX, double valueY, double valueZ, double valueA, double valueB, double valueC){
    m_point.setValue(valueX, valueY, valueZ);
    m_attitude.setValue(valueA, valueB, valueC);
}

void Terminal::setValue(const Point &point, const AttitudeAngle &attitude){
    m_point = point;
    m_attitude = attitude;
}

void Terminal::setValue(const Point &point, const RotateMatrix &rotate){
    m_point = point;
    m_attitude = rotate.getAttitudeAngle();
}

void Terminal::setValue(const HomogeneousMatrix& homo){
    m_point = homo.getPoint();
    m_attitude = homo.getAttitudeAngle();
}

Point Terminal::getPoint() const{
    return m_point;
}

AttitudeAngle Terminal::getAttitudeAngle() const{
    return m_attitude;
}

RotateMatrix Terminal::getRotateMatrix() const{
    return m_attitude.getRotateMatrix();
}

HomogeneousMatrix Terminal::getHomogeneousMatrix() const{
    return HomogeneousMatrix(m_point, m_attitude);
}

double Terminal::getValue(TERMINALINDEX index) const{
    if(index < 3){
        return m_point.getValue(POINTINDEX(index));
    }
    else{
        return m_attitude.getValue(ATTITUDEINDEX(index-3));
    }
}

Terminal Terminal::getTerminalInWorkFrame(Terminal workframe) const{
    HomogeneousMatrix homoWorkFrame = workframe.getHomogeneousMatrix();
    HomogeneousMatrix homo = this->getHomogeneousMatrix();
    return homo.getHomogeneousMatrixInWorkFrame(homoWorkFrame).getTerminal();
}

Terminal Terminal::getTerminalFromWorkFrame(Terminal workframe) const{
    HomogeneousMatrix homoWorkFrame = workframe.getHomogeneousMatrix();
    HomogeneousMatrix homo = this->getHomogeneousMatrix();
    return homo.getHomogeneousMatrixFromWorkFrame(homoWorkFrame).getTerminal();
}

double& Terminal::operator [](TERMINALINDEX index){
    if(index < TERMINAL_A){
        return m_point[POINTINDEX(index)];
    }
    else{
        return m_attitude[ATTITUDEINDEX(index-TERMINAL_A)];
    }
}

const double& Terminal::operator [](TERMINALINDEX index) const{
    if(index < TERMINAL_A){
        return m_point[POINTINDEX(index)];
    }
    else{
        return m_attitude[ATTITUDEINDEX(index-TERMINAL_A)];
    }
}

Terminal Terminal::operator +(const Terminal& terminal) const{
    return Terminal(m_point+terminal.m_point, m_attitude+terminal.m_attitude);
}

Terminal Terminal::operator -(const Terminal& terminal) const{
    return Terminal(m_point-terminal.m_point, m_attitude-terminal.m_attitude);
}

Terminal Terminal::operator *(const Terminal& terminal) const{
    return (this->getHomogeneousMatrix() * terminal.getHomogeneousMatrix()).getTerminal();
}

Terminal Terminal::operator *(double value) const{
    return Terminal(m_point * value, m_attitude * value);
}

bool Terminal::operator ==(const Terminal& terminal) const{
    for(int i=0; i<6; i++){
        double diff = getValue(TERMINALINDEX(i))-terminal.getValue(TERMINALINDEX(i));
        if(!num_is_zero(diff)){
            return false;
        }
    }
    return true;
}

bool Terminal::operator !=(const Terminal& terminal) const{
    return !(*this == terminal);
}

bool Terminal::judgeOverMinimum(const Terminal &terminal) const{
    bool state = false;
    for(int i=0; i<6; i++){
        if(this->getValue(TERMINALINDEX(i))<terminal.getValue(TERMINALINDEX(i))){
            state = true;
            break;
        }
    }
    return state;
}

bool Terminal::judgeOverMaximum(const Terminal &terminal) const{
    bool state = false;
    for(int i=0; i<6; i++){
        if(this->getValue(TERMINALINDEX(i))<terminal.getValue(TERMINALINDEX(i))){
            state = true;
            break;
        }
    }
    return state;
}

void Terminal::print(const char *str) const{
    cout << str << ": " << endl;
    m_point.print("Terminal Point");
    m_attitude.print("Terminal Attitude");
}

// class TerminalList
TerminalList::TerminalList(){

}

TerminalList::TerminalList(const std::vector<Terminal> &ts){
    m_vector = ts;
}

TerminalList::~TerminalList(){

}

void TerminalList::clear(){
    m_vector.clear();
}

void TerminalList::push_back(const Terminal& element){
    m_vector.push_back(element);
}

void TerminalList::push_front(const Terminal &element){
    m_vector.insert(m_vector.begin(), element);
}

void TerminalList::pop_back(){
    m_vector.pop_back();
}

void TerminalList::erase(int index){
    m_vector.erase(m_vector.begin()+index);
}

Terminal& TerminalList::operator [](int index){
    return m_vector[index];
}

const Terminal& TerminalList::operator [](int index) const{
    return m_vector[index];
}

bool TerminalList::empty() const{
    return m_vector.empty();
}

int TerminalList::size() const{
    return m_vector.size();
}

// class HomogeneousMatrix

HomogeneousMatrix::HomogeneousMatrix(){
    CMatrix<double> value;
    value.eye(4, 4);
    this->setValue(value);
}

HomogeneousMatrix::HomogeneousMatrix(const CMatrix<double> &value){
    this->setValue(value);
}

HomogeneousMatrix::HomogeneousMatrix(const Point &point, const AttitudeAngle &attitude){
    this->setValue(point, attitude);
}

HomogeneousMatrix::HomogeneousMatrix(const Point &point, const RotateMatrix &rotate){
    this->setValue(point, rotate);
}

HomogeneousMatrix::~HomogeneousMatrix(){

}

void HomogeneousMatrix::setValue(const CMatrix<double> &value){
    m_value = value;
}

void HomogeneousMatrix::setValue(const Point &point, const AttitudeAngle &attitude){
    this->setValue(point, attitude.getRotateMatrix());
}

void HomogeneousMatrix::setValue(const Point &point, const RotateMatrix &rotate){
    m_value.eye(4, 4);
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            m_value.at(i, j) = rotate.getValue().at(i, j);
        }
    }
    m_value.at(0, 3) = point.getValue(POINT_X);
    m_value.at(1, 3) = point.getValue(POINT_Y);
    m_value.at(2, 3) = point.getValue(POINT_Z);
}

CMatrix<double> HomogeneousMatrix::getValue() const{
    return m_value;
}

double HomogeneousMatrix::getValue(int i, int j) const{
    return m_value.at(i, j);
}

Point HomogeneousMatrix::getPoint() const{
    double r14 = m_value.at(0, 3);
    double r24 = m_value.at(1, 3);
    double r34 = m_value.at(2, 3);

    return Point(r14, r24, r34);
}

AttitudeAngle HomogeneousMatrix::getAttitudeAngle() const{
    double r11 = m_value.at(0, 0);
    double r21 = m_value.at(1, 0);
    double r31 = m_value.at(2, 0);
    double r12 = m_value.at(0, 1);
    double r22 = m_value.at(1, 1);
    double r32 = m_value.at(2, 1);
    double r13 = m_value.at(0, 2);
    double r23 = m_value.at(1, 2);
    double r33 = m_value.at(2, 2);

    double value[] = {r11, r12, r13, r21, r22, r23, r31, r32, r33};

    RotateMatrix rotate(CMatrix<double>(3, 3, value));

    return rotate.getAttitudeAngle();
}

RotateMatrix HomogeneousMatrix::getRotateMatrix() const{
    double r11 = m_value.at(0, 0);
    double r21 = m_value.at(1, 0);
    double r31 = m_value.at(2, 0);
    double r12 = m_value.at(0, 1);
    double r22 = m_value.at(1, 1);
    double r32 = m_value.at(2, 1);
    double r13 = m_value.at(0, 2);
    double r23 = m_value.at(1, 2);
    double r33 = m_value.at(2, 2);

    double value[] = {r11, r12, r13, r21, r22, r23, r31, r32, r33};

    return RotateMatrix(CMatrix<double>(3, 3, value));
}

Terminal HomogeneousMatrix::getTerminal() const{
    Point point = this->getPoint();
    AttitudeAngle angle = this->getAttitudeAngle();

    return Terminal(point, angle);
}

HomogeneousMatrix HomogeneousMatrix::operator *(const HomogeneousMatrix& homo) const{
    return HomogeneousMatrix(m_value * homo.m_value);
}

HomogeneousMatrix HomogeneousMatrix::inv() const{
    RotateMatrix rotate = this->getRotateMatrix();
    Point point = this->getPoint();

    Point zeropoint(0, 0, 0);
    return HomogeneousMatrix(zeropoint-rotate.inv()*point, rotate.inv());
}

HomogeneousMatrix HomogeneousMatrix::getHomogeneousMatrixInWorkFrame(HomogeneousMatrix workframe) const{
    return workframe.inv() * (*this);
}

HomogeneousMatrix HomogeneousMatrix::getHomogeneousMatrixFromWorkFrame(HomogeneousMatrix workframe) const{
    return workframe * (*this);
}

void HomogeneousMatrix::print(const char *str) const{
    m_value.print(str);
}


RobotMotion::RobotMotion(){

}

RobotMotion::~RobotMotion(){

}

void RobotMotion::setCurrentJointPosition(const Joints& joint){
    m_currentJointPosition = joint;
}

Joints RobotMotion::getCurrentJointPosition() const{
    return m_currentJointPosition;
}

void RobotMotion::setCurrentJointVelocity(const Joints& joint){
    m_currentJointVelocity = joint;
}

Joints RobotMotion::getCurrentJointVelocity() const{
    return m_currentJointVelocity;
}

void RobotMotion::setCurrentJointAcceleration(const Joints &joint){
    m_currentJointAcceleration = joint;
}

Joints RobotMotion::getCurrentJointAcceleration() const{
    return m_currentJointAcceleration;
}

void RobotMotion::setCurrentJointTorque(const Joints& joint){
    m_currentJointTorque = joint;
}

Joints RobotMotion::getCurrentJointsTorque() const{
    return m_currentJointTorque;
}

void RobotMotion::setCurrentTerminal(const Terminal& terminal){
    m_currentTerminal = terminal;
}

Terminal RobotMotion::getCurrentTerminal() const{
    return m_currentTerminal;
}

void RobotMotion::setCurrentWorkTerminal(const Terminal& terminal){
    m_currentWorkTerminal = terminal;
}

Terminal RobotMotion::getCurrentWorkTerminal() const{
    return m_currentWorkTerminal;
}

#endif
