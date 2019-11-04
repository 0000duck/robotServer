#ifndef CPATHPLANNING_CPP
#define CPATHPLANNING_CPP

#include "CPathPlanning.hpp"
#include "CMathBasic.hpp"

#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace robsoft;

Line::Line(){

}

Line::Line(const Point &startPoint, const Point &endPoint){
    this->setPoints(startPoint, endPoint);
}

Line::~Line(){

}

void Line::setPoints(const Point &startPoint, const Point &endPoint){
    m_startPoint = startPoint;
    m_endPoint = endPoint;
    m_length = (m_endPoint - m_startPoint).norm();

    if(!num_is_zero(m_length)){ // 判断如果直线长度不为０，则计算只想的方向向量，否则直线首尾两点重合, 直线收缩成一个点
        m_dirVec = m_endPoint - m_startPoint;
    }
}

Point Line::getPointFormStartWithLen(double len) const{
    if(num_is_zero(m_length)){  // 判断如果直线长度为０，则直线收尾两点重合，返回直线终点
        return m_endPoint;
    }
    else{
        return m_startPoint + m_dirVec * len;
    }
}

Point Line::getPointFormEndWithLen(double len) const{
    return getPointFormStartWithLen(getLength() - len);
}

double Line::getLenFromStartWithPoint(const Point &p) const{
    return (p - m_startPoint).norm();
}

Point Line::getCenterPoint() const{
    return (m_startPoint + m_endPoint) * 0.5;
}

double Line::getLength() const{
    return m_length;
}

UnitVector Line::getDirVector() const{
    return m_dirVec;
}

Circle::Circle(){

}

Circle::Circle(const Point &startPoint, const Point &middlePoint, const Point &endPoint, CIRCLETYPE type){
    this->setPoints(startPoint, middlePoint, endPoint, type);
}

Circle::~Circle(){

}

void Circle::setPoints(const Point &startPoint, const Point &middlePoint, const Point &endPoint, CIRCLETYPE type){
    m_startPoint = startPoint;
    m_middlePoint = middlePoint;
    m_endPoint = endPoint;
    m_type = type;

    // 判断是否存在两点重合
    if(num_is_zero((m_startPoint - m_middlePoint).norm()) || num_is_zero((m_startPoint-m_endPoint).norm()) || num_is_zero((m_middlePoint - m_endPoint).norm())){
        m_partLength = 0;
        m_wholeLength = 0;
        return;
    }

    // 根据两个向量叉乘是否为零判断三点是否共线
    Point s2m = m_middlePoint - m_startPoint;
    Point m2e = m_endPoint - m_middlePoint;
    UnitVector axis = s2m.cross(m2e);
    double cross = axis.norm();
    if(num_is_zero(cross)){
        m_partLength = 0;
        m_wholeLength = 0;
        return;
    }

    // 计算圆心
    double x1 = m_startPoint.getValue(POINT_X);
    double y1 = m_startPoint.getValue(POINT_Y);
    double z1 = m_startPoint.getValue(POINT_Z);
    double x2 = m_middlePoint.getValue(POINT_X);
    double y2 = m_middlePoint.getValue(POINT_Y);
    double z2 = m_middlePoint.getValue(POINT_Z);
    double x3 = m_endPoint.getValue(POINT_X);
    double y3 = m_endPoint.getValue(POINT_Y);
    double z3 = m_endPoint.getValue(POINT_Z);
    double x = axis.getValue(POINT_X);
    double y = axis.getValue(POINT_Y);
    double z = axis.getValue(POINT_Z);
    double coe[] = {2 * (x1 - x2), 2 * (y1 - y2), 2 * (z1 - z2),
                  2 * (x1 - x3), 2 * (y1 - y3), 2 * (z1 - z3),
                  x, y, z};
    double res[] = {(pow(x1, 2) + pow(y1, 2) + pow(z1, 2)) - (pow(x2, 2) + pow(y2, 2) + pow(z2, 2)),
                    (pow(x1, 2) + pow(y1, 2) + pow(z1, 2)) - (pow(x3, 2) + pow(y3, 2) + pow(z3, 2)),
                    x*x1+y*y1+z*z1};
    CMatrix<double> matCoe(3, 3, coe);
    CMatrix<double> matRes(3, 1, res);
    CMatrix<double> matCenter = matCoe.inv() * matRes;
    m_centerPoint.setValue(matCenter);

    // 计算半径
    m_radius = (m_centerPoint - m_startPoint).norm();

    // 计算整圆周长
    m_wholeLength = 2 * PI * m_radius;

    // 计算圆平面法相量
    Point os = m_startPoint - m_centerPoint;
    Point om = m_middlePoint - m_centerPoint;
    Point oe = m_endPoint - m_startPoint;
    Point som = os.cross(om);
    Point moe = om.cross(oe);
    if(som.dot(moe) < 0){
        m_normVec = oe.cross(os);
    }
    else{
        m_normVec = os.cross(om);
    }

    // 计算部分圆弧长度
    m_partLength = getLenFromStartWithPoint(m_endPoint);

    // 计算平面圆坐标相对于基坐标的换算关系
    UnitVector ox, oy, oz;
    ox = m_startPoint - m_centerPoint;
    oz = m_normVec;
    oy = oz.cross(ox);
    double rotateMat[] = {ox.getValue(POINT_X), oy.getValue(POINT_X), oz.getValue(POINT_X),
                         ox.getValue(POINT_Y), oy.getValue(POINT_Y), oz.getValue(POINT_Y),
                         ox.getValue(POINT_Z), oy.getValue(POINT_Z), oz.getValue(POINT_Z)};
    m_homoMatrix.setValue(m_centerPoint, RotateMatrix(CMatrix<double>(3, 3, rotateMat)));
}

Point Circle::getPointFormStartWithLen(double len) const{
    double x = m_radius * cos(len / m_wholeLength * 2 * PI);
    double y = m_radius * sin(len / m_wholeLength * 2 * PI);

    CMatrix<double> homePoint(4, 1, {x, y, 0, 1});  // 相对于圆中心坐标系的齐次坐标点
    CMatrix<double> basePoint = m_homoMatrix.getValue() * homePoint;    // 计算相对于基坐标系的坐标值
    return Point(basePoint.at(0, 0), basePoint.at(1, 0), basePoint.at(2, 0));
}

Point Circle::getPointFormEndWithLen(double len) const{
    return getPointFormStartWithLen(getLength() - len);
}

double Circle::getLenFromStartWithPoint(const Point &p) const{
    Point os = m_startPoint - m_centerPoint;
    Point op = p - m_centerPoint;
    Point sop = os.cross(op);
    double sinTheta = sop.norm() / (os.norm() * op.norm());
    if(sop.dot(m_normVec) < 0){
        sinTheta = -sinTheta;
    }
    double cosTheta = os.dot(op) / (os.norm() * op.norm());
    double theta = atan2(sinTheta, cosTheta);
    if(theta < 0){
        theta = 2 * PI +theta;
    }
    return theta * m_radius;
}

Point Circle::getCenterPoint() const{
    return getPointFormStartWithLen(getLength()/2);
}

double Circle::getLength() const{
    if(m_type == PART_CIRCLE){
        return m_partLength;
    }
    else{
        return m_wholeLength;
    }
}

UnitVector Circle::getDirVector(const Point &p) const{
    UnitVector dirVec = m_normVec.cross(p - m_centerPoint);
    return dirVec;
}

UnitVector Circle::getNormVector() const{
    return m_normVec;
}

Slerp::Slerp(){

}

Slerp::Slerp(const Quaternion &startAttitude, const Quaternion &endAttitude){
    this->setAttitudes(startAttitude, endAttitude);
}

Slerp::~Slerp(){

}

void Slerp::setAttitudes(const Quaternion &startAttitude, const Quaternion &endAttitude){
    m_startAttitude = startAttitude;
    m_endAttitude = endAttitude;
    double rt = m_startAttitude*m_endAttitude;
    if(rt>1-EPSLON)
    {
        m_theta = 0;
    }else if(rt < -1+EPSLON)
    {
        m_theta = PI;
    }else {
        m_theta = acos(rt);
    }
}

Quaternion Slerp::getAttitudeWithRatio(double ratio) const{
    if(ratio<(0-EPSLON) || ratio>(1+EPSLON)){
        cout << "Quaternion::getAttitudeWithRatio ratio should belong to [0, 1]" << endl;
        exit(1);
    }
    if(num_is_zero(m_theta)){
        return m_endAttitude;
    }
    else{
        return m_startAttitude*(sin((1-ratio)*m_theta)/sin(m_theta))+m_endAttitude*(sin(ratio*m_theta)/sin(m_theta));
    }
}

#endif
