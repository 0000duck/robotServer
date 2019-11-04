#ifndef CPATHPLANNING_HPP
#define CPATHPLANNING_HPP

#include "CDataStructure.hpp"

namespace robsoft{

class Line{ // 空间直线
public:
    Line();
    Line(const Point &startPoint, const Point &endPoint);
    ~Line();

    void setPoints(const Point &startPoint, const Point &endPoint);

    Point getPointFormStartWithLen(double len) const; // 根据距离起点的距离计算点的坐标
    Point getPointFormEndWithLen(double len) const; // 根据距离终点的距离计算点的坐标
    double getLenFromStartWithPoint(const Point &p) const;  // 根据点计算到起点的距离
    Point getCenterPoint() const;   // 获取直线的中心点
    double getLength() const;   // 获取直线长度
    UnitVector getDirVector() const;    // 获取直线方向向量

private:
    Point m_startPoint; // 直线起始点
    Point m_endPoint;   // 直线中点

    UnitVector m_dirVec;    // 直线方向向量
    double m_length;    // 直线总长度
};

class Circle{   // 空间圆
public:
    Circle();
    Circle(const Point &startPoint, const Point &middlePoint, const Point &endPoint, CIRCLETYPE type = PART_CIRCLE);
    ~Circle();

    void setPoints(const Point &startPoint, const Point &middlePoint, const Point &endPoint, CIRCLETYPE type = PART_CIRCLE);

    Point getPointFormStartWithLen(double len) const; // 根据距离起点的距离计算点的坐标
    Point getPointFormEndWithLen(double len) const; // 根据距离终点的距离计算点的坐标
    double getLenFromStartWithPoint(const Point &p) const;  // 根据点计算到起点的距离
    Point getCenterPoint() const;   // 获取圆弧段的中心点
    double getLength() const;
    UnitVector getDirVector(const Point &p) const;    // 获取圆弧上点的方向向量
    UnitVector getNormVector() const;   // 获取圆弧的法向量

private:
    Point m_startPoint; // 圆起始点
    Point m_middlePoint;    // 圆中间点
    Point m_endPoint;   // 圆终点
    CIRCLETYPE m_type;  // 圆类型

    UnitVector m_normVec;   // 圆面法向量，右手定则
    Point m_centerPoint;    // 圆心所在位置
    double m_radius;    // 圆的半径
    double m_partLength;    // 部分圆的弧长，从圆起始点到圆终点
    double m_wholeLength;   // 整圆的周长

    HomogeneousMatrix m_homoMatrix; // 以圆心为原点，圆心和起始点连线为ｘ轴，法向量为ｙ轴建立坐标系相对于基座标的齐次矩阵
};

class Slerp{    // 姿态四元数Slerp球面
public:
    Slerp();
    Slerp(const Quaternion &startAttitude, const Quaternion &endAttitude);
    ~Slerp();

    void setAttitudes(const Quaternion &startAttitude, const Quaternion &endAttitude);

    Quaternion getAttitudeWithRatio(double ratio) const;

private:
    Quaternion m_startAttitude;
    Quaternion m_endAttitude;

    double m_theta;
};

class BCurve{
public:
    BCurve();
    BCurve(const std::vector<Point> &ps, CURVETYPE type = OPEN_CURVE);
    ~BCurve();

    void setPoints(const std::vector<Point> &ps, CURVETYPE type = OPEN_CURVE);

private:
    std::vector<Point> m_points;
    CURVETYPE m_type;
};

}

#endif
