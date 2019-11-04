#ifndef CPOLYNOMIAL_HPP
#define CPOLYNOMIAL_HPP

#include "CMatrix.hpp"

namespace robsoft{

class NewtonPolynomial{ // 牛顿插值多项式
public:
    NewtonPolynomial();
    NewtonPolynomial(const CMatrix<double>& x, const CMatrix<double>& y);
    ~NewtonPolynomial();

    void setPoints(const CMatrix<double>& x, const CMatrix<double>& y);  // 列向量，点列
    double getValue(double x) const;  // 根据变量返回值
    double getDerivative(double x, int n) const;    // 根据变量返回n阶导数

    void print(const char* str = "NewtonPolynomial") const;

private:
    CMatrix<double> m_x;    // 变量
    CMatrix<double> m_dividedDifference;    // 差商
};

class QuinticPolynomial{    // 五次多项式
public:
    QuinticPolynomial();
    QuinticPolynomial(const CMatrix<double>& coefficient);
    ~QuinticPolynomial();

    void setCoefficient(const CMatrix<double>& coefficient);  // 设置多项式系数，列向量
    void setBoundaryCondition(const CMatrix<double>& boundaryCondition);    // 根据边界条件（值、导数）初始化多项式
    /* boundaryCondition =
     * | x1 f1 fd1 fdd1 |
     * | x2 f2 fd2 fdd2 |
     */

    double getValue(double x) const;  // 根据变量返回值
    double getDerivative(double x, int n) const;    // 根据变量返回n阶导数

    void print(const char* str = "QuinticPolynomial") const;

private:
    CMatrix<double> m_coefficient;  // 列向量，多项式系数，a0, a1, a2...
};

class QuinticNewtonPolynomial{  // 五次牛顿多项式，首尾段用五次多项式拟合，中间段用牛顿插值多项式拟合
public:
    QuinticNewtonPolynomial();
    QuinticNewtonPolynomial(double period);
    QuinticNewtonPolynomial(double period, const CMatrix<double>& x, const CMatrix<double>& y);
    QuinticNewtonPolynomial(double period, const CMatrix<double>& x, const CMatrix<double>& y, const CMatrix<double>& yd, const CMatrix<double>& ydd);
    ~QuinticNewtonPolynomial();

    void setPeriod(double period);    // 设置周期时间
    void setPoints(const CMatrix<double>& x, const CMatrix<double>& y);  // 列向量，点列
    void setPoints(const CMatrix<double>& x, const CMatrix<double>& y, const CMatrix<double>& yd, const CMatrix<double>& ydd);  // 列向量，点列

    void resetState();  // 重置状态
    int getSegmentNum() const; // 获取轨迹段数
    CMatrix<double> getNextSegment();    // 分段依次获取轨迹点，ｎＸ３
    CMatrix<double> getAllSegment(); // 获取全部的轨迹点，ｎＸ３

private:
    double m_period;  // 周期时间
    CMatrix<double> m_x;    // 列向量，变量
    CMatrix<double> m_y;    // 列向量，值
    QuinticPolynomial m_firstSegment;  // 第一段五次多项式
    QuinticPolynomial m_endSegment;   // 最后一段五次多项式
    NewtonPolynomial m_middleSegment;  // 中间段Newton多项式

    int m_index;    // 分段取点序号
    double m_time;
};

class QuinticContinuousPolynomial{  // 分段五次多项式拟合
public:
    QuinticContinuousPolynomial();
    QuinticContinuousPolynomial(double period);
    QuinticContinuousPolynomial(double period, const CMatrix<double>& x, const CMatrix<double>& y);
    QuinticContinuousPolynomial(double period, const CMatrix<double>& x, const CMatrix<double>& y, const CMatrix<double>& yd, const CMatrix<double>& ydd);
    ~QuinticContinuousPolynomial();

    void setPeriod(double period);    // 设置周期时间
    void setPoints(const CMatrix<double>& x, const CMatrix<double>& y);  // 列向量，点列
    void setPoints(const CMatrix<double>& x, const CMatrix<double>& y, const CMatrix<double>& yd, const CMatrix<double>& ydd);  // 列向量，点列

    void resetState();  // 重置状态
    int getSegmentNum() const; // 获取轨迹段数
    CMatrix<double> getNextSegment();    // 分段依次获取轨迹点，ｎＸ３
    CMatrix<double> getAllSegment(); // 获取全部的轨迹点，ｎＸ３

private:
    double m_period;  // 周期时间
    CMatrix<double> m_x;    // 列向量，变量
    CMatrix<double> m_y;    // 列向量，值
    CMatrix<double> m_yd;    // 列向量，值一阶导数
    CMatrix<double> m_ydd;    // 列向量，值二阶导数

    int m_index;    // 分段取点序号
    double m_time;
};

class JOGPolynomial{    // 点动曲线，Ｓ型加速，梯形减速
public:
    JOGPolynomial();
    JOGPolynomial(double period);
    JOGPolynomial(double period, double vel, double acc, double jerk);
    ~JOGPolynomial();

    void setPeriod(double period);    // 设置周期时间
    void setMotionCoefficient(double vel, double acc, double jerk); // 设置运动参数

    void resetState();  // 重置状态
    bool isEnd() const;   // 判断取点是否已经结束
    CMatrix<double> getNextPoint();
    void setEnd();

private:
    double m_period;  // 周期时间
    double m_vmax, m_amax, m_jmax;    // 最大速度，加速度，冲击

    double m_t1, m_t2, m_t3, m_t4, m_t5;
    double m_v1, m_v2, m_v3;
    double m_p1, m_p2, m_p3;

    double m_time;  // 记录当前点相距开始点动的时间
    double m_vs, m_ps;  // 随时记录当前点的位置和速度，作为停止点动时，减速的初始状态
    int m_moveEndFlag;  // 0:正常 1:开始结束 2:已结束
};

class SPolynomial{  // 步进型曲线，状态从零到零，Ｓ型加减速
public:
    SPolynomial();
    SPolynomial(double period);
    ~SPolynomial();

    void setPeriod(double period);    // 设置周期时间
    void setMotionCoefficientWithVel(double len, double vel, double acc, double jerk); // 设置运动参数，限制目标速度
    int setMotionCoefficientWithTime(double len, double time, double acc, double jerk); // 设置运动参数，限制目标时间

    double getTime(double len) const; // 根据距离计算时间
    double getValue(double x) const;  // 根据变量返回值
    double getDerivative(double x, int n) const;    // 根据变量返回n阶导数
    double getAccLength() const;    // 返回加速的距离

    void resetState();  // 重置状态
    CMatrix<double> getAllSegment();    // 获取全部的轨迹点，ｎＸ３

private:
    double m_period;    // 周期时间
    double m_length;    // 总长度
    double m_vmax, m_amax, m_jmax;    // 最大速度，加速度，冲击

    double m_t1, m_t2, m_t3, m_t4, m_t5, m_t6, m_t7;
    double m_v1, m_v2, m_v3;
    double m_p1, m_p2, m_p3, m_p4, m_p5, m_p6, m_p7;
};

class SPolynomialGen{  // S型曲线，始末速度和加速度不一定为０，给定最大冲击和长度，给定稳定时速度或者运行时间
public:
    SPolynomialGen();
    SPolynomialGen(double period);
    SPolynomialGen(double period, double len, double vel, double acc, double jerk);
    ~SPolynomialGen();

    void setPeriod(double period);
    void setMotionCoefficient(double len, double vel, double acc, double jerk);
    void setVelMode(double vel, double startVel, double startAcc, double endVel, double endAcc);
    void setTimeMode(double time, double startVel, double startAcc, double endVel, double endAcc);

    void resetState();  // 重置状态
    CMatrix<double> getAllSegment();    // 获取全部的轨迹点，ｎＸ３

private:
    double m_period;    // 周期时间
    double m_length;    // 总长度
    double m_vmax, m_amax, m_jmax;    // 最大速度，加速度，冲击
    double m_sVel, m_sAcc, m_eVel, m_eAcc;  // 起始速度、加速度，结束速度、加速度
    int m_mode; // 运行模式，0:目标速度模式　1:总时间模式
    double m_vel; // 目标速度
    double m_time; // 总时间
};

}

#endif
