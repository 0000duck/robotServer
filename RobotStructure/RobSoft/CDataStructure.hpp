#ifndef CDATASTRUCTURE_HPP
#define CDATASTRUCTURE_HPP

#include <vector>
#include <list>

#include "CMatrix.hpp"

namespace robsoft{

enum JOINTINDEX{JOINT_WHOLE=-1, JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, JOINT_6, JOINT_7, JOINT_8};   // 关节序号
enum TERMINALINDEX{TERMINAL_X, TERMINAL_Y, TERMINAL_Z, TERMINAL_A, TERMINAL_B, TERMINAL_C}; // 末端位姿序号
enum POINTINDEX{POINT_X, POINT_Y, POINT_Z}; // 点的序号
enum ATTITUDEINDEX{ATTITUDE_A, ATTITUDE_B, ATTITUDE_C}; // 姿态角序号
enum QUATERNIONINDEX{QUATERNION_W, QUATERNION_X, QUATERNION_Y, QUATERNION_Z};   // 四元素序号
enum COORDINATESYSTEM{COORDINATE_BASE, COORDINATE_WORK, COORDINATE_TOOL};    // 坐标系
enum CIRCLETYPE {PART_CIRCLE, WHOLE_CIRCLE};    // 部分圆，整圆
enum CURVETYPE {OPEN_CURVE, CLOSE_CURVE};    // 部分圆，整圆

MY_CLASS Joints{   // 关节
public:
    Joints();
    Joints(int dof);
    Joints(int dof, double* value);
    Joints(int dof, const std::vector<double>& value);
    Joints(int dof, const CMatrix<double>& value);
    ~Joints();

    void setValue(int dof);
    void setValue(int dof, double* value);
    void setValue(int dof, const std::vector<double>& value);
    void setValue(int dof, const CMatrix<double>& value);

    void append(const Joints& joint);

    int getJointsDOF() const;
    double getValue(JOINTINDEX index) const;
    CMatrix<double> getValue() const;
    void getValue(double* value) const;
    void getValue(std::vector<double>& value) const;
    void getValue(CMatrix<double>& value) const;
    double norm(const Joints& joint) const;

    double& operator [](JOINTINDEX index);
    const double& operator [](JOINTINDEX index) const;
    Joints operator +(const Joints& joint) const;
    Joints operator -(const Joints& joint) const;
    Joints operator *(double value) const;
    Joints operator /(double value) const;

    bool operator ==(const Joints& joint) const;
    bool operator !=(const Joints& joint) const;

    bool judgeOverMinimum(const Joints& joint) const;
    bool judgeOverMaximum(const Joints& joint) const;

    void print(const char* str = "Joints") const;

protected:
    int m_dof;
    CMatrix<double> m_value;  // m_dof X 1
};

MY_CLASS JointsList{   // 关节列表
public:
    JointsList();
    JointsList(const std::vector<Joints> &js);
    ~JointsList();

    void clear();
    void push_back(const Joints& element);
    void push_front(const Joints& element);
    void pop_back();
    void erase(int index);
    Joints& operator [](int index);
    const Joints& operator [](int index) const;

    CMatrix<double> getCertainJointList(JOINTINDEX index) const;    // 获取关节空间点列中的某一关节的点列

    bool empty() const;
    int size() const;

private:
    std::vector<Joints> m_vector;
};

class JointsMotionState{
public:
    JointsMotionState();
    JointsMotionState(int dof);
    ~JointsMotionState();

    void setValue(int dof);
    void setPosValue(const Joints& pos);
    void setVelValue(const Joints& vel);
    void setAccValue(const Joints& acc);

    int getJointsDOF() const;
    Joints getPosValue() const;
    Joints getVelValue() const;
    Joints getAccValue() const;

    void append(const JointsMotionState& jMotion);
    void calVelAcc(const JointsMotionState& jMotion, double time);

private:
    Joints m_pos;
    Joints m_vel;
    Joints m_acc;
};

class JointsMotionStateList{   // 关节状态列表
public:
    JointsMotionStateList();
    ~JointsMotionStateList();

    void clear();
    void push_back(const JointsMotionState& element);
    void pop_back();
    void pop_front();
    void erase(int index);
    void append(const JointsMotionStateList& jMotionList);
    JointsMotionState front() const;
    JointsMotionState back() const;
    JointsMotionState& operator [](int index);
    const JointsMotionState& operator [](int index) const;

    bool empty() const;
    int size() const;

    void print() const;

private:
    std::vector<JointsMotionState> m_vector;
};


MY_CLASS Point{    // 点的位置
public:
    Point();
    Point(double valueX, double valueY, double valueZ);
    Point(const CMatrix<double>& value);
    ~Point();

    void setValue(double valueX, double valueY, double valueZ);
    void setValue(const CMatrix<double>& value);

    double getValue(POINTINDEX index) const;
    CMatrix<double> getValue() const;
    void getValue(double& valueX, double& valueY, double& valueZ) const;
    void getValue(CMatrix<double>& value) const;

    double& operator [](POINTINDEX index);
    const double& operator [](POINTINDEX index) const;
    Point operator +(const Point& point) const;
    Point operator -(const Point& point) const;
    Point operator *(double value) const;

    double dot(const Point& point) const;   // 点积
    Point cross(const Point& point) const;  // 叉乘
    double norm() const;    // 欧氏距离
    Point normalization() const;    // 归一化

    void print(const char* str = "Point") const;

protected:
    CMatrix<double> m_value;    // 3 X 1
};

class Point;
MY_CLASS UnitVector:public Point{  // 单位向量
public:
    UnitVector();
    UnitVector(double valueX, double valueY, double valueZ);
    UnitVector(const Point& point);
    UnitVector(const CMatrix<double>& value);
    ~UnitVector();

    void setValue(double valueX, double valueY, double valueZ);
    void setValue(const Point& point);
    void setValue(const CMatrix<double>& value);

    void operator=(const Point& point);

    void print(const char* str = "UnitVector") const;
};

class RotateMatrix;
class DualVector;
class Quaternion;
class RotateAxis;
MY_CLASS AttitudeAngle{    // xyz固定角，先绕x轴gamma，再绕y轴beta，最后绕z轴alpha
public:
    AttitudeAngle();
    AttitudeAngle(double valueA, double valueB, double valueC);
    AttitudeAngle(const CMatrix<double>& value);
    ~AttitudeAngle();

    void setValue(double valueA, double valueB, double valueC);
    void setValue(const CMatrix<double>& value);

    double getValue(ATTITUDEINDEX index) const;
    CMatrix<double> getValue() const;
    void getValue(double& valueA, double& valueB, double& valueC) const;
    void getValue(CMatrix<double>& value) const;

    RotateMatrix getRotateMatrix() const;
    DualVector getDualVector() const;
    Quaternion getQuaternion() const;

    double& operator [](ATTITUDEINDEX index);
    const double& operator [](ATTITUDEINDEX index) const;
    AttitudeAngle operator +(const AttitudeAngle& angle) const;
    AttitudeAngle operator -(const AttitudeAngle& angle) const;
    AttitudeAngle operator *(double value) const;

    void print(const char* str = "AttitudeAngle") const;

protected:
    CMatrix<double> m_value;    // 3 X 1
};

class AttitudeAngle;
class DualVector;
class Quaternion;
class RotateAxis;
class RotateMatrix{ // 旋转矩阵
public:
    RotateMatrix();
    RotateMatrix(const CMatrix<double>& value);
    ~RotateMatrix();

    void setValue(const CMatrix<double>& value);

    CMatrix<double> getValue() const;

    AttitudeAngle getAttitudeAngle() const;
    DualVector getDualVector() const;
    Quaternion getQuaternion() const;
    RotateAxis getRotateAxis() const;

    RotateMatrix operator *(double value) const;
    RotateMatrix operator +(const RotateMatrix& rotate) const;
    RotateMatrix operator -(const RotateMatrix& rotate) const;
    RotateMatrix operator *(const RotateMatrix& rotate) const;
    Point operator *(const Point& point) const;
    RotateMatrix trans() const;
    RotateMatrix inv() const;

    void print(const char* str = "RotateMatrix") const;

protected:
    CMatrix<double> m_value;    // 3 X 3
};


class AttitudeAngle;
class RotateMatrix;
class Quaternion;
class RotateAxis;
MY_CLASS DualVector{   // 双向量
public:
    DualVector();
    DualVector(const UnitVector& dirVector, const UnitVector& normVector);
    ~DualVector();

    void setValue(const UnitVector& dirVector, const UnitVector& normVector);
    void setDirVector(const UnitVector &dirVector);
    void setNormVector(const UnitVector& normVector);

    UnitVector getDirVector() const;
    UnitVector getNormVector() const;

    RotateMatrix getRotateMatrix() const;
    AttitudeAngle getAttitudeAngle() const;
    Quaternion getQuaternion() const;

    void print(const char* str = "DualVector") const;

protected:
    UnitVector m_dirVector;
    UnitVector m_normVector;
};


class AttitudeAngle;
class RotateMatrix;
class DualVector;
class RotateAxis;
class Quaternion{   // 四元数
public:
    Quaternion();
    Quaternion(const CMatrix<double>& value);
    Quaternion(double valueW, double valueX, double valueY, double valueZ);
    ~Quaternion();

    void setValue(const CMatrix<double>& value);
    void setValue(double valueW, double valueX, double valueY, double valueZ);

    double getValue(QUATERNIONINDEX index) const;

    RotateMatrix getRotateMatrix() const;
    AttitudeAngle getAttitudeAngle() const;
    DualVector getDualVector() const;

    Quaternion operator *(double value) const;
    double operator *(const Quaternion &q) const;
    Quaternion operator +(const Quaternion &q) const;
    double dot(const Quaternion &q) const;
    Quaternion cross(const Quaternion &q) const;
    double norm() const;

    void print(const char* str = "Quaternion") const;

protected:
    CMatrix<double> m_value;    // 0:w 1:x 2:y 3:z   4 X 1
};


class AttitudeAngle;
class RotateMatrix;
class DualVector;
class Quaternion;
class RotateAxis{   // 绕固定轴旋转的姿态表示
public:
    RotateAxis();
    RotateAxis(const UnitVector& axis, double angle);
    ~RotateAxis();

    void setValue(const UnitVector& axis, double angle);

    UnitVector getRotateAxis() const;
    double getRotateAngle() const;
    RotateMatrix getRotateMatrix() const;

    void print(const char* str = "RotateAxis") const;

protected:
    UnitVector m_axis;
    double m_angle;
};

class Point;
class AttitudeAngle;
class RotateMatrix;
class HomogeneousMatrix;
MY_CLASS Terminal{ // 末端位姿
public:
    Terminal();
    Terminal(const CMatrix<double>& value);
    Terminal(double valueX, double valueY, double valueZ, double valueA, double valueB, double valueC);
    Terminal(const Point& point, const AttitudeAngle& attitude);
    Terminal(const Point& point, const RotateMatrix& rotate);
    Terminal(const HomogeneousMatrix& homo);
    ~Terminal();

    void setValue(const CMatrix<double>& value);
    void setValue(double valueX, double valueY, double valueZ, double valueA, double valueB, double valueC);
    void setValue(const Point& point, const AttitudeAngle& attitude);
    void setValue(const Point& point, const RotateMatrix& rotate);
    void setValue(const HomogeneousMatrix& homo);

    Point getPoint() const;
    AttitudeAngle getAttitudeAngle() const;
    RotateMatrix getRotateMatrix() const;
    HomogeneousMatrix getHomogeneousMatrix() const;

    double getValue(TERMINALINDEX index) const;

    Terminal getTerminalInWorkFrame(Terminal workframe) const;  // 已知基坐标系下值，求工件坐标系下的值
    Terminal getTerminalFromWorkFrame(Terminal workframe) const;    // 已知工件坐标系下值，求基坐标系下值

    double& operator [](TERMINALINDEX index);
    const double& operator [](TERMINALINDEX index) const;
    Terminal operator +(const Terminal& terminal) const;
    Terminal operator -(const Terminal& terminal) const;
    Terminal operator *(const Terminal& terminal) const;
    Terminal operator *(double value) const;

    bool operator ==(const Terminal& terminal) const;
    bool operator !=(const Terminal& terminal) const;

    bool judgeOverMinimum(const Terminal& terminal) const;
    bool judgeOverMaximum(const Terminal& terminal) const;

    void print(const char* str = "Terminal") const;

protected:
    Point m_point;
    AttitudeAngle m_attitude;
};

class TerminalList{   // 关节状态列表
public:
    TerminalList();
    TerminalList(const std::vector<Terminal> &ts);
    ~TerminalList();

    void clear();
    void push_back(const Terminal& element);
    void push_front(const Terminal& element);
    void pop_back();
    void erase(int index);
    Terminal& operator [](int index);
    const Terminal& operator [](int index) const;

    bool empty() const;
    int size() const;

private:
    std::vector<Terminal> m_vector;
};

class Point;
class AttitudeAngle;
class RotateMatrix;
class Terminal;
MY_CLASS HomogeneousMatrix{    // 齐次矩阵
public:
    HomogeneousMatrix();
    HomogeneousMatrix(const CMatrix<double>& value);
    HomogeneousMatrix(const Point& point, const AttitudeAngle& attitude);
    HomogeneousMatrix(const Point& point, const RotateMatrix& rotate);

    ~HomogeneousMatrix();

    void setValue(const CMatrix<double>& value);
    void setValue(const Point& point, const AttitudeAngle& attitude);
    void setValue(const Point& point, const RotateMatrix& rotate);

    CMatrix<double> getValue() const;
    double getValue(int i, int j) const;

    Point getPoint() const;
    AttitudeAngle getAttitudeAngle() const;
    RotateMatrix getRotateMatrix() const;
    Terminal getTerminal() const;

    HomogeneousMatrix operator *(const HomogeneousMatrix& homo) const;
    HomogeneousMatrix inv() const;

    HomogeneousMatrix getHomogeneousMatrixInWorkFrame(HomogeneousMatrix workframe) const;   // 已知基坐标系下值，求工件坐标系下的值
    HomogeneousMatrix getHomogeneousMatrixFromWorkFrame(HomogeneousMatrix workframe) const; // 已知工件坐标系下值，求基坐标系下值

    void print(const char* str = "HomogeneousMatrix") const;

protected:
    CMatrix<double> m_value;    // 4 X 4
};

MY_CLASS RobotMotion{
public:
    RobotMotion();
    ~RobotMotion();

    void setCurrentJointPosition(const Joints& joint);
    Joints getCurrentJointPosition() const;

    void setCurrentJointVelocity(const Joints& joint);
    Joints getCurrentJointVelocity() const;

    void setCurrentJointAcceleration(const Joints& joint);
    Joints getCurrentJointAcceleration() const;

    void setCurrentJointTorque(const Joints& joint);
    Joints getCurrentJointsTorque() const;

    void setCurrentTerminal(const Terminal& terminal);
    Terminal getCurrentTerminal() const;

    void setCurrentWorkTerminal(const Terminal& terminal);
    Terminal getCurrentWorkTerminal() const;

private:
    Joints m_currentJointPosition;            //当前关节角度
    Joints m_currentJointVelocity;         //当前关节速度
    Joints m_currentJointAcceleration;         //当前关节加速度
    Joints m_currentJointTorque;           //当前关节力矩
    Terminal m_currentTerminal;       //当前末端位姿
    Terminal m_currentWorkTerminal;   //当前工件坐标系下姿态
};

}

#endif
