#ifndef CROBOTPARAMETER_HPP
#define CROBOTPARAMETER_HPP

#include "CDataStructure.hpp"

namespace robsoft{

enum ROBOTTYPE{ROBSOFT_SERIAL_SIX_CONVENTION,   // 六轴常规串联机型
               ROBSOFT_SERIAL_SIX_COOPERATION,  // 六轴协作串联机型
               ROBSOFT_SERIAL_FOUR_CONVENTION,  // 四轴常规串联机型，码垛
               ROBOTSOFT_SCARA_FOUR_ONERF,      // SCARA机器人，一轴可升降
               ROBOTSOFT_SCARA_FOUR_FOURRF,     // SCARA机器人，四轴可升降
               ROBOTSOFT_DELTA_FOUR,            // DELTA机器人，单姿态
               ROBOTSOFT_DELTA_SIX              // DELTA机器人，三姿态
              };

MY_CLASS RobotParameter{
public:
    RobotParameter();
    ~RobotParameter();

    void readRobotParameter(const char* path);
    void writeRobotParameter(const char* path) const;

    bool isDriverSame(const RobotParameter& robot) const;

    void setRobotParameter(const RobotParameter& robot);
    RobotParameter getRobotParameter() const;

    void setRobotDOF(int robotDof);
    void setExternDOF(int externDof);

    void setRobotType(ROBOTTYPE robotType);
    void setSamplePeriod(double samplePeriod);

    void setEncoderResolution(const Joints& encoderResolution);
    void setRateTorque(const Joints& rateTorque);
    void setReduceRatio(const Joints& reduceRatio);

    void setJointRange(const Joints& jointRangeMinus, const Joints& jointRangePlus);
    void setJointMaxVelRange(const Joints& jointMaxVel);
    void setJointMaxAccRange(const Joints& jointMaxAcc);
    void setJointMaxJerkRange(const Joints& jointMaxJerk);
    void setJointMaxVelRatio(double jointMaxVelRatio);
    void setJointMaxAccRatio(double jointMaxAccRatio);
    void setJointMaxJerkRatio(double jointMaxJerkRatio);

    void setTerminalRange(const Terminal& terminalRangeMinus, const Terminal& terminalRangePlus);
    void setTerminalMaxVelRange(const Terminal& terminalMaxVel);
    void setTerminalMaxAccRange(const Terminal& terminalMaxAcc);
    void setTerminalMaxJerkRange(const Terminal& terminalMaxJerk);
    void setTerminalMaxVelRatio(double terminalMaxVelRatio);
    void setTerminalMaxAccRatio(double terminalMaxAccRatio);
    void setTerminalMaxJerkRatio(double terminalMaxJerkRatio);

    void setDHParameter(const Joints& DHParameterAlpha, const Joints& DHParameterA, const Joints& DHParameterD, const Joints& DHParameterTheta);

    void setToolFrame(const Terminal& toolFrame);
    void setWorkFrame(const Terminal& workFrame);

    void getRobotType(ROBOTTYPE& robotType) const;
    void getSamplePeriod(double& samplePeriod) const;

    void getEncoderResolution(Joints& encoderResolution) const;
    void getRateTorque(Joints& rateTorque) const;
    void getReduceRatio(Joints& reduceRatio) const;

    void getJointRange(Joints& jointRangeMinus, Joints& jointRangePlus) const;
    void getJointMaxVelRange(Joints& jointMaxVel) const;
    void getJointMaxAccRange(Joints& jointMaxAcc) const;
    void getJointMaxJerkRange(Joints& jointMaxJerk) const;
    void getJointMaxVel(Joints& jointMaxVel) const;
    void getJointMaxAcc(Joints& jointMaxAcc) const;
    void getJointMaxJerk(Joints& jointMaxJerk) const;
    void getJointMaxVelRatio(double& jointMaxVelRatio) const;
    void getJointMaxAccRatio(double& jointMaxAccRatio) const;
    void getJointMaxJerkRatio(double& jointMaxJerkRatio) const;

    void getTerminalRange(Terminal& terminalRangeMinus, Terminal& terminalRangePlus) const;
    void getTerminalMaxVelRange(Terminal& terminalMaxVel) const;
    void getTerminalMaxAccRange(Terminal& terminalMaxAcc) const;
    void getTerminalMaxJerkRange(Terminal& terminalMaxJerk) const;
    void getTerminalMaxVel(Terminal& terminalMaxVel) const;
    void getTerminalMaxAcc(Terminal& terminalMaxAcc) const;
    void getTerminalMaxJerk(Terminal& terminalMaxJerk) const;
    void getTerminalMaxVelRatio(double& terminalMaxVelRatio) const;
    void getTerminalMaxAccRatio(double& terminalMaxAccRatio) const;
    void getTerminalMaxJerkRatio(double& terminalMaxJerkRatio) const;

    void getDHParameter(Joints& DHParameterAlpha, Joints& DHParameterA, Joints& DHParameterD, Joints& DHParameterTheta) const;

    void getToolFrame(Terminal& toolFrame) const;
    void getWorkFrame(Terminal& workFrame) const;

    ROBOTTYPE getRobotType() const;
    double getSamplePeriod() const;

    Joints getEncoderResolution() const;
    Joints getRateTorque() const;
    Joints getReduceRatio() const;

    int getRobotDOF() const;
    int getExternDOF() const;
    int getWholeDOF() const;

    Joints getJointRangeMinus() const;
    Joints getJointRangePlus() const;
    Joints getJointMaxVelRange() const;
    Joints getJointMaxAccRange() const;
    Joints getJointMaxJerkRange() const;
    Joints getJointMaxVel() const;  // 返回的最大速度已乘上比例系数
    Joints getJointMaxAcc() const;  // 返回的最大加速度已乘上比例系数
    Joints getJointMaxJerk() const; // 返回的最大冲击已乘上比例系数

    Terminal getTerminalRangeMinus() const;
    Terminal getTerminalRangePlus() const;
    Terminal getTerminalMaxVelRange() const;
    Terminal getTerminalMaxAccRange() const;
    Terminal getTerminalMaxJerkRange() const;
    Terminal getTerminalMaxVel() const; // 返回的最大速度已乘上比例系数
    Terminal getTerminalMaxAcc() const; // 返回的最大加速度已乘上比例系数
    Terminal getTerminalMaxJerk() const;    // 返回的最大冲击已乘上比例系数

    Terminal getToolFrame() const;
    Terminal getWorkFrame() const;

    bool operator ==(const RobotParameter &robot) const;
    bool operator !=(const RobotParameter &robot) const;

    void print() const;

protected:
    int m_robotDOF;                     //机器人自由度
    int m_externDOF;                    //外部轴自由度

    ROBOTTYPE m_robotType;              //机器人类型
    double m_samplePeriod;              //指令周期，单位秒

    Joints m_encoderResolution;         //编码器分辨率
    Joints m_rateTorque;                //额定转矩
    Joints m_reduceRatio;               //减速器减速比  输入/输出

    Joints m_jointRangeMinus;           //负向关节范围
    Joints m_jointRangePlus;            //正向关节范围
    Joints m_jointMaxVel;               //关节最大速度
    Joints m_jointMaxAcc;               //关节最大加速度
    Joints m_jointMaxJerk;              //关节最大冲击
    double m_jointMaxVelRatio;          //关节最大速度比例
    double m_jointMaxAccRatio;          //关节最大加速度比例
    double m_jointMaxJerkRatio;         //关节最大冲击比例

    Terminal m_terminalRangeMinus;      //负向末端位置范围
    Terminal m_terminalRangePlus;       //正向末端位置范围
    Terminal m_terminalMaxVel;          //末端最大速度
    Terminal m_terminalMaxAcc;          //末端最大加速度
    Terminal m_terminalMaxJerk;         //末端最大冲击
    double m_terminalMaxVelRatio;       //末端最大速度比例
    double m_terminalMaxAccRatio;       //末端最大加速度比例
    double m_terminalMaxJerkRatio;      //末端最大冲击比例

    Joints m_DHParameterAlpha;         //DH参数, alpha
    Joints m_DHParameterA;             //DH参数, a
    Joints m_DHParameterD;             //DH参数, d
    Joints m_DHParameterTheta;         //DH参数, theta

    Terminal m_toolFrame;               //工具坐标系
    Terminal m_workFrame;               //工件坐标系
};

}

#endif
