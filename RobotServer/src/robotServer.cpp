#include "robotServer.h"
#include "functions.h"
using namespace std;
using namespace rclib;

RobotServer* RobotServer::m_robotServer = new RobotServer;
RobotServer* RobotServer::robotServer;

RobotServer::RobotServer()
		: isFirst(1)
{
    robotServer = this;
    m_controlFlag = SWITCHOFF;
    programLoad((m_strPath + ROBOT_PROGRAM_PATH).c_str());
}

RobotServer* RobotServer::initance(){
    return m_robotServer;
}

void RobotServer::delInitance()
{
    if(m_robotServer)
    {
        delete m_robotServer;
        m_robotServer = NULL;
    }
}

RobotServer::~RobotServer(){
    setSystemStateLight(SWITCHOFF);
}

bool RobotServer::initSystem(int listen_port){
    m_robotFrame.readRobotFrame((m_strPath + ROBOT_FRAME_PATH).c_str());
    m_robotPreference.readRobotPreference((m_strPath + ROBOT_PREFERENCE_PATH).c_str());

    // robotics初始化
    m_robotParameter.readRobotParameter((m_strPath+ROBOT_PARAMETER_PATH).c_str());
    m_robotParameter.setToolFrame(m_robotFrame.getToolFrame(m_robotFrame.getCurrentToolFrame()));
    m_robotParameter.setWorkFrame(m_robotFrame.getWorkFrame(m_robotFrame.getCurrentWorkFrame()));
    setDof(m_robotParameter.getWholeDOF());
    m_robot.setRobotParameter(m_robotParameter);

    m_controlFlag = SWITCHON;

    setServo(m_robotState.getServoState());
    setRunStateLight(m_robotState.getRunState());
    setPlayStateLight(m_robotState.getPlayState());
    setSystemStateLight(SWITCHOFF);
    setErrorStateLight(SWITCHOFF);

    // 嵌入式初始化
    initLink(m_robotParameter.getSamplePeriod(), m_robotParameter.getEncoderResolution(), m_robotParameter.getReduceRatio(), m_robotParameter.getRateTorque());

    setSystemStateLight(SWITCHON);

    m_timerUpdateState.start(0.1);
    int ret;
    if(ret = m_threadUpdateState.create(cycleTaskupdateState, NULL)!=0){
        printf("create update state thread failed %d\n", ret);
        exit(-1);
    }

    // TCP初始化
    initSystemBase(listen_port);

    return true;
}

void RobotServer::controlSystem(){
    m_controlFlag = SWITCHON;
}

SWITCHSTATE RobotServer::getControlState() const{
    return m_controlFlag;
}

void RobotServer::setErrorClear(){
    clearError();
	isFirst = 1;
    setCMDRightFlag(0);
    setCMDFinishFlag(0);
}

bool RobotServer::setJointZero(JOINTINDEX joint){
    setCMDRightFlag(-1);
    if(getServo()){
        printServoOn();
        setCMDRightFlag(1);
        return false;
    }
    else{
        setZero(joint);
        setCMDRightFlag(0);
        return true;
    }
}

double RobotServer::calibrateTCP(const JointsList& js, Terminal& t){
    setCMDRightFlag(-1);

    // 计算并发送结果
    if(js.size() < 3 || js.size() > 10){
        printTCPPointsNumber();
        setCMDRightFlag(1);
        return -1;
    }
    double pre = m_robot.TCPCalibrate(js, t);
    sendTCPF(t, pre);

    setCMDRightFlag(0);
    return pre;
}

double RobotServer::calibrateTCFZ(const JointsList& js, const Joints& jo, const Joints& jz, Terminal& t){
    setCMDRightFlag(-1);

    // 计算并发送结果
    if(js.size() < 3 || js.size() > 10){
        printTCPPointsNumber();
        setCMDRightFlag(1);
        return -1;
    }
    double pre = m_robot.TCPCalibrateZ(js, jo, jz, t);
    sendTCPF(t, pre);


    setCMDRightFlag(0);
    return pre;
}

double RobotServer::calibrateTCFX(const JointsList& js, const Joints& jo, const Joints& jx, const Joints& jz, Terminal& t){
    setCMDRightFlag(-1);

    // 计算并发送结果
    if(js.size() < 3 || js.size() > 10){
        printTCPPointsNumber();
        setCMDRightFlag(1);
        return -1;
    }
    double pre = m_robot.TCPCalibrateXZ(js, jo, jx, jz, t);
    sendTCPF(t, pre);

    setCMDRightFlag(0);
    return pre;
}

void RobotServer::calibrateUSRF(const Terminal& to, const Terminal& tx, const Terminal& ty, Terminal& t){
    setCMDRightFlag(-1);

    // 计算并发送结果
    m_robot.workFrameCalibrate(to, tx, ty, t);
    sendUSRF(t);

    setCMDRightFlag(0);
}


bool RobotServer::programVelocity(double vel){
    m_robotState.setVel(vel);
    return true;
}

double RobotServer::getProgramVelocity()
{
    return m_robotState.getVel();
}

void RobotServer::programLoad(const char* path){
    readRobotProgram(path);
}

void RobotServer::programRun(){
    setRunState(SYSRUN_RUN);
    int debugState = m_robotState.getDebugState();
    if(!m_bRunning)
    {
        m_threadRun.create(runThread,(void*)&debugState);
    }

}

void RobotServer::programPause(bool bPause){
    m_robotState.setRunState( bPause?SYSRUN_PAUSE:SYSRUN_RUN);
    pause(bPause);
}

void RobotServer::programStop(){
    setRunState(SYSRUN_STOP);
    stop();
}

void RobotServer::programStepRun(){
    step();
}

void RobotServer::programStepPause(){
    stepPause();
}

void RobotServer::setProgramPointer(const ProgramPointer &pointer){
    setPointer(pointer);
}

void RobotServer::setBreakPointer(const ProgramPointer &pointer)
{
    setTmpPointer(pointer);
}

void RobotServer::updateProgramPointer(const ProgramPointer &pointer){
    m_programPointer = pointer;
    sendCXXH(pointer);
}

SWITCHSTATE RobotServer::getVisionConnenct(LOCATEVISIONINDEX index){
    return getCameraConnect(index);
}

Terminal RobotServer::getVisionLocation(LOCATEVISIONINDEX index){
    setCMDRightFlag(-1);
    setCMDOtherFlag(-1);    // 更改标志位为等待状态
    sendSJXH(index);    // 发送获取视觉的消息
    waitCMDOtherFlag(); // 等到消息回馈，标志位更改
    sendSJWZ(index, m_visionLocation[index]);
    setCMDRightFlag(0);
    return m_visionLocation[index]; // 返回值
}

void RobotServer::setVisionLocation(LOCATEVISIONINDEX index, const Terminal &t){
    m_visionLocation[index] = t;
    sendSJWZ(index, t);
    setCMDOtherFlag(0);
}

void RobotServer::updateVisionLocation(int index, const Terminal &t){
    m_visionLocation[index] = t;    // 更新位姿信息
    setCMDOtherFlag(0); // 修改标志位为完成状态
}

void RobotServer::updateRobotTerminal(RobotMotion &rm){
    rm.setCurrentTerminal(m_robot.forwardKinematics(rm.getCurrentJointPosition()));
    rm.setCurrentWorkTerminal(rm.getCurrentTerminal().getTerminalInWorkFrame(m_robot.getWorkFrame()));
}

void RobotServer::updateIOState(const RobotIO &rio){
    m_robotIO = rio;
}

void RobotServer::updateMotionState(SWITCHSTATE servo, const RobotMotion &rm){
    m_robotState.setServoState(servo);
}

void RobotServer::updateErrorState(int error, const Joints &jointError){
    m_errorFlag = error;
    m_errorCode = jointError;

    if(m_errorFlag & 0b10000){  // 判断是否触发零位传感器
        m_homeState = SWITCHON;
    }
    else{
        m_homeState = SWITCHOFF;
    }

    if(m_errorFlag & 0b1101){
        setErrorStateLight(SWITCHON);

        if(error & 0b0001&isFirst){
			isFirst = 0;
            printEmergencySwitchOn();
		}
//        if(error & 0x02){
//            printServoOff();
//        }
        if(error & 0b0100){
            printDriverError();
        }
        if(error & 0b1000){
            printJointMoveToLimit();
        }
    }
    else{
		isFirst = 1;
        setErrorStateLight(SWITCHOFF);
    }
}


void RobotServer::setCMDRightFlag(int flag){
    m_cmdRightFlag = flag;
    sendWCIO(flag);
}

int RobotServer::waitCMDRightFlag(){
    while(1){
        if(m_cmdRightFlag >= 0){
            return m_cmdRightFlag;
        }
        sleep_ms(m_robotParameter.getSamplePeriod()*1000);
    }
}

bool RobotServer::isCMDFinished(){
    if(m_cmdFinishFlag >= 0){
        return true;
    }
    return false;
}

void RobotServer::setCMDFinishFlag(int flag){
    m_cmdFinishFlag = flag;
    sendWCZL(flag);
}

int RobotServer::waitCMDFinishFlag(){
    while(1){
        if(m_cmdFinishFlag >= 0){
            return m_cmdFinishFlag;
        }
        sleep_ms(m_robotParameter.getSamplePeriod()*1000);
    }
}

void RobotServer::setCMDOtherFlag(int flag)
{
    m_cmdOtherFlag = flag;
}

int RobotServer::waitCMDOtherFlag()
{
    while(1){
        if(m_cmdOtherFlag >= 0){
            return m_cmdOtherFlag;
        }
        sleep_ms(m_robotParameter.getSamplePeriod()*1000);
    }
}

int RobotServer::judgeMovePrepare(){
    //　判断伺服状态是否打开
    if(!getServo()){
        printServoOff();
        setCMDRightFlag(1);
        return 1;
    }

    // 判断此时是否出错
    if(m_errorFlag & 0x0f){
        setCMDRightFlag(1);
        return 1;
    }

    // 判断是否在运行过程中
    if(!isCMDFinished()){
        printWaitCommandEnd();
        setCMDRightFlag(1);
        return 1;
    }
    setCMDFinishFlag(-1);
    setCMDRightFlag(-1);
    return 0;
}

void RobotServer::updateRobotStateMotionIO(){
    sendGJMD(getRobotMotion());

    sendIOZT(getIOConnect());
    sendSRZT(getDigitalInput());
    sendSCZT(getDigitalOutput());
    sendMNSR(getAnalogInput());
    sendMNSC(getAnalogOutput());

    sendCSYX(m_robotState.getVirtualState());
    sendTSZT(m_robotState.getDebugState());
    sendSFZT(m_robotState.getServoState());
    sendFSZT(m_robotState.getRunState());
    sendYXMS(m_robotState.getPlayState());
    sendCXSD(m_robotState.getVel());

    sendHOME(m_getHomeState);
}

THREAD_ROUTINE_RETURN RobotServer::cycleTaskupdateState(void* lpParameter){
    thread_detach();
    thread_name("RobotServer::cycleTaskupdateState");

    while(1){
        robotServer->updateRobotStateMotionIO();

        if(robotServer->m_robotState.getPlayState() == SYSPLAY_REMOTE){
            if(robotServer->m_remoteState)
                cout << "state: " << robotServer->m_remoteState << "  m_robotState.getRunState(): " << robotServer->m_robotState.getRunState() << endl;
            switch(robotServer->m_remoteState){
            case REMOTE_RUN:
                if(robotServer->m_robotState.getRunState() == SYSRUN_RUN){
                    robotServer->programStop();
                }
                else{
                    robotServer->programRun();
                }
                robotServer->m_remoteState = REMOTE_NONE;
                break;
            case REMOTE_PAUSE:
                if(robotServer->m_robotState.getRunState() == SYSRUN_RUN){
                    robotServer->programPause();
                }
                else if(robotServer->m_robotState.getRunState() == SYSRUN_PAUSE){
                    robotServer->programPause(false);
                }
                robotServer->m_remoteState = REMOTE_NONE;
                break;
            case REMOTE_RETURN:
                if(robotServer->m_robotState.getRunState() == SYSRUN_STOP){
                    robotServer->moveToZero(0.1);
                }
                robotServer->m_remoteState = REMOTE_NONE;
                break;
            default:
                break;
            }
        }

        robotServer->m_timerUpdateState.wait();
    }
}
