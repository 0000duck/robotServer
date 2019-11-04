#ifndef ROBOTINTERPRETER_H
#define ROBOTINTERPRETER_H

#include <map>
#include <string>
#include <deque>
#include <stack>
#include "robotStructure.h"
#include "RobSoft/CErrorCode.hpp"

using namespace std;
namespace rclib {

enum PROGRAM_KEY_WORD{
    VALUE, BOOL, INT, DOUBLE, JOINT, TERMINAL, CALCU,
    MAIN, FUNC, ENDMAIN, ENDFUNC, RETURN, CALL,
    FOR, ENDFOR, WHILE, ENDWHILE, CONTINUE, BREAK,
    IF, ELSEIF, ELSE, ENDIF,
    WAITDIN, DOUT, AOUT,
    DELAY, COM,
    FRAME, TOOLFRAME, USERFRAME,
    MOVEABSJ, MOVEABSJR, MOVEJ, MOVEJR, MOVEL, MOVELR, MOVEC, MOVECR,
    MOVEB, MOVEJT,
    TOOL,WORK,VLOCATE,
    //ABB
    VAR,LOCAL,TASK,NUM,STRING,PERS,
    ORIENT,POS,POSE,CONFDATA,ROBTARGET,SIGNAKXX,TOOLDATA,WOBJDATA
    ,PROC,TRAP
    ,ACCSET, ADD, COMMENT, PULSEDO, SET, SETDO,TEST,VELSET, WAITDI,WAITDO,WAITTIME
};

enum PROGRAM_OPERATOR{
    OPERATOR_ASS,   // =
    OPERATOR_ASS1,  // :=
    OPERATOR_ADD,   // +
    OPERATOR_SUB,   // -
    OPERATOR_MUL,   // *
    OPERATOR_DIV,   // /
    OPERATOR_MOD,   // %
    OPERATOR_EQU,   // ==
    OPERATOR_NEQ,   // !=
    OPERATOR_LAG,   // >
    OPERATOR_LES,   // <
    OPERATOR_GEQ,   // >=
    OPERATOR_LEQ    // <=
};

enum TRUE_OR_FALSE{
    TRUE_T,
    FALSE_F
};
enum ON_OR_OFF{
    ON,
    OFF
};

enum TEACH_TYPE{HR,ABB,OTHER};  //示教器类型

struct ProgramPointer{  // 程序指针
    int nFunction;      //函数序号
    int nSentence;      //语句序号
    ProgramPointer()
        : nFunction(0)
        , nSentence(0)
    {}
};

struct ProgramSentence{ // 程序语句
    std::string strSentence;
    ProgramSentence(){}
    ProgramSentence(std::string str)
        :strSentence(str)
    {}

};

typedef std::vector<double> VECTOR_DOUBLE;
typedef std::vector<std::string> VECTOR_STRING;
typedef std::vector<ProgramSentence> VECTOR_SENTENCE;
struct pointABB{
  string strFunName;    //函数名
  int iNum;             //第n条语句
};
namespace ABBInterpreter {

enum ActionScope{GLOBAL, MODULUS, FUNCTION};    //全局、模块、程序
enum FuncType{FT_PROC,FT_FUNC,FT_TRAP};                  //程序、功能、中断
enum VarAttribute{VA_CONST,VA_VAR,VA_PERS};              //变量属性

typedef struct VariableInfo{
        string strName;             //名字
        string strType;             //类型
        string  strAS;              //作用域
        string strValue;            //变量值
        double doubValue;

        ActionScope  strActionScope;        //作用域
        VECTOR_STRING vecStrValue;          //变量值
        VECTOR_DOUBLE   vecDoubleValue;     //
        string strAttribute;                //变量属性
};
typedef struct VariableInfo VARIABLEINFO;
typedef struct VariableInfo* LP_VARIABLEINFO;

typedef   map<string, LP_VARIABLEINFO>     MAP_VARIABLEINFO;

typedef struct FunctionInfo{
    FunctionInfo();
    ~FunctionInfo();
    string      strName;           //函数名字
    string      strRetType;        //返回类型
    string      strType;           //程序类型
    string      strAS;             //作用域
    PROGRAM_KEY_WORD    pkwRetType;          //返回类型

    MAP_VARIABLEINFO    mapParam;  //参数表
    MAP_VARIABLEINFO    mapVar;     //局部变量
    string      strModuleName;     //模块名
    FuncType    funType;           //程序类型

    VECTOR_SENTENCE  vecSentence;   //语句
    ActionScope actionScope;        //作用域
    int addVariable(string strInfo);   //添加局部变量
	LP_VARIABLEINFO getVariable(string strInfo);   //获取局部变量
    MAP_VARIABLEINFO m_mapVar;        //全局变量
}FUNCTIONINFO,*LP_FUNCTIONINFO;

template<class T>
struct DisableCompare : public std::binary_function<T, T, bool>
{
    bool operator()(T lhs, T rhs) const
    {
        if (lhs == rhs)
            return false;
        return true;
    }
};

typedef   map<string, LP_FUNCTIONINFO, DisableCompare<string >> MAP_FUNCINFO;
MY_CLASS VarManager{
public:
    VarManager() {}
    virtual ~VarManager();

    int addVariable(string strInfo,string strModName);
    MAP_VARIABLEINFO getVariable(ActionScope, string ); //获取模块的所有变量信息
    LP_VARIABLEINFO getVariable(string strName ,LP_FUNCTIONINFO=NULL);       //获取变量信息
    void clear();           //清除容器
private:
    MAP_VARIABLEINFO m_mapGlobleVar;        //全局变量
    map<string, MAP_VARIABLEINFO> m_mapModuleVar;       //模块变量
    map<string, MAP_VARIABLEINFO> m_mapFuncVar;         //程序变量
};

MY_CLASS FunctionManager{
public:
    FunctionManager();
    virtual ~FunctionManager();
    void clear();           //清除容器
    void addFuncHead(string strInfo,LP_FUNCTIONINFO pFuncInfo);     //添加程序头
    string getOneSentence(struct pointABB& pPoint);                 //获取一条语句

    void getFunctionTable(vector<VECTOR_SENTENCE>&, std::map<std::string, int>&);   //获取函数表和索引
    LP_FUNCTIONINFO getFuncinfo(string strName , int index=0);		//0：程序，1：功能函数，2：中断
private:
    void addFunc(string,LP_FUNCTIONINFO);   //添加程序模块
    void initParam();
private:
    MAP_FUNCINFO m_mapFuncIndex;       //功能
    MAP_FUNCINFO m_mapProcIndex;       //程序
    MAP_FUNCINFO m_mapTrapIndex;       //终端
    std::map<std::string, PROGRAM_KEY_WORD>     m_programKeyWords;  //关键字索引
};


}

struct ForCount
{
    int nTotal;
    int nCompletedTime;

    int nBegin;     //开始
    int nEnd;       //结束
    int nStep;      //步长
    int nWalker;    //行者
    bool    step()
    {
        nWalker += nStep;
        return nWalker > nEnd;
    }
    ForCount()
        : nTotal(0)
        , nCompletedTime(0)
        , nStep(1)
        , nBegin(0)
        , nEnd(0)
    {
        nWalker = nBegin;
    }
};

MY_CLASS RobotInterpreter{
public:
    RobotInterpreter();
    ~RobotInterpreter();

    void setDebugState(SWITCHSTATE state);  // 设置程序调试状态
    void setPointer(const ProgramPointer& pointer);    // 设置程序指针
    void setTmpPointer(const ProgramPointer& pointer);    // 设置程序指针
#ifdef __linux__
    static THREAD_ROUTINE_RETURN runThread(void* debugState);    //运行程序
#else
    static THREAD_ROUTINE_RETURN (__stdcall runThread)(void* debugState);
#endif
    void run(); // 非调试模式下运行程序
    void pause(bool bPause = true);   // 程序暂停
    void stop(); // 程序停止
    void stepRun(); // 调试模式下运行程序
    void step();    // 调试模式下单步运行程序
    void stepPause();   // 调试模式下暂停程序
    bool IsStop();

    void readRobotProgram(const char* path);    // 读取程序
    void writeRobotProgram(const char* path);   // 写入程序

    int readFunction(const char* path);         // 读取子函数
    int writeFunction(const char* path, std::string funcName);  // 写入子函数
    int deleteFunction(std::string funcName);   // 删除子函数

    int addProgramSentence(const ProgramPointer& pointer, const ProgramSentence& sen);  // 添加程序
    int modifyProgramSentence(const ProgramPointer& pointer, ProgramSentence& sen);     // 修改程序
    int deleteProgramSentence(const ProgramPointer& pointer);           // 删除程序
    ProgramSentence getProgramSentence(const ProgramPointer& pointer);  // 获取程序语句

    std::vector<std::vector<ProgramSentence>>  getFuncTable() const;     //存放函数
    int addVariable(std::string strVariable);           //添加变量
    int delVariable(std::string strName);                                   //删除
    int modifyVariable(std::string strVarName,std::string strVariable);     //修改
    int getVariable(std::string strVarName,PROGRAM_KEY_WORD& type,VECTOR_DOUBLE& vecVal);  //获取变量类型、值
    std::map<std::string, PROGRAM_KEY_WORD> getVariableMap() const;
    void clearFuncTable();          //清除函数表
    void clearVarTable();           //清除变量表

    std::map<std::string, PROGRAM_KEY_WORD> getKeyWordIndex();       //获取关键字索引
    std::map<std::string, PROGRAM_OPERATOR> getOpertorIndex();       //获取运算符索引
    std::map<std::string, int>              getFuncIndex();          //获取函数索引

    void setTeach(TEACH_TYPE type); //设置示教器的类型
    //设置错误信息的语言类型，0为英文，1为中文
    void setErrorInfoLanguage(ERROR_INIF_LANGUAGE type);

    Thread m_threadRun;     //程序线程
    bool m_bRunning;         //程序线程正在运行
    static RobotInterpreter* m_robotInterpreter;
private:
    void initParameter();   // 初始化参数
        
    //add by lf
    int executeOneSentence(const ProgramSentence& sen);     //执行一条语句
    ProgramSentence getOneProgramSentence(const ProgramPointer& pointer);  // 获取程序语句
    void setFlag();

    bool CtreateBivariateTable(const std::string strBeginKey,const std::string strEndKey,std::vector<std::vector<int>>&);
    int handleFor();        //处理for循环
    int handleWhile();      //处理while循环
    int handleIf();         //处理if语句
    int handleCall();       //处理call语句
    int handleOperation(const VECTOR_STRING&);  //处理运算语句
    bool formulaComputing(std::string strFormula);

    int arithmometer(const VECTOR_STRING&,PROGRAM_OPERATOR opt=OPERATOR_ASS);  //四则运算
    bool isPort(std::string strPortName,double fValue);
    bool bCompare(double Value1,double Value2,PROGRAM_OPERATOR opt=OPERATOR_ASS);   //比较判断
#ifndef QT_NO_DEBUG
public:
    void testSet();
#endif
    int grammarCheck(std::string strSentence);          //语法检查

private:
    SWITCHSTATE m_debugState;   // 当前是否处于调试模式下
    ProgramPointer m_Pointer;    // 当前运行的程序序号
    ProgramPointer m_tmpPointer;    // 调试模式下暂停时的程序序号

    std::string s_toolFrame;    // 程序运行前的工具坐标系
    std::string s_userFrame;    // 程序运行前的工件坐标系
    
    std::vector<std::vector<ProgramSentence>>   m_vecFuncTable;     //存放函数
    std::map<std::string, int>              m_mapFuncIndex;         //函数索引

    std::map<int,std::string>   m_mapErrorInfo;             //错误信息
    std::stack<ProgramPointer>  m_funPointerStack;          //函数指针栈
    std::map <std::string, PORTINDEX> m_mapPortIndex;         //端口索引

    std::map<std::string, PROGRAM_KEY_WORD>     m_programKeyWords;  //关键字索引
    std::map<std::string, PROGRAM_OPERATOR>     m_programOperator;  //运算符索引
    std::map<std::string, PROGRAM_KEY_WORD>     m_mapVarType;  //变量类型
    std::map<std::string, std::vector<double>>  m_mapVarValue;      //变量值
public:
    bool m_bPause;      //暂停
    bool m_bStop;       //停止
    bool m_bSingle;     //单步
    bool m_bConStartFlag;       //联动标记

    TEACH_TYPE m_teachType;         //示教器的类型
    ERROR_INIF_LANGUAGE m_language;   //语言种类

///////ABB////////////
public:
    void runABB();
    int readFunctionABB(const char* path);			// 读取子函数
    int ABB2HR(string &strSentence);
private:
    ABBInterpreter::VarManager          m_VarMan;   //变量管理
    ABBInterpreter::FunctionManager     m_FunMan;   //程序管理
    void InitParamABB();
    string getOneSentence(struct pointABB& pPoint);
    int executeOneSentenceABB(const ProgramSentence& sen);     //执行一条ABB语句

    int handleForABB();        //处理for循环
    int handleWhileABB();      //处理while循环
    int handleIfABB();         //处理if语句
    int handleCallABB(vector<string>& vecField);		//处理call语句
    int handleOperationABB(string );					//处理指令语句
    bool formulaComputingABB(string strFormula);		//条件判断
    int grammarCheck(vector<string>& vecField);			//语法检查

	bool IsProcCall(VECTOR_STRING& vecField);
	int  arithmometerABB(const VECTOR_STRING&);			//:=赋值计算
														
	//将含变量的表达式转换成无变量表达式
	bool makeExpress(const VECTOR_STRING& vecSour, string& strDest);
	bool IsJudgeOperator(string& str, PROGRAM_OPERATOR& op);
private:
    jsonWrapper m_jsWrapper;        //json接口
    ABBInterpreter::LP_FUNCTIONINFO m_pCurrFuninfo ;//当前函数指针；
///////END ABB//////////////

private:
    virtual void programStop(){}    //程序运行停止
    virtual void updateProgramPointer(const ProgramPointer &pointer){}  // 传递程序当前运行的指针
    string  makePrintPrefix( ProgramPointer* pointer =NULL);
    virtual void printInfo(INFOSTATE infoType, std::string infoString){}

    virtual int moveABSJoint(const Joints& ps, double vel){return 0;}
	virtual int moveABSJoint(const JointsList& ps, double vel) { return 0; } // 关节运动
    virtual int moveABSJoint(const JointsList& ps, std::vector<double>& tm){ return 0; }    // 关节运动
    virtual int moveABSJointR(const JointsList &ps, double vel){ return 0; }    // 相对关节运动

    virtual int moveJoint(const Terminal& p, double vel, COORDINATESYSTEM frame = COORDINATE_BASE){return 0; }
    virtual int moveJoint(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }    // 以末端点为目标的关节运动
    virtual int moveJointR(const TerminalList& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }   // 以末端点为目标的相对关节运动

    virtual int moveLine(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }    // 直线运动
    virtual int moveLineR(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }   //　相对直线运动
    virtual int moveCircle(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; } //　圆弧运动
    virtual int moveCircleR(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }    // 相对圆弧运动

    virtual int moveCurve(const TerminalList& ps, std::vector<double>& vel, double acc = 0.8, double jerk = 0.8, double angle = 0.8, double bpre = 0.01, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; } // 样条曲线运动

    virtual int moveLineCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }  // 连续轨迹直线运动
    virtual int moveLineRCon(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; } // 连续轨迹相对直线运动
    virtual int moveCircleCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }   // 连续轨迹圆弧运动
    virtual int moveCircleRCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE){ return 0; }  // 连续轨迹相对圆弧运动
    virtual int moveStartCon(int delay = 2){ return 0; }   // 连续轨迹运动开始
    virtual int moveEndCon(){ return 0; }  // 连续轨迹运动结束

    virtual int waitCommandEnd(){ return 0; }  // 等待命令执行完成

	virtual int modifyToolFrame(std::string name) { return 0; }    // 修改工具坐标系
    virtual int modifyWorkFrame(std::string name){ return 0; }    // 修改工件坐标系

    virtual SWITCHSTATE getIOConnect() const{ return SWITCHOFF; }    // 是否连接了ＩＯ模块
    virtual void setDigitalOutput(PORTINDEX index, SWITCHSTATE state){}    //　设置数字输出状态
    virtual void setAnalogOutput(PORTINDEX index, double state){}  // 设置模拟输出状态
    virtual SWITCHSTATE getDigitalInput(PORTINDEX index) const{ return SWITCHOFF; } // 获取数字输入状态
    virtual SWITCHSTATE getDigitalOutput(PORTINDEX index) const{ return SWITCHOFF; }    // 获取数字输出状态
    virtual double getAnalogInput(PORTINDEX index) const{ return 0.0; }   // 获取模拟输入状态
    virtual double getAnalogOutput(PORTINDEX index) const{ return 0.0; }  // 获取模拟输出状态

    virtual SWITCHSTATE getVisionConnenct(LOCATEVISIONINDEX index){ return SWITCHOFF; }   // 获取视觉设备连接状态
	virtual Terminal getVisionLocation(LOCATEVISIONINDEX index) {return Terminal();}  // 获取视觉定位

    virtual double getProgramVelocity(){return  0;}        //获取速度比
protected:
    Terminal m_visionLocation[3];   // 视觉位姿记录
};

}

#endif
