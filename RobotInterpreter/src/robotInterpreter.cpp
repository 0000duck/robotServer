#include "robotInterpreter.h"
#include "robotFile.h"
#include "functions.h"
#include <sstream>
#include <string.h>
//#include <unistd.h>

#define PROGRAMFILE "./programFile"

using namespace std;
using namespace rclib;
using namespace Common;


RobotInterpreter* RobotInterpreter::m_robotInterpreter;

RobotInterpreter::RobotInterpreter()
            : m_bPause(false)
            , m_bSingle(false)
            , m_bStop(false)
            , m_teachType(HR)
            , m_bRunning(false)
{
    m_robotInterpreter = this;
    initParameter();
}

RobotInterpreter::~RobotInterpreter(){
    m_threadRun.cancel();
    m_threadRun.join();
}

void RobotInterpreter::setDebugState(SWITCHSTATE state){
    m_debugState = state;
}

THREAD_ROUTINE_RETURN RobotInterpreter::runThread(void* debugState)
{
    thread_detach();
#ifdef __linux__
    thread_name("RobotInterpreter::runThread");
#endif
    int* state = (int*)debugState;

    m_robotInterpreter->m_bRunning = true;
    if(m_robotInterpreter->m_teachType==ABB)
    {
        m_robotInterpreter->runABB();
    }else {
        if(*state)
        {
            m_robotInterpreter->stepRun();
        }else {
            m_robotInterpreter->run();
        }
    }

    if (m_robotInterpreter->m_bConStartFlag)
    {
        m_robotInterpreter->m_bConStartFlag = false;
        m_robotInterpreter->moveEndCon();
        m_robotInterpreter->waitCommandEnd();
    }
    m_robotInterpreter->programStop();
    m_robotInterpreter->m_bRunning = false;

    thread_exit();
	return 0;
}

void RobotInterpreter::setPointer(const ProgramPointer& pointer){
    m_Pointer.nFunction = pointer.nFunction;
    m_Pointer.nSentence = pointer.nSentence;
}

void RobotInterpreter::setTmpPointer(const ProgramPointer &pointer)
{
    m_tmpPointer = pointer;
}

void RobotInterpreter::initParameter(){
    m_programKeyWords["VALUE"] = VALUE;
    m_programKeyWords["BOOL"] = BOOL;
    m_programKeyWords["INT"] = INT;
    m_programKeyWords["DOUBLE"] = DOUBLE;
    m_programKeyWords["JOINT"] = JOINT;
    m_programKeyWords["TERMINAL"] = TERMINAL;
    m_programKeyWords["CALCU"] = CALCU;

    m_programKeyWords["MAIN"] = MAIN;
    m_programKeyWords["FUNC"] = FUNC;
    m_programKeyWords["ENDMAIN"] = ENDMAIN;
    m_programKeyWords["ENDFUNC"] = ENDFUNC;
    m_programKeyWords["RETURN"] = RETURN;
    m_programKeyWords["CALL"] = CALL;

    m_programKeyWords["FOR"] = FOR;
    m_programKeyWords["ENDFOR"] = ENDFOR;
    m_programKeyWords["WHILE"] = WHILE;
    m_programKeyWords["ENDWHILE"] = ENDWHILE;
    m_programKeyWords["CONTINUE"] = CONTINUE;
    m_programKeyWords["BREAK"] = BREAK;

    m_programKeyWords["IF"] = IF;
    m_programKeyWords["ELSEIF"] = ELSEIF;
    m_programKeyWords["ELSE"] = ELSE;
    m_programKeyWords["ENDIF"] = ENDIF;

    m_programKeyWords["WAITDIN"] = WAITDIN;
    m_programKeyWords["DOUT"] = DOUT;
    m_programKeyWords["AOUT"] = AOUT;

    m_programKeyWords["DELAY"] = DELAY;
    m_programKeyWords["COM"] = COM;

    m_programKeyWords["FRAME"] = FRAME;
    m_programKeyWords["TOOLFRAME"] = TOOLFRAME;
    m_programKeyWords["USERFRAME"] = USERFRAME;

    m_programKeyWords["MOVABSJ"] = MOVEABSJ;
    m_programKeyWords["MOVABSJR"] = MOVEABSJR;
    m_programKeyWords["MOVJ"] = MOVEJ;
    m_programKeyWords["MOVJR"] = MOVEJR;

    m_programKeyWords["MOVL"] = MOVEL;
    m_programKeyWords["MOVLR"] = MOVELR;
    m_programKeyWords["MOVC"] = MOVEC;
    m_programKeyWords["MOVCR"] = MOVECR;

    m_programKeyWords["MOVB"] = MOVEB;
    m_programKeyWords["MOVJT"] = MOVEJT;

    m_programKeyWords["VLOCATE"] = VLOCATE;

    m_programKeyWords["TOOL"] = TOOL;
    m_programKeyWords["WORK"] = WORK;

    //m_programKeyWords["TRUE"] = TRUE;
    //m_programKeyWords["FALSE"] = FALSE;
    //m_programKeyWords["ON"] = ON;
    //m_programKeyWords["OFF"] = OFF;

    m_programOperator["="] = OPERATOR_ASS;
    m_programOperator["+"] = OPERATOR_ADD;
    m_programOperator["-"] = OPERATOR_SUB;
    m_programOperator["*"] = OPERATOR_MUL;
    m_programOperator["/"] = OPERATOR_DIV;
    m_programOperator["%"] = OPERATOR_MOD;
    m_programOperator["=="] = OPERATOR_EQU;
    m_programOperator["!="] = OPERATOR_NEQ;
    m_programOperator[">"] = OPERATOR_LAG;
    m_programOperator["<"] = OPERATOR_LES;
    m_programOperator[">="] = OPERATOR_GEQ;
    m_programOperator["<="] = OPERATOR_LEQ;

    m_mapPortIndex["DI1"] = PORT_1;
    m_mapPortIndex["DI2"] = PORT_2;
    m_mapPortIndex["DI3"] = PORT_3;
    m_mapPortIndex["DI4"] = PORT_4;
    m_mapPortIndex["DI5"] = PORT_5;
    m_mapPortIndex["DI6"] = PORT_6;
    m_mapPortIndex["DI7"] = PORT_7;
    m_mapPortIndex["DI8"] = PORT_8;
    m_mapPortIndex["DI9"] = PORT_9;
    m_mapPortIndex["DI10"] = PORT_10;
    m_mapPortIndex["DI11"] = PORT_11;
    m_mapPortIndex["DI12"] = PORT_12;
    m_mapPortIndex["DI13"] = PORT_13;
    m_mapPortIndex["DI14"] = PORT_14;
    m_mapPortIndex["DI15"] = PORT_15;
    m_mapPortIndex["DI16"] = PORT_16;
    m_mapPortIndex["DO1"] = PORT_1;
    m_mapPortIndex["DO2"] = PORT_2;
    m_mapPortIndex["DO3"] = PORT_3;
    m_mapPortIndex["DO4"] = PORT_4;
    m_mapPortIndex["DO5"] = PORT_5;
    m_mapPortIndex["DO6"] = PORT_6;
    m_mapPortIndex["DO7"] = PORT_7;
    m_mapPortIndex["DO8"] = PORT_8;
    m_mapPortIndex["DO9"] = PORT_9;
    m_mapPortIndex["DO10"] = PORT_10;
    m_mapPortIndex["DO11"] = PORT_11;
    m_mapPortIndex["DO12"] = PORT_12;
    m_mapPortIndex["DO13"] = PORT_13;
    m_mapPortIndex["DO14"] = PORT_14;
    m_mapPortIndex["DO15"] = PORT_15;
    m_mapPortIndex["DO16"] = PORT_16;

    m_mapPortIndex["AI1"] = PORT_1;
    m_mapPortIndex["AI2"] = PORT_2;
    m_mapPortIndex["AI3"] = PORT_3;
    m_mapPortIndex["AI4"] = PORT_4;
    m_mapPortIndex["AO1"] = PORT_1;
    m_mapPortIndex["AO2"] = PORT_2;
    m_mapPortIndex["AO3"] = PORT_3;
    m_mapPortIndex["AO4"] = PORT_4;

    setErrorInfoLanguage(ENGLISH);

    InitParamABB();
}

void RobotInterpreter::setErrorInfoLanguage(ERROR_INIF_LANGUAGE type)
{
    m_language = type;
    if(type == ENGLISH)
    {
        m_mapErrorInfo[ERROR_TRUE] = "TRUE";
        m_mapErrorInfo[ERROR_OVERRANGE] = "OVER RANGE";
        m_mapErrorInfo[ERROR_OVERVELOCITY] = "OVER VELOCITY";
        m_mapErrorInfo[ERROR_OVERACCELERATION] = "OVER ACCELERATION";
        m_mapErrorInfo[ERROR_UNDEFINE_VARIABLE] = "Undefined variable types are used";
        m_mapErrorInfo[ERROR_VARIABLE_ASSIGNMENT] = "Variable assignment error";
        m_mapErrorInfo[ERROR_POINTER_ACCESS_VIOLATION] = "Function pointer access is out of bounds";
        m_mapErrorInfo[ERROR_SYNTAX_ERROR] = "Syntax error!";
        m_mapErrorInfo[ERROR_OPERATION_ASSIGNMENT] = "The operation or assignment of variables between different types";
        m_mapErrorInfo[ERROR_NULL_SENTENCE] = "NULL Sentence";
        m_mapErrorInfo[ERROR_VARIABLE_REDEFINITION] = "Variable definition error";
        m_mapErrorInfo[ERROR_VARIABLE_REDEFINITION] = "Variable redefinition";
        m_mapErrorInfo[ERROR_FUNCTION_REDEFINITION] = "Function redefinition";
        m_mapErrorInfo[ERROR_INVALID_KEYWORD] = "Invalid keyword";
        m_mapErrorInfo[ERROR_CAMEL_CONNECT] = "The camera is not connected or does not exist";
        m_mapErrorInfo[ERROR_IO_CONNECT] = "IO no connect";
        m_mapErrorInfo[ERROR_NO_FUNCTION] = "The function doesn't exist";
		m_mapErrorInfo[ERROR_NO_MATCH_FUNCTION] = "No matching function was found";
    }
    else
    {
        m_mapErrorInfo[ERROR_TRUE] = "TRUE";
        m_mapErrorInfo[ERROR_UNDEFINE_VARIABLE] = "使用未定义的变量类型";
        m_mapErrorInfo[ERROR_VARIABLE_ASSIGNMENT] = "变量赋值错误";
        m_mapErrorInfo[ERROR_POINTER_ACCESS_VIOLATION] = "函数指针访问越界";
        m_mapErrorInfo[ERROR_SYNTAX_ERROR] = "语法错误!";
        m_mapErrorInfo[ERROR_OPERATION_ASSIGNMENT] = "不同变量间的运算或赋值";
        m_mapErrorInfo[ERROR_NULL_SENTENCE] = "空语句";
        m_mapErrorInfo[ERROR_VARIABLE_DEFINITION] = "变量定义错误";
        m_mapErrorInfo[ERROR_VARIABLE_REDEFINITION] = "变量重复定义";
        m_mapErrorInfo[ERROR_FUNCTION_REDEFINITION] = "函数重复定义";
        m_mapErrorInfo[ERROR_INVALID_KEYWORD] = "无效的关键字";
        m_mapErrorInfo[ERROR_CAMEL_CONNECT] = "相机未连接或不存在";
        m_mapErrorInfo[ERROR_CAMEL_CONNECT] = "IO 未连接";/**/
    }
}

int RobotInterpreter::executeOneSentence(const ProgramSentence &sen)
{
    if(m_teachType==ABB)
    {
        return executeOneSentenceABB(sen);
    }
    vector<string> vecField = Common::ToVectors(sen.strSentence, " ");
    int nRt = ERROR_TRUE;
    if(vecField.size()>0)
    {
        switch (m_programKeyWords[vecField[0]]) {
        case FOR:
        {
            nRt = handleFor();
            break;
        }
        case WHILE:
        {
            nRt = handleWhile();
            break;
        }
        case IF:
        {
            nRt = handleIf();
            break;
        }
        case CALL:
        {
            nRt = handleCall();
            break;
        }
        default:
        {
            nRt = handleOperation(vecField);
            break;
        }
        }
    }
    else
    {
        printInfo(INFO_WARNING, "代码解析错误");
    }
    if(nRt > ERROR_OVERACCELERATION)
    {
        printInfo(INFO_WARNING, makePrintPrefix()+m_mapErrorInfo[nRt]);
    }
    return nRt;
}

bool RobotInterpreter::CtreateBivariateTable(const string strBeginKey, const string strEndKey, std::vector<vector<int>> &vecT)
{
    const vector<ProgramSentence>& vecSentence = m_vecFuncTable[m_Pointer.nFunction] ;
    int nSentenceSerial = m_Pointer.nSentence;

    // 生成二维表
    for(int i = nSentenceSerial; i<vecSentence.size(); i++)
    {
        vector<string> vecField = Common::ToVectors(vecSentence[i].strSentence," ");
        if(vecField.size()>0)
        {
            if(vecField[0]==strBeginKey)
            {
                vector<int> vecSerial;
                vecSerial.push_back(i);
                vecT.push_back(vecSerial);
            }

            if(vecField[0] == strEndKey)
            {
                //vector<vector<int>>::reverse_iterator iter = vecFor.rbegin();
                for(auto iter = vecT.rbegin();iter!= vecT.rend();iter++)
                {
                    if( (*iter).size()<2)
                    {
                        (*iter).push_back(i);
                        if((iter+1) == vecT.rend())
                        {
                            //二维表生成完成
                            i = vecSentence.size();
                        }
                        break;
                    }

                }
            }
        }
        else
        {
            printInfo(INFO_WARNING, m_mapErrorInfo[ERROR_SYNTAX_ERROR]);
        }

    }

    //检查二维表正确性
    for(auto iter = vecT.begin();iter!= vecT.end();iter++)
    {
        if((*iter).size() != 2)
        {
            return ERROR_SYNTAX_ERROR;
        }
    }

    return ERROR_TRUE;
}

int RobotInterpreter::handleFor()
{
    const vector<ProgramSentence>& vecSentence = m_vecFuncTable[m_Pointer.nFunction] ;
    int nSentenceSerial = m_Pointer.nSentence;

    vector<vector<int>> vecFor;      //存放for循环体的二维表
    // 生成二维表
    for(int i = nSentenceSerial; i<vecSentence.size(); i++)
    {
        vector<string> vecField = Common::ToVectors(vecSentence[i].strSentence," ");
        if(vecField.size()>0)
        {
            if(vecField.size()==2 && vecField[0]=="FOR")
            {
                vector<int> vecSerial;
                vecSerial.push_back(i);
                vecFor.push_back(vecSerial);
            }

            if(vecField[0] == "ENDFOR")
            {
                //vector<vector<int>>::reverse_iterator iter = vecFor.rbegin();
                for(auto iter = vecFor.rbegin();iter!= vecFor.rend();iter++)
                {
                    if( (*iter).size()<2)
                    {
                        (*iter).push_back(i);
                        if((iter+1) == vecFor.rend())
                        {
                            //二维表生成完成
                            i = vecSentence.size();
                        }
                        break;
                    }
                }
            }
        }
        else
        {
            printInfo(INFO_WARNING, m_mapErrorInfo[ERROR_SYNTAX_ERROR]);
        }

    }

    //检查二维表正确性
    for(auto iter = vecFor.begin();iter!= vecFor.end();iter++)
    {
        if((*iter).size() != 2)
        {
            return ERROR_SYNTAX_ERROR;
        }
    }

    stack<ForCount> stackFor;           //记录for的执行
    //执行for循环 到for的下一行
    //for(int i = nSentenceSerial; i<vecFor[0][1]+1/*vecSentence.size()*/; i++,m_Pointer.nSentence=i)
    while(m_Pointer.nSentence < vecFor[0][1]+1)
    {
        int& i = m_Pointer.nSentence;
        ProgramSentence oneSentence = getOneProgramSentence(m_Pointer);
        if(m_bStop)
        {
            break;
        }
        vector<string> vecField = Common::ToVectors(oneSentence.strSentence," ");
        if(vecField.size()>0)
        {
            if(vecField.size()==2 && vecField[0]=="FOR")
            {
                ForCount forCount;
                forCount.nTotal = atoi(vecField[1].c_str());
                if(forCount.nTotal>0)
                {
                    stackFor.push(forCount);
                    i++;
                }
                else
                {
                    vector<vector<int>>::iterator iter = vecFor.begin();
                    for(;iter!= vecFor.end();iter++)
                    {
                        vector<int> vecMap = *iter;
                        if(vecMap[0] == i)
                        {
                            i= vecMap[1] + 1;   //跳转endfor的下一行
                        }
                    }
                }
            }else if(vecField[0]=="ENDFOR")
            {
                ForCount& forTop = stackFor.top();
                forTop.nCompletedTime++;
                if(forTop.nCompletedTime>=forTop.nTotal)
                {
                    stackFor.pop();
                    //i++到endfor的下一行
                    i++;
                }
                else
                {
                    vector<vector<int>>::iterator iter = vecFor.begin();
                    for(;iter!= vecFor.end();iter++)
                    {
                        vector<int> vecMap = *iter;
                        if(vecMap[1] == i)
                        {
                            i= vecMap[0]+1;   //跳转for,i++到下一行
                        }
                    }
                }
            }else if(vecField[0]=="BREAK"||vecField[0]=="CONTINUE")
            {
                int nTmp = i;
                //查找所属数对
                vector<vector<int>>::iterator iter = vecFor.begin();
                for(;iter!= vecFor.end();iter++)
                {
                    vector<int> vecMap = *iter;
                    //找到该位置的下一个endfor,
                    if(i > vecMap[0]&&i < vecMap[1])
                    {
                        if(vecField[0]=="BREAK")
                        {
                            nTmp = vecMap[1];   //跳转endfor的下一行
                        }
                        else
                        {
                            //跳转for,i++到下一行
                            ForCount& forTop = stackFor.top();
                            if(forTop.nCompletedTime + 1>=forTop.nTotal)
                            {
                                //i++到endfor的下一行
                                nTmp = vecMap[1];
                            }
                            else
                            {
                                nTmp = vecMap[0];   //跳转for,i++到下一行
                            }
                        }
                    }
                }
                i = nTmp+1;
                if(vecField[0]=="BREAK")
                {
                    //跳转endfor的下一行
                    stackFor.pop();
                }
                else
                {
                    ForCount& forTop = stackFor.top();
                    forTop.nCompletedTime++;
                    if(forTop.nCompletedTime>=forTop.nTotal)
                    {
                        stackFor.pop();
                        //i++到endfor的下一行
                    }
                }
            }
            else
            {
#ifndef QT_NO_DEBUG
            cout << "HadleFor:" << vecSentence[i].strSentence<<endl;
#endif
                //cout << "for:"<<vecSentence[i].strSentence<<std::endl;
                executeOneSentence(vecSentence[i]);
            }
        }
        else
        {
            printInfo(INFO_WARNING, m_mapErrorInfo[ERROR_SYNTAX_ERROR]);
        }
    }
    return 0;
}

int RobotInterpreter::handleWhile()
{
    int nRt;
    const vector<ProgramSentence>& vecSentence = m_vecFuncTable[m_Pointer.nFunction] ;
    int nSentenceSerial = m_Pointer.nSentence;

    vector<vector<int>> vecWhile;
    if((nRt=CtreateBivariateTable("WHILE","ENDWHILE", vecWhile))!= ERROR_TRUE)
    {
        printInfo(INFO_WARNING,m_mapErrorInfo[ERROR_SYNTAX_ERROR]);
        return nRt;
    }

    //执行while循环 到endwhle的下一行
    //while(m_Pointer.nSentence<vecSentence.size())
    while(m_Pointer.nSentence<=vecWhile[0][1])
    {
        ProgramSentence oneSentence = getOneProgramSentence(m_Pointer);
        if(m_bStop)
        {
            break;
        }
        vector<string> vecField = Common::ToVectors(oneSentence.strSentence," ");
        if(vecField.size()>0)
        {
            if (vecField[0]=="WHILE")
            {
                //判断
                if(formulaComputing(oneSentence.strSentence))
                {
                    m_Pointer.nSentence++;
                    continue;
                }else
                {
                   for(auto iter = vecWhile.begin();iter!=vecWhile.end();iter++)
                   {
                       vector<int> vecMap = *iter;
                       if(vecMap[0] == m_Pointer.nSentence)
                       {
                           m_Pointer.nSentence= vecMap[1]+1;   //跳转endwhile的下一行
                           break;
                       }
                   }
                }
            }
            else if(vecField[0]=="ENDWHILE")
            {
                for(auto iter = vecWhile.begin();iter!=vecWhile.end();iter++)
                {
                    vector<int> vecMap = *iter;
                    if(vecMap[1] == m_Pointer.nSentence)
                    {
                        m_Pointer.nSentence= vecMap[0];   //跳转while
                        break;
                    }
                }
            }else if(vecField[0]=="BREAK"||vecField[0]=="CONTINUE")
            {
                int nTmp = m_Pointer.nSentence;
                vector<vector<int>>::iterator iter = vecWhile.begin();
                for(;iter!= vecWhile.end();iter++)
                {
                    vector<int> vecMap = *iter;
                    //找到该位置的下一个endwhile,
                    if(m_Pointer.nSentence>vecMap[0]&&m_Pointer.nSentence<vecMap[1])
                    {
                        if(vecField[0]=="BREAK")
                        {
                            nTmp = vecMap[1]+1;   //跳转endwhile的下一行
                        }
                        else
                        {
                            //跳转,i++到while
                            nTmp = vecMap[0] ;
                        }
                    }
                }
                m_Pointer.nSentence = nTmp;
            }else if(vecField[0]=="ENDFUNC")
            {
                break;
            }
            else
            {
#ifndef QT_NO_DEBUG
                cout << "Handle while:"<<oneSentence.strSentence << endl;
#endif
                executeOneSentence(oneSentence);
                //m_Pointer.nSentence--;
            }
        }
        else
        {
            m_Pointer.nSentence++;
        }
        //m_Pointer.nSentence++;
    }//while(m_Pointer.nSentence<vecSentence.size());

    return ERROR_TRUE;
}

int RobotInterpreter::handleIf()
{
    int nRt;
    const vector<ProgramSentence>& vecSentence = m_vecFuncTable[m_Pointer.nFunction] ;
    int nSentenceSerial = m_Pointer.nSentence;

    vector<vector<int>> vecIF;
    vector<int> vecIFAUX;
    for(int i=nSentenceSerial; i<vecSentence.size(); i++)
    {
        vector<string> vecField = Common::ToVectors(vecSentence[i].strSentence, " ");
        if(vecField.size()>0)
        {
            if(vecField[0]== "IF")
            {
                vector<int> vecSerial;
                vecSerial.push_back(i);
                vecIF.push_back(vecSerial);
                vecIFAUX.push_back(0);
            }
            else if(vecField[0]== "ELSEIF" ||vecField[0]== "ELSE" )
            {
                for(int index=vecIFAUX.size()-1; index>=0; index--)
                {
                    if(vecIFAUX[index]==0)
                    {
                        vecIF[index].push_back(i);
                        break;
                    }
                }
            }
            else if(vecField[0]== "ENDIF")
            {
                for(int index=vecIFAUX.size()-1; index>=0; index--)
                {
                    if(vecIFAUX[index]==0)
                    {
                        vecIFAUX[index]=1;
                        vecIF[index].push_back(i);
                        break;
                    }
                }
            }
        }
    }

    stack<bool> stackIf;
    //执行if
    //for(int i=nSentenceSerial; i<vecSentence.size(); i++,m_Pointer.nSentence=i)
    //while(m_Pointer.nSentence< vecSentence.size())
	while (m_Pointer.nSentence <= *(vecIF[0].rbegin()))
    {
        ProgramSentence oneSentence = getOneProgramSentence(m_Pointer);
        if(m_bStop)
        {   break; }
        vector<string> vecField = Common::ToVectors(oneSentence.strSentence, " ");
        if(vecField.size()>0)
        {
            int iNextIfSentence;
            if(vecField[0]=="IF" || vecField[0]=="ELSEIF"||vecField[0]=="ELSE")
            {
                //查表到下条if系列语句
                for(vector<vector<int>>::iterator iter = vecIF.begin(); iter!=vecIF.end();iter++)
                {
                    vector<int> vecSerial = *iter;
                    for(vector<int>::size_type index=0; index < vecSerial.size();index++)
                    {
                        if(vecSerial[index]==m_Pointer.nSentence&&index<vecSerial.size()-1)
                        {
                            iNextIfSentence = vecSerial[index+1];
                        }
                    }
                }
            }
            if(vecField[0]=="IF")
            {
                //判断
                if(formulaComputing(oneSentence.strSentence))
                {
                    stackIf.push(true);
                    m_Pointer.nSentence++;
                }else
                {
                    stackIf.push(false);
                    //查表到下条if语句
                   m_Pointer.nSentence = iNextIfSentence ;
                }
            }else if(vecField[0]=="ELSEIF" ||vecField[0]=="ELSE")
            {
                bool& isTrue = stackIf.top();
                //条件判断
                if(!isTrue &&formulaComputing(oneSentence.strSentence))
                {
                    isTrue=true;
                    m_Pointer.nSentence++;
                }else
                {
                    m_Pointer.nSentence = iNextIfSentence ;
                }
            }else if(vecField[0]=="ENDIF")
            {
                stackIf.pop();
                if(stackIf.empty())
                {
                   break;
                }
                m_Pointer.nSentence++;
            }
			else if (vecField[0] == "BREAK" || vecField[0] == "CONTINEU")
			{
				return 0;
			}
            else
            {
#ifndef QT_NO_DEBUG
                cout << "handle IF:" << oneSentence.strSentence << endl;
#endif
                executeOneSentence(oneSentence);
            }
        }
    }
    return 0;
}

int RobotInterpreter::handleCall()
{
    ProgramSentence oneProgramSentence = getOneProgramSentence(m_Pointer);
    vector<string> vecField = Common::ToVectors(oneProgramSentence.strSentence," ");
    if(vecField.size()==2 && vecField[0]=="CALL")
    {
        m_funPointerStack.push(m_Pointer);
        int nFunc = m_mapFuncIndex[vecField[1]];
        m_Pointer.nFunction = nFunc;
        m_Pointer.nSentence = 0;

        vector<ProgramSentence> vecSentence = m_vecFuncTable[nFunc];
        for(auto iter=vecSentence.begin(); iter!=vecSentence.end(); iter++/*,m_Pointer.nSentence++*/)
        {
            ProgramSentence oneNextProgramSentence = getOneProgramSentence(m_Pointer);
            if(m_bStop)
            {  break; }
            if(oneNextProgramSentence.strSentence=="RETURN" ||oneNextProgramSentence.strSentence=="ENDFUNC")
            {
                if(!m_funPointerStack.empty())
                {
                    ProgramPointer& pFunc= m_funPointerStack.top();
                    //pFunc
                    m_Pointer.nFunction = pFunc.nFunction ;
                    m_Pointer.nSentence = pFunc.nSentence+1;
                    m_funPointerStack.pop();
                }
               break;
            }
#ifndef QT_NO_DEBUG
            cout << "call:" << oneNextProgramSentence.strSentence<<endl;
#endif
            executeOneSentence(oneNextProgramSentence);
        }

    }
	return 0;
}

int RobotInterpreter::handleOperation(const VECTOR_STRING& vecField)
{
    int nRt=ERROR_TRUE;

    switch (m_programKeyWords[vecField[0]]) {
    case MOVEABSJ:
    case MOVEABSJR:
    {
        //【MOVEABSJ】【速度（0-1）】【关节路点1】【关节路点2】…
        //例：MOVEABSJ 0.2 j1 j2 …
        //virtual int moveABSJoint(std::vector<Joints>& ps, double vel){} // 关节运动
        if(vecField.size()>2)
        {
            std::vector<Joints> ps;

            for(int i=2; i<vecField.size();i++)
            {
                Joints Jn;
                auto iter = m_mapVarValue.find(vecField[i]);
                if(iter==m_mapVarValue.end())
                {
                    nRt = ERROR_UNDEFINE_VARIABLE;
                    break;
                }
                Jn.setValue(6,m_mapVarValue[vecField[i]]);
                ps.push_back(Jn);
            }

            if(m_bConStartFlag)
            {
                m_bConStartFlag = false;
                moveEndCon();
                nRt = waitCommandEnd();
            }

            double v = atof(vecField[1].c_str()) * getProgramVelocity();
            if(m_programKeyWords[vecField[0]] == MOVEABSJ)
            {
                nRt = moveABSJoint(ps,v);
            }else if(m_programKeyWords[vecField[0]] == MOVEABSJR)
            {
                 nRt = moveABSJointR(ps,v);
            }
            if(nRt==0)
            {
               nRt = waitCommandEnd();
            }
        }
        break;
    }
    case MOVEB:
    {
        //MOVEB 0.6 0.5 0.7 0.2 work 2
        //5 10 10 10 10 10 10
        //6 20 20 20 20 20 20
        if(vecField.size()>7)
        {
            //【MOVEB】【加速度】【冲击】【角速度】【精度】【坐标系】【点数量】
//moveCurve(std::vector<Terminal>& ps, std::vector<double>& vel, double acc = 0.8, double jerk = 0.8, double angle = 0.8, double bpre = 0.01, COORDINATESYSTEM frame = COORDINATE_BASE){} // 样条曲线运动
            std::vector<Terminal> ps;
            VECTOR_DOUBLE vel;
            double acc = atof(vecField[1].c_str());
            double jerk = atof(vecField[2].c_str());
            double angle = atof(vecField[3].c_str()) * getProgramVelocity();
            double bpre = atof(vecField[4].c_str());
            int frame = atof(vecField[5].c_str());
            for(int i=7;i<vecField.size();i++)
            {
                if(i%7==0)
                {
                    vel.push_back(atof(vecField[i].c_str()));
                    if(i+6<vecField.size())
                    {
                        Terminal terTmp(atof(vecField[i+1].c_str()),atof(vecField[i+2].c_str()),atof(vecField[i+3].c_str())
                                        ,atof(vecField[i+4].c_str()),atof(vecField[i+5].c_str()),atof(vecField[i+6].c_str()));
                        i +=6;
                        ps.push_back(terTmp);
                    }
                }
            }

            if(m_bConStartFlag)
            {
                m_bConStartFlag = false;
                moveEndCon();
                 waitCommandEnd();
            }
            nRt = moveCurve(ps,vel,acc,jerk,angle,bpre,(COORDINATESYSTEM)frame);
            if(nRt==0)
            {
               nRt = waitCommandEnd();
            }
        }
        break;
    }
    case MOVECR:
    case MOVEC:
    {
        //【MOVEC】【圆类型】【速度】【加速度】【冲击】【转弯区】【坐标系】【末端路点1】【末端路点2】
        //例：MOVEC whole 100 0.8 0.8 1 tool t1 t2
        if(vecField.size()==9)
        {
            int cir = atoi(vecField[1].c_str());
            VECTOR_DOUBLE vecValue1 = m_mapVarValue[vecField[7]];
            VECTOR_DOUBLE vecValue2 = m_mapVarValue[vecField[8]];
            Terminal p1(vecValue1[0],vecValue1[1],vecValue1[2],vecValue1[3],vecValue1[4],vecValue1[5]);
            Terminal p2(vecValue2[0],vecValue2[1],vecValue2[2],vecValue2[3],vecValue2[4],vecValue2[5]);
            double vel = atof(vecField[2].c_str()) * getProgramVelocity();
            double acc = atof(vecField[3].c_str());
            double jerk = atof(vecField[4].c_str());
            double turn = atof(vecField[5].c_str());
            int frame = atof(vecField[6].c_str());

            if(m_debugState == SWITCHON)
            {
                if(m_programKeyWords[vecField[0]] == MOVEC)
                {
                    nRt = moveCircle((CIRCLETYPE)cir,p1,p2,vel,acc,jerk,(COORDINATESYSTEM)frame);
                }else if(m_programKeyWords[vecField[0]] == MOVECR)
                {
                     nRt = moveCircleR((CIRCLETYPE)cir,p1,p2,vel,acc,jerk,(COORDINATESYSTEM)frame);
                }

                if(nRt == 0)
                {
                    nRt = waitCommandEnd();
                }
            }else
            {
                if(!m_bConStartFlag)
                {
                    m_bConStartFlag = true;
                    moveStartCon();
                }
                if(m_programKeyWords[vecField[0]] == MOVEC)
                {
                    nRt = moveCircleCon((CIRCLETYPE) cir, p1, p2, vel, acc, jerk,  turn, (COORDINATESYSTEM) frame );//   // 连续轨迹圆弧运动
                }else if(m_programKeyWords[vecField[0]] == MOVECR)
                {
                     nRt = moveCircleRCon((CIRCLETYPE)cir,p1,p2,vel,acc,jerk,turn, (COORDINATESYSTEM)frame);
                }
                if(nRt)
                {
                    m_bConStartFlag = false;
                    moveEndCon();
                    waitCommandEnd();
                }
            }
        }
        break;
    }
    case MOVEJR:
    case MOVEJ:
    {
        //【MOVEJ】【速度（0-1）】【坐标系】【末端路点1】【末端路点2】…
        //例：MOVEJ 0.2 tool t1 t2 …
        //int moveJoint(std::vector<Terminal>& ps, double vel, COORDINATESYSTEM frame = COORDINATE_BASE){}    // 以末端点为目标的关节运动
        if(vecField.size()>3)
        {
            std::vector<Terminal> ps;
            for(int i=3; i<vecField.size();i++)
            {
                auto iter = m_mapVarValue.find(vecField[i]);
                if(iter==m_mapVarValue.end())
                {
                    nRt = ERROR_UNDEFINE_VARIABLE;
                    break;
                }
                const vector<double>& vecValue = m_mapVarValue[vecField[i]];
                Terminal terN(vecValue[0],vecValue[1],vecValue[2],vecValue[3],vecValue[4],vecValue[5]);
                ps.push_back(terN);
            }
            int frame = atof(vecField[2].c_str());

            if(m_bConStartFlag)
            {
                m_bConStartFlag = false;
                moveEndCon();
                 waitCommandEnd();
            }

            double vel = atof(vecField[1].c_str()) * getProgramVelocity();
            if(m_programKeyWords[vecField[0]] == MOVEJ)
            {
                nRt = moveJoint(ps,vel, (COORDINATESYSTEM)frame);
            }else if(m_programKeyWords[vecField[0]] == MOVEJR)
            {
                nRt = moveJointR(ps,vel, (COORDINATESYSTEM)frame);
            }
            if(nRt == 0)
            {
                nRt = waitCommandEnd();
            }
        }
        break;
    }
    case MOVELR:
    case MOVEL:
    {
        //【MOVEL】【速度】【加速度】【冲击】【转弯区】【坐标系】【末端路点】
        //例：MOVEL 100 0.8 0.8 1 tool t1
        //virtual int moveLine(const Terminal& p, double vel, double acc = 0.8, double jerk = 0.8, COORDINATESYSTEM frame = COORDINATE_BASE){}    // 直线运动
        if(vecField.size()==7)
        {
            auto iter = m_mapVarValue.find(vecField[6]);
            if(iter==m_mapVarValue.end())
            {
                nRt = ERROR_UNDEFINE_VARIABLE;
                break;
            }
            vector<double> vecValue = m_mapVarValue[vecField[6]];
            Terminal p(vecValue[0],vecValue[1],vecValue[2],vecValue[3],vecValue[4],vecValue[5]);

            double vel = atof(vecField[1].c_str()) * getProgramVelocity();
            double acc = atof(vecField[2].c_str());
            double jerk = atof(vecField[3].c_str());
            double turn = atof(vecField[4].c_str());
            int frame = atoi(vecField[5].c_str());

            if(m_debugState == SWITCHON)
            {

                if(m_programKeyWords[vecField[0]] == MOVEL)
                {
                    nRt = moveLine(p,vel,acc,jerk,(COORDINATESYSTEM)frame);
                }else if(m_programKeyWords[vecField[0]] == MOVELR)
                {
                    nRt = moveLineR(p,vel,acc,jerk,(COORDINATESYSTEM)frame);
                }

                if(nRt==0)
                    nRt = waitCommandEnd();
            }else
            {
                if(!m_bConStartFlag)
                {
                    m_bConStartFlag = true;
                    moveStartCon();
                }
                if(m_programKeyWords[vecField[0]] == MOVEL)
                {
                    nRt = moveLineCon(p,vel,acc,jerk,turn,(COORDINATESYSTEM)frame);
                }else if(m_programKeyWords[vecField[0]] == MOVELR)
                {
                    nRt = moveLineRCon(p,vel,acc,jerk,turn, (COORDINATESYSTEM)frame);
                }
                if(nRt)
                {
                    m_bConStartFlag = false;
                    moveEndCon();
                    waitCommandEnd();
                }
            }
        }
        break;
    }
    case CALCU:
    {
        //【CALCU】【变量名】【=】【变量名/值】【运算符】【变量名/值】（若无运算符，则为赋值语句）
        //CALCU b = TRUE
        //CALCU i = i1 + i2
        //JOINT/TERMINAL CALCU j = j1 + j2  右侧仅可为JOINT变量
        if(vecField.size()==6&&vecField[2]== "=")
        {
            int opt = m_programOperator[vecField[4]] ;//m_programKeyWords[vecField[4]];
            nRt = arithmometer(vecField,(PROGRAM_OPERATOR)opt);
        }else if(vecField.size()==4&&vecField[2]== "=")
        {
            //string strType = ;m_programKeyWords[strType]
            switch (m_mapVarType[vecField[1]]) {
            case DOUBLE:
            case INT:
            {
                if(Common::isnum(vecField[3]))
                {
                    m_mapVarValue[vecField[1]][0] = atof(vecField[3].c_str());
                }
                else
                {
                    auto iter = m_mapVarValue.find(vecField[3]);
                    if(iter != m_mapVarValue.end())
                    {
                        if(m_mapVarType[vecField[3]]==INT||m_mapVarType[vecField[3]]==DOUBLE)
                        {
                            m_mapVarValue[vecField[1]][0] = m_mapVarValue[vecField[3]][0];
                        }
                        else
                        {
                            nRt = ERROR_OPERATION_ASSIGNMENT;
                        }
                    }
                    else
                    {
                        nRt = ERROR_UNDEFINE_VARIABLE;
                    }

                }
                break;
            }
            case TERMINAL:
            case JOINT:
            {
                auto iter = m_mapVarValue.find(vecField[3]);
                if(iter != m_mapVarValue.end())
                {
                    if(m_mapVarType[vecField[3]]==m_mapVarType[vecField[1]])
                    {
                        for(int i =0; i<6;i++)
                        {
                            m_mapVarValue[vecField[1]][i] = m_mapVarValue[vecField[3]][i];
                        }
                    }
                    else
                    {
                        nRt = ERROR_OPERATION_ASSIGNMENT;
                    }
                }
                break;
            }
            case BOOL:
            {
                auto iter = m_mapVarValue.find(vecField[1]);
                if(iter != m_mapVarValue.end())
                {
                    auto iter1 = m_mapVarValue.find(vecField[3]);
                    if(iter1 != m_mapVarValue.end())
                    {
                        m_mapVarValue[vecField[1]][0] = m_mapVarValue[vecField[3]][0];
                    }
                    else
                    {
                        auto iter2 = m_programKeyWords.find(vecField[3]);
                        if((iter2 != m_programKeyWords.end()))
                        {
                            //on off false true
                            m_mapVarValue[vecField[1]][0] = m_programKeyWords[vecField[3]];
                        }
                        else
                        {
                            nRt = ERROR_UNDEFINE_VARIABLE;
                        }
                    }
                }
                else
                {
                    nRt = ERROR_UNDEFINE_VARIABLE;
                }
                break;
            }
            default:
                break;
            }
        }

        break;
    }
    case TOOL:
    {
        //TOOL toolname;
        if(vecField.size()==2)
        {
            nRt = modifyToolFrame(vecField[1]);
            if (nRt == 0)
            {
                nRt = waitCommandEnd();
            }
        }else {
            nRt = ERROR_SYNTAX_ERROR;
        }
        break;
    }
    case WORK:
    {
        //TOOL toolname;
        if(vecField.size()==2)
        {
            nRt = modifyWorkFrame(vecField[1]);
            if (nRt == 0)
            {
                nRt = waitCommandEnd();
            }
        }else
        {
            nRt = ERROR_SYNTAX_ERROR;
        }
        break;
    }
    case WAITDIN:
    {
        //【WAITDIN】【DIx】【变量名/值】（BOOL变量或TRUE、FALSE、ON、OFF）
        //例：WAITDIN DI1 ON
        if(vecField.size()!=3)
        {
            break;
        }
        if(getIOConnect()==SWITCHOFF){
            nRt = ERROR_IO_CONNECT;
        }else {
            SWITCHSTATE state;
            if(vecField[2]=="TRUE"||vecField[2]=="ON")
            {
                state = SWITCHON;
            }else if(vecField[2]=="FALSE"||vecField[2]=="OFF"){
                state = SWITCHOFF;
            }else {
                auto iter = m_mapVarValue.find(vecField[2]);
                if(iter!=m_mapVarValue.end())
                {
                    state = SWITCHSTATE((int)(iter->second)[0]);
                }
            }
            while(true)
            {
                sleep_ms(1);
                if(state==getDigitalInput(m_mapPortIndex[vecField[1]]))
                {
                    break;
                }
            }
        }
        break;
    }
    case DOUT:
    {
        //【DOUT】【DOx】【变量名/值】（BOOL变量或TRUE、FALSE、ON、OFF）
        //例：DOUT DO1 ON
        //virtual void setDigitalOutputState(PORTINDEX index, SWITCHSTATE state){}    //　设置数字输出状态

        if(vecField.size()==3)
        {
            int state;
            PORTINDEX index = m_mapPortIndex[vecField[1]];
            auto iter = m_mapVarValue.find(vecField[2]);
            if(iter != m_mapVarValue.end())
            {
                state = ((*iter).second)[0];
            }else{
                if(vecField[2]=="ON"||vecField[2]=="TRUE"){
                    state = SWITCHON;
                }else if (vecField[2]=="FLIP")
                {
                    state = (getDigitalOutput(index)==SWITCHON)?SWITCHOFF:SWITCHON;
                }
                else {
                    state = SWITCHOFF;
                }
            }

            if(m_bConStartFlag)
            {
                m_bConStartFlag = false;
                moveEndCon();
                nRt = waitCommandEnd();
            }
            if(getIOConnect()==SWITCHON){
                setDigitalOutput(index,(SWITCHSTATE)state);
            }else {
                nRt = ERROR_IO_CONNECT;
            }
        }
        break;
    }
    case AOUT:
    {
        //【AOUT】【AOx】【变量名/值】（INT、DOUBLE变量或数值）
        //例：AOUT AO1 5.1
        //virtual void setAnalogOutputState(PORTINDEX index, double state){}  // 设置模拟输出状态
        double dValue;
        if(vecField.size()==3)
        {
            auto iter = m_mapVarValue.find(vecField[2]);
            if(iter != m_mapVarValue.end())
            {
                dValue = ((*iter).second)[0];
            }else
            {
                dValue = atof(vecField[2].c_str());
            }
            if(m_bConStartFlag)
            {
                m_bConStartFlag = false;
                moveEndCon();
                nRt = waitCommandEnd();
            }
            if(getIOConnect()==SWITCHON)
            {
                setAnalogOutput(m_mapPortIndex[vecField[1]],dValue);
            }else
            {
                nRt = ERROR_IO_CONNECT;
            }
        }
        break;
    }
    case DELAY:{
        if(vecField.size()==2)
        {
            double t = atof(vecField[1].c_str());
            sleep_ms(1000*t);
        }
        break;
    }
    case VLOCATE:
    {
        auto iter = m_mapVarValue.find(vecField[1]);
        if(iter != m_mapVarValue.end())
        {
            VECTOR_DOUBLE& vecValue=iter->second;
            LOCATEVISIONINDEX index;
            if(vecField[1]=="VPOS1"){
                index = LOCATEVISION_1;
            }
            else if(vecField[1]=="VPOS2"){
                index = LOCATEVISION_2;
            }
            else if(vecField[1]=="VPOS3"){
                index = LOCATEVISION_3;
            }
            else {
                nRt = ERROR_SYNTAX_ERROR;
                break;
            }
            if(SWITCHON==getVisionConnenct(index))
            {
               m_visionLocation[index] = getVisionLocation(index);
               for (int i=0;i<6;i++) {
                   m_mapVarValue[vecField[1]][i]= m_visionLocation[index].getValue(TERMINALINDEX(i));
               }

            }else {
                nRt=ERROR_CAMEL_CONNECT;
            }
        }
        break;
    }
    case COM:{
        break;
    }
    default:
        break;
    }
    if(nRt!=ERROR_TRUE)
    {
        m_bStop = true;
    }
    // 指针后移
    m_Pointer.nSentence++;
    return nRt;
}

bool RobotInterpreter::formulaComputing(string strFormula)
{
    if(m_teachType!=HR)
    {
        return  formulaComputingABB(strFormula);
    }
    //WHILE b != FALSE
    //IF i >= 1 ELSEIF i >= 1 ELSE i >= 1
    if(strFormula=="ELSE")
    {
        return true;
    }
    bool bRt;
    VECTOR_STRING vecField = Common::ToVectors(strFormula," ");
    if(vecField.size()==4)
    {
        // 两个都是变量
        auto iter1 = m_mapVarValue.find(vecField[1]);
        auto iter2 = m_mapVarValue.find(vecField[3]);
        double fVal1 ;    //= vecField[1];
        double fVal2 ;    //= vecField[3];
        int varType1 ,varType2;
        varType1 = -1;
        varType2 = -1;
        if(iter1 != m_mapVarValue.end())
        {
            varType1 = m_mapVarType[vecField[1]];
        }
        if(iter2 != m_mapVarValue.end())
        {
            varType2 = m_mapVarType[vecField[3]];
        }

//        if(varType1!=-1&&varType2!=-1&&varType1!=varType2)
//        {
//            printInfo(INFO_ERROR, m_mapErrorInfo[ERROR_OPERATION_ASSIGNMENT]);
//            return false;
//        }

        //PROGRAM_OPERATOR opt = (PROGRAM_OPERATOR)m_programKeyWords[vecField[2]];
        PROGRAM_OPERATOR opt = (PROGRAM_OPERATOR)m_programOperator[vecField[2]];
        /////////////////////////////////////////////////
        //INT或double
        if(varType1==INT || varType1==DOUBLE)
        {
            fVal1 = m_mapVarValue[vecField[1]][0];
            if(varType2==INT || varType2==DOUBLE)
            {
                //bRt = m_mapVarValue[vecField[1]][0]==m_mapVarValue[vecField[3]][0];
                fVal2 = m_mapVarValue[vecField[3]][0];
            }else if(Common::isnum(vecField[3]))
            {
                fVal2 = atof(vecField[3].c_str());
            }else if(isPort(vecField[3],fVal2))
            {
                //bRt = m_mapVarValue[vecField[1]][0]== fVal2;
            }
        }else if(Common::isnum(vecField[1]))
        {
            fVal1 = atof(vecField[1].c_str());
            if(varType2==INT || varType2==DOUBLE)
            {
                //bRt = atof(vecField[1].c_str())==m_mapVarValue[vecField[3]][0];
                fVal2 = m_mapVarValue[vecField[3]][0];
            }else if(Common::isnum(vecField[3]))
            {
                //两个数字常量的比较？
                //bRt = atof(vecField[1].c_str())== atof(vecField[3].c_str());
                fVal2 = atof(vecField[3].c_str());
            }else if(isPort(vecField[3],fVal2))
            {
                //bRt = atof(vecField[1].c_str())== fVal2;
            }
        }
        //port
        else if(isPort(vecField[1],fVal1))
        {
            if(varType2==INT || varType2==DOUBLE)
            {
                //bRt = fVal1==m_mapVarValue[vecField[3]][0];
                fVal2 = m_mapVarValue[vecField[3]][0];
            }else if(Common::isnum(vecField[3]))
            {
                //bRt = fVal1== atof(vecField[3].c_str());
                fVal2 = atof(vecField[3].c_str());
            }else if(isPort(vecField[3],fVal2))
            {
                //bRt = fVal1== fVal2;
            }else if (vecField[3]=="ON") {
                fVal2 = 1;
            }else if(vecField[3]=="OFF"){
                fVal2 = 0;
            }
        }
        //Terminal JOINT
        else if((varType1==TERMINAL || varType1==JOINT) && (varType1==varType2))
        {
            VECTOR_DOUBLE vecV1=m_mapVarValue[vecField[1]];
            VECTOR_DOUBLE vecV2=m_mapVarValue[vecField[3]];
            for(int i=0;i<6;i++)
            {
                if(vecV1[i]!=vecV2[i])
                {
                    return false;
                }
            }
        }
        //bool
        else if(varType1==BOOL)
        {
            fVal1 = m_mapVarValue[vecField[1]][0];
            if(varType2==BOOL)
            {
                //bRt = (m_mapVarValue[vecField[1]]==m_mapVarValue[vecField[3]]);
                fVal2 = m_mapVarValue[vecField[3]][0];
            }
            else if(vecField[3]=="TRUE"||vecField[3]=="ON")
            {
                //bRt = (m_mapVarValue[vecField[1]][0]==1);
                fVal2 = 1;
            }else if(vecField[3]=="FALSE"||vecField[3]=="OFF")
            {
                //bRt = (m_mapVarValue[vecField[1]][0]==0);
                fVal2 = 0;
            }else if(isPort(vecField[3],fVal2))
            {
                //bRt = (m_mapVarValue[vecField[1]][0]==fVal2);
            }
        }

        bRt = bCompare(fVal1, fVal2,opt);
        ///////////////////////////////////////////////
#if 0
        switch (m_programKeyWords[vecField[2]]) {
        case OPERATOR_NEQ:
        case OPERATOR_EQU:
        {
            //INT或double
            if(varType1==INT || varType1==DOUBLE)
            {
                if(varType2==INT || varType2==DOUBLE)
                {
                    //bRt = m_mapVarValue[vecField[1]][0]==m_mapVarValue[vecField[3]][0];
                    bRt = bCompare(m_mapVarValue[vecField[1]][0], m_mapVarValue[vecField[3]][0],opt );
                }else if(Common::isnum(vecField[3]))
                {
                    bRt = m_mapVarValue[vecField[1]][0]== atof(vecField[3].c_str());
                }else if(isPort(vecField[3],fVal2))
                {
                    bRt = m_mapVarValue[vecField[1]][0]== fVal2;
                }

            }else if(Common::isnum(vecField[1]))
            {
                if(varType2==INT || varType2==DOUBLE)
                {
                    bRt = atof(vecField[1].c_str())==m_mapVarValue[vecField[3]][0];
                }else if(Common::isnum(vecField[3]))
                {
                    //两个数字常量的比较？
                    bRt = atof(vecField[1].c_str())== atof(vecField[3].c_str());
                }else if(isPort(vecField[3],fVal2))
                {
                    bRt = atof(vecField[1].c_str())== fVal2;
                }
            }
            //port
            else if(isPort(vecField[1],fVal1))
            {
                if(varType2==INT || varType2==DOUBLE)
                {
                    bRt = fVal1==m_mapVarValue[vecField[3]][0];
                }else if(Common::isnum(vecField[3]))
                {
                    bRt = fVal1== atof(vecField[3].c_str());
                }else if(isPort(vecField[3],fVal2))
                {
                    bRt = fVal1== fVal2;
                }

            }
            //Terminal JOINT
            else if((varType1==TERMINAL || varType1==JOINT) && (varType1==varType2))
            {
                VECTOR_DOUBLE vecV1=m_mapVarValue[vecField[1]];
                VECTOR_DOUBLE vecV2=m_mapVarValue[vecField[3]];
                for(int i=0;i<6;i++)
                {
                    if(vecV1[i]!=vecV2[i])
                    {
                        bRt = false;
                        break;
                    }
                }
            }
            //bool DIx DOx
            else if(varType1==BOOL)
            {
                if(varType2==BOOL)
                {
                    bRt = (m_mapVarValue[vecField[1]]==m_mapVarValue[vecField[3]]);
                }
                else if(vecField[3]=="TRUE"||vecField[3]=="ON")
                {
                    bRt = (m_mapVarValue[vecField[1]][0]==1);
                }else if(vecField[3]=="FALSE"||vecField[3]=="OFF")
                {
                    bRt = (m_mapVarValue[vecField[1]][0]==0);
                }else if(isPort(vecField[3],fVal2))
                {
                    bRt = (m_mapVarValue[vecField[1]][0]==fVal2);
                }
            }

            if(m_programKeyWords[vecField[2]] == OPERATOR_NEQ)
            {
                bRt = bRt? false:true;
            }

            break;
        }
        case OPERATOR_LAG:
        {
            break;
        }
        case OPERATOR_LES:
        {
            break;
        }
        case OPERATOR_GEQ:
        {
            break;
        }
        case OPERATOR_LEQ:
        {
            break;
        }
        default:
            break;
        }
        return bRt;
#endif
    }

    return bRt;
}

int RobotInterpreter::arithmometer(const VECTOR_STRING& vecField ,PROGRAM_OPERATOR opt)
{
    //CALCU j = j1 + j2
    //switch (m_programKeyWords[vecField[1]]) {
    switch (m_mapVarType[vecField[1]]) {
    case INT:
    case DOUBLE:
    {
        //CALCU i= i1 * i2
        double num1,num2;
        if(Common::isnum( vecField[3]))
        {
            num1=atof(vecField[3].c_str());
        }else
        {
            auto iter = m_mapVarValue.find(vecField[3]);
            if(iter!=m_mapVarValue.end())
            {
                if(m_mapVarType[vecField[3]]==INT||m_mapVarType[vecField[3]]==DOUBLE)
                {
                    num1 = ((*iter).second)[0];
                }
                else
                {
                    return ERROR_OPERATION_ASSIGNMENT;
                }
            }
            else
            {
                return ERROR_UNDEFINE_VARIABLE;
            }
        }
        if(Common::isnum( vecField[5]))
        {
            num2=atof(vecField[5].c_str());
        }else
        {
            auto iter = m_mapVarValue.find(vecField[5]);
            if(iter!=m_mapVarValue.end())
            {

                if(m_mapVarType[vecField[5]]==INT||m_mapVarType[vecField[5]]==DOUBLE)
                {
                    num2 = ((*iter).second)[0];
                }
                else
                {
                    return ERROR_OPERATION_ASSIGNMENT;
                }
            }
            else
            {
                return ERROR_UNDEFINE_VARIABLE;
            }
        }

        auto iter = m_mapVarValue.find(vecField[1]);
        if(iter != m_mapVarValue.end())
        {
            VECTOR_DOUBLE& vecValue = (*iter).second;
            //vecValue[0]=num1 + num2;
            switch (opt) {
            case OPERATOR_ADD:
            {
                vecValue[0]=num1 + num2;
                break;
            }
            case OPERATOR_SUB:
            {
                vecValue[0]=num1 - num2;
                break;
            }
            case OPERATOR_MUL:
            {
                vecValue[0]=num1 * num2;
                break;
            }
            case OPERATOR_DIV:
            {
                if(num2>0.000001||num2<-0.000001)
                {
                    vecValue[0]=num1 / num2;
                    break;
                }
            }
            case OPERATOR_MOD:
            {
                if(m_mapVarType[vecField[1]]==INT)
                {
                    vecValue[0]=(int)num1 % (int)num2;
                    break;
                }
            }
            default:
                break;
            }
        }
        else
        {
            return ERROR_UNDEFINE_VARIABLE;
        }
        break;
    }
    case TERMINAL:
    case JOINT:
    {
        auto iter = m_mapVarValue.find(vecField[1]);
        if(iter!=m_mapVarValue.end())
        {
            auto iter1 = m_mapVarValue.find(vecField[3]);
            auto iter2 = m_mapVarValue.find(vecField[5]);

            if(iter1!=m_mapVarValue.end()&&iter2!=m_mapVarValue.end())
            {
                VECTOR_DOUBLE& vecValue = (*iter).second;
                VECTOR_DOUBLE& vecValue1 = (*iter1).second;
                VECTOR_DOUBLE& vecValue2 = (*iter2).second;
                for(int i=0; i<6;i++)
                {
                    switch (opt) {
                    case OPERATOR_ADD:
                    {
                        vecValue[i] = vecValue1[i] + vecValue2[i];
                        break;
                    }
                    case OPERATOR_SUB:
                    {
                        vecValue[i] = vecValue1[i] - vecValue2[i];
                        break;
                    }
                    case OPERATOR_MUL:
                    {
                       vecValue[i] = vecValue1[i] * vecValue2[i];
                        break;
                    }
                    case OPERATOR_DIV:
                    {
                        if(vecValue2[i]>0.000001||vecValue2[i]<-0.000001)
                        {
                            vecValue[i] = vecValue1[i] / vecValue2[i];
                        }
                        break;
                    }
                    default:
                        break;
                    }
                }
            }
        }
        else
        {
            return ERROR_UNDEFINE_VARIABLE;
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

bool RobotInterpreter::isPort(string strPortName, double fValue)
{
    //DXn AXn
    if(strPortName.length()>=3)
    {
        string strSub = strPortName.substr(2);
        int index;
        if(Common::isnum(strSub))
        {
            index = atoi(strSub.c_str());
            do
            {
                //数字输入
                if(strPortName.at(0)=='D'&&strPortName.at(1)=='I')
                {
                    //virtual SWITCHSTATE getDigitalInputState(PORTINDEX index){} // 获取数字输入状态
                    fValue = getDigitalInput((PORTINDEX) index);
                }
                //数字输入
                else if(strPortName.at(0)=='D'&&strPortName.at(1)=='O')
                {
                    //virtual SWITCHSTATE getDigitalOutputState(PORTINDEX index){}    // 获取数字输出状态
                    fValue = getDigitalOutput((PORTINDEX) index);
                }
                //模拟输入
                else if(strPortName.at(0)=='A'&&strPortName.at(1)=='I')
                {
                    //virtual double getAnalogInputState(PORTINDEX index){}   // 获取模拟输入状态
                    fValue = getAnalogInput((PORTINDEX) index);
                }
                //模拟输出
                else if(strPortName.at(0)=='A'&&strPortName.at(1)=='O')
                {
                    //virtual double getAnalogOutputState(PORTINDEX index){}  // 获取模拟输出状态
                    fValue = getAnalogInput((PORTINDEX) index);
                }else
                {
                    break;
                }
                return true;
            }while(false);
        }
    }
    return false;
}

bool RobotInterpreter::bCompare(double Value1, double Value2, PROGRAM_OPERATOR opt)
{
    bool bRt=false;
    switch (opt) {
	case OPERATOR_ASS:
    case OPERATOR_EQU:
    {
        bRt = (Value1==Value2?true:false);
        break;
    }
    case OPERATOR_NEQ:
    {
        bRt = (Value1==Value2?false:true);
        break;
    }
    case OPERATOR_LAG:
    {
        bRt = (Value1>Value2?true:false);
        break;
    }
    case OPERATOR_LES:
    {
        bRt = (Value1<Value2?true:false);
        break;
    }
    case OPERATOR_GEQ:
    {
        bRt = (Value1>=Value2?true:false);
        break;
    }
    case OPERATOR_LEQ:
    {
        bRt = (Value1<=Value2?true:false);
        break;
    }
    default:
        break;
    }
	return bRt;
}

#ifndef QT_NO_DEBUG
void RobotInterpreter::testSet()
{
    string strSen("MoveL Offs(p2, 0, 0, 10), v1000, z50, tool1;");
    handleOperationABB(strSen);
    return ;

    for (int i=0;i<2;i++) {
        readRobotProgram("Movej.program");
        ProgramPointer p;
        p.nFunction=0;
        p.nSentence=1;
        int nRt = addProgramSentence(p,ProgramSentence("MOVABSJ 0.8 joP7 joP8 joP9 joP10 joP11 joP12"));
        addVariable("INT I = 10");
    }
    writeRobotProgram("Movej.program");

    //Common::TSingleton<Common::CDbgPrint>::Instance();
    //Common::TSingleton<Common::CDbgPrint>::Instance()->SetDbgFunction(Common::hrDbgPrint);
    //Common::TSingleton<Common::CDbgPrint>::Instance()->ReadLevelFromFile();
    char buf[BUFSIZ];
    sprintf(buf,"%x,%x",DBG_ERROR,0x3001);
    //Common::TSingleton<Common::CDbgPrint>::Instance()->SetDebug((const char*)buf);

    //DBGPRINT(DBG_ERROR,"lifei test");

    //readFunctionABB("Module2VAR.MOD");
    //runABB();
    Json::StreamWriterBuilder builder;
    std::unique_ptr<Json::StreamWriter> writeInfo(builder.newStreamWriter());
    Json::Value root, lang;
    root["0"] = "false";
    root["id"] = 123;
    lang["first"] = "one";
    lang["last"] = "last";
    root["value"] = lang;
    ostringstream oStr;
    writeInfo->write(root,&oStr);
    string str = oStr.str();
    cout<<str<<endl;

    bool bOK=false;
    Json::Value rootV;
    {
        Json::CharReaderBuilder jsreader;
        std::unique_ptr<Json::CharReader> const readerinfo(jsreader.newCharReader());
        string err;
        int nLen = str.length();
        if (readerinfo->parse(str.c_str(), str.c_str() + nLen, &rootV, &err))
        {
            bOK=true;
        }
    }


    if(root["id"].isInt())
    {
        int id = root["id"].asInt();
        cout <<"id="<<id<<endl;
    }

    m_jsWrapper.getJsonPredefineValue("var.json");
    string strValue ;
    bool b = m_jsWrapper.getJsonValue("string","diskhome",strValue);
    cout <<"bool : " << b<<"strValue:"<<strValue<<endl;
}
#endif

int RobotInterpreter::grammarCheck(string strSentence)
{
    int nRt=0;
    VECTOR_STRING vecField = Common::ToVectors(strSentence," ");
    if(vecField.size()>0)
    {
        int nType = m_programKeyWords[vecField[0]];
        switch (nType) {
        case BOOL:
        case INT:
        case DOUBLE:
        case JOINT:
        case TERMINAL:
        {
            //赋值语句
            do{
                if((vecField.size()==4&&vecField[2]=="=")||(vecField.size()==6&&vecField[2]=="="))
                {
                    auto iter=m_mapVarValue.find(vecField[1]);
                    if (iter!=m_mapVarValue.end())
                    {
                        nRt = ERROR_VARIABLE_REDEFINITION;
                        break;
                    }else
                    {
                        auto iter1=m_mapVarValue.find(vecField[3]);
                        if (iter1!=m_mapVarValue.end())
                        {
                            nRt = ERROR_TRUE;
                            break;
                        }else
                        {
                            if(nType == BOOL)
                            {
                                if(vecField[3]=="TRUE"||vecField[3]=="FALSE"
                                   || vecField[3]=="ON"||vecField[3]=="OFF")
                                {
                                    nRt = ERROR_TRUE;
                                    break;
                                }
                            }else if(nType == INT||nType==DOUBLE)
                            {
                                if(vecField.size()==4&& Common::isnum(vecField[3]))
                                {
                                    nRt = ERROR_TRUE;
                                    break;
                                }
                            }else if(nType == TERMINAL||nType == JOINT)
                            {
                                if(vecField.size()==6)
                                {
                                    nRt = ERROR_TRUE;
                                    break;
                                }
                            }
                        }
                    }
                }
                nRt = ERROR_VARIABLE_DEFINITION;
            }while(false);

            break;
        }
        case FOR :
        {
            if(vecField.size()==2||Common::isnum(vecField[1]))
            {
                nRt = ERROR_TRUE;
            }else{
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case BREAK:
        case CONTINUE:
        case MAIN:
        case ENDMAIN:
        case ENDFUNC:
        case ENDWHILE:
        case ENDFOR:
        case RETURN:
        case ENDIF:
        case ELSE:
        {
            if(vecField.size()!=1)
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case CALL:
        case FUNC:
        {
            if(vecField.size()!=2)
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case IF:
        case ELSEIF:
        case WHILE:
        {
            //auto iter = m_programOperator.find(vecField[])
            if(vecField.size()==4)
            {
                auto iter = m_programOperator.find(vecField[2]);
                if(iter!= m_programOperator.end()){

                }else
                {
                    nRt = ERROR_SYNTAX_ERROR;
                    break;
                }

                // 两个都是变量
                auto iter1 = m_mapVarValue.find(vecField[1]);
                auto iter2 = m_mapVarValue.find(vecField[3]);
                double fVal1,fVal2;
                int varType1 ,varType2;
                varType1 = -1;
                varType2 = -1;
                if(iter1 != m_mapVarValue.end())
                {
                    varType1 = m_mapVarType[vecField[1]];
                }
                if(iter2 != m_mapVarValue.end())
                {
                    varType2 = m_mapVarType[vecField[3]];
                }

                //INT或double
                if(varType1==INT || varType1==DOUBLE)
                {
                    if(varType2==INT || varType2==DOUBLE)
                    {
                    }else if(Common::isnum(vecField[3]))
                    {
                    }else if(isPort(vecField[3],fVal2))
                    {
                        //bRt = m_mapVarValue[vecField[1]][0]== fVal2;
                    }else
                    {
                        nRt = ERROR_SYNTAX_ERROR;
                    }
                }else if(Common::isnum(vecField[1]))
                {
                    if(varType2==INT || varType2==DOUBLE)
                    {
                    }else if(Common::isnum(vecField[3]))
                    {
                    }else if(isPort(vecField[3],fVal2))
                    {
                    }else
                    {
                        nRt = ERROR_SYNTAX_ERROR;
                    }
                }
                //port
                else if(isPort(vecField[1],fVal1))
                {
                    if(varType2==INT || varType2==DOUBLE)
                    {
                    }else if(Common::isnum(vecField[3]))
                    {
                    }else if(isPort(vecField[3],fVal2))
                    {
                    }else if(vecField[3]=="ON"||vecField[3]=="OFF")
                    {}else
                    {
                        nRt = ERROR_SYNTAX_ERROR;
                    }
                }
                //Terminal JOINT
                else if(varType1==TERMINAL || varType1==JOINT )
                {
                   if(varType1!=varType2)
                   {
                       nRt = ERROR_SYNTAX_ERROR;
                   }
                }
                //bool
                else if(varType1==BOOL)
                {
                    fVal1 = m_mapVarValue[vecField[1]][0];
                    if(varType2==BOOL)
                    {
                    }
                    else if(vecField[3]=="TRUE"||vecField[3]=="ON")
                    {
                    }else if(vecField[3]=="FALSE"||vecField[3]=="OFF")
                    {
                    }else if(isPort(vecField[3],fVal2))
                    {
                    }else
                    {
                        nRt = ERROR_SYNTAX_ERROR;
                    }
                }
                else
                {
                    nRt = ERROR_UNDEFINE_VARIABLE;
                }
            }else
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case CALCU:
        {
            //【CALCU】【变量名】【=】【变量名/值】【运算符】【变量名/值】（若无运算符，则为赋值语句）
            //CALCU b = TRUE
            //CALCU i = i1 + i2
            //JOINT/TERMINAL CALCU j = j1 + j2  右侧仅可为JOINT变量
            if(vecField.size()==6&&vecField[2]== "=")
            {
                switch (m_mapVarType[vecField[1]]) {
                case DOUBLE:
                case INT:
                {
                    int iArray[]={3,5};
                    for(int i=0;i<2;i++)
                    {
                        if(Common::isnum(vecField[iArray[i]]))
                        {
                        }
                        else
                        {
                            auto iter = m_mapVarValue.find(vecField[iArray[i]]);
                            if(iter != m_mapVarValue.end())
                            {
                                if(m_mapVarType[vecField[iArray[i]]]==INT||m_mapVarType[vecField[iArray[i]]]==DOUBLE)
                                {
                                }
                                else
                                {
                                    nRt = ERROR_OPERATION_ASSIGNMENT;
                                    break;
                                }
                            }
                            else
                            {
                                nRt = ERROR_UNDEFINE_VARIABLE;
                                break;
                            }
                        }
                    }
                    break;
                }
                case TERMINAL:
                case JOINT:
                {
                    auto iter = m_mapVarType.find(vecField[3]);
                    auto iter1 = m_mapVarType.find(vecField[3]);
                    do{
                        if(iter != m_mapVarType.end()&&(iter1 != m_mapVarType.end()))
                        {
                            if((m_mapVarType[vecField[3]]==m_mapVarType[vecField[1]])
                                &&(m_mapVarType[vecField[5]]==m_mapVarType[vecField[1]]))
                            {
                                break;
                            }
                        }
                        nRt = ERROR_OPERATION_ASSIGNMENT;
                    }while(false);

                    break;
                }
                case BOOL:
                {
                    auto iter = m_mapVarValue.find(vecField[1]);
                    if(iter != m_mapVarValue.end())
                    {
                        auto iter1 = m_mapVarValue.find(vecField[3]);
                        if(iter1 != m_mapVarValue.end())
                        {
                        }
                        else
                        {
                            auto iter2 = m_programKeyWords.find(vecField[3]);
                            if((iter2 != m_programKeyWords.end()))
                            {
                            }
                            else
                            {
                                nRt = ERROR_UNDEFINE_VARIABLE;
                            }
                        }
                    }
                    else
                    {
                        nRt = ERROR_UNDEFINE_VARIABLE;
                    }
                    break;
                }
                default:
                    break;
                }
            }else if(vecField.size()==4&&vecField[2]== "=")
            {
                //string strType = ;m_programKeyWords[strType]
                switch (m_mapVarType[vecField[1]]) {
                case DOUBLE:
                case INT:
                {
                    if(Common::isnum(vecField[3]))
                    {
                    }
                    else
                    {
                        auto iter = m_mapVarValue.find(vecField[3]);
                        if(iter != m_mapVarValue.end())
                        {
                            if(m_mapVarType[vecField[3]]==INT||m_mapVarType[vecField[3]]==DOUBLE)
                            {
                            }
                            else
                            {
                                nRt = ERROR_OPERATION_ASSIGNMENT;
                            }
                        }
                        else
                        {
                            nRt = ERROR_UNDEFINE_VARIABLE;
                        }

                    }
                    break;
                }
                case TERMINAL:
                case JOINT:
                {
                    auto iter = m_mapVarValue.find(vecField[3]);
                    if(iter != m_mapVarValue.end())
                    {
                        if(m_mapVarType[vecField[3]]==m_mapVarType[vecField[1]])
                        {
                        }
                        else
                        {
                            nRt = ERROR_OPERATION_ASSIGNMENT;
                        }
                    }
                    break;
                }
                case BOOL:
                {
                    auto iter = m_mapVarValue.find(vecField[1]);
                    if(iter != m_mapVarValue.end())
                    {
                        auto iter1 = m_mapVarValue.find(vecField[3]);
                        if(iter1 != m_mapVarValue.end())
                        {
                        }
                        else
                        {
                            auto iter2 = m_programKeyWords.find(vecField[3]);
                            if((iter2 != m_programKeyWords.end()))
                            {
                            }
                            else
                            {
                                nRt = ERROR_UNDEFINE_VARIABLE;
                            }
                        }
                    }
                    else
                    {
                        nRt = ERROR_UNDEFINE_VARIABLE;
                    }
                    break;
                }
                default:
                    break;
                }
            }

            break;
        }
        case MOVEABSJ:
        case MOVEABSJR:
        {
            if(vecField.size() < 3)
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case MOVEB:
        {
            if(vecField.size()<7||vecField.size()%7!=0)
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case MOVECR:
        case MOVEC:
        {
            if(vecField.size()!=9)
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case MOVEJR:
        case MOVEJ:
        {
            if(vecField.size()<4)
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        //case MOVEJT:
        case MOVELR:
        case MOVEL:
        {
             if(vecField.size()!=7)
             {
                 nRt = ERROR_SYNTAX_ERROR;
             }
            break;
        }
        case DELAY:
        {
            auto iter = m_mapVarType.find(vecField[1]);
            if(Common::isnum(vecField[1])||iter!=m_mapVarType.end())
                break;
            nRt=ERROR_SYNTAX_ERROR;
            break;
        }
        case WORK:
        case TOOL:{
            if(vecField.size()!=2)
            {
                nRt=ERROR_SYNTAX_ERROR;
            }
            break;
        }
        case COM:{
            break;
        }
        case VLOCATE:{
            break;
        }
        //WAITDIN DI1 ON
        case WAITDIN:
        case DOUT:
        case AOUT:
        {
            if(vecField.size()!=3)
            {
                nRt = ERROR_SYNTAX_ERROR;
            }
            break;
        }
        default:
            nRt = ERROR_INVALID_KEYWORD;
            break;
        }
    }
    else
    {
        nRt = ERROR_NULL_SENTENCE;
    }
    return nRt;
}



void RobotInterpreter::run()
{
#ifndef QT_NO_DEBUG
    //readRobotProgram(PROGRAMFILE);
#endif

    //设置main函数入口指针
    ProgramPointer pPoint;
    int nMainIndex = 0;
    auto iter = m_mapFuncIndex.find("main");
    if(iter!=m_mapFuncIndex.end())
    {
        nMainIndex = iter->second;
    }
    pPoint.nFunction = nMainIndex;
    pPoint.nSentence = 0;
    setPointer(pPoint);

    if(m_vecFuncTable.size()==0)
    {
        cout << "m_vecFuncTable is empty!"<< endl;
        printInfo(INFO_WARNING,"程序读取为空");
        return;
    }
    m_bStop = false;
    m_bPause = false;
    m_bConStartFlag = false;
    while(!m_bStop)
    {
        ProgramSentence OneProgramSentence = getOneProgramSentence(m_Pointer);
        if(OneProgramSentence.strSentence == "ENDMAIN" ||OneProgramSentence.strSentence == "RETURN"
            ||OneProgramSentence.strSentence.empty()||OneProgramSentence.strSentence == "ENDPROC")
        {
            ProgramPointer pPoint1;
            pPoint1.nFunction = nMainIndex;
            pPoint1.nSentence = 0;
            setPointer(pPoint1);
            break;
        }
        cout << "main::"<<OneProgramSentence.strSentence<<endl;
        int nRt = executeOneSentence(OneProgramSentence);

    }

}

void RobotInterpreter::pause(bool bPause)
{
    m_bPause = bPause;
}

void RobotInterpreter::stop()
{
    m_bStop = true;
}

void RobotInterpreter::stepRun()
{
#ifndef QT_NO_DEBUG
    //readRobotProgram(PROGRAMFILE);
#endif
    //设置指针
    int nMainIndex = 0;
    auto iter = m_mapFuncIndex.find("main");
    if(iter!=m_mapFuncIndex.end())
    {
        nMainIndex = iter->second;
        m_debugState = SWITCHON;
        m_bSingle = true;
    }
    m_Pointer.nFunction = nMainIndex;
    m_Pointer.nSentence = 0;

    if(m_vecFuncTable.size()==0)
    {
        cout << "m_vecFuncTable is empty!"<< endl;
        printInfo(INFO_WARNING,"程序读取为空");
        return;
    }
    m_bStop = false;
    m_bPause = false;
    m_bConStartFlag = false;

    while(!m_bStop)
    {
        ProgramSentence OneProgramSentence = getOneProgramSentence(m_Pointer);
        if(OneProgramSentence.strSentence == "ENDMAIN" ||OneProgramSentence.strSentence == "RETURN"
            ||OneProgramSentence.strSentence.empty()||OneProgramSentence.strSentence == "ENDPROC")
        {
            //重设指针
            m_Pointer.nFunction = 0;
            m_Pointer.nSentence = 0;
            break;
        }
        cout << "main::"<<OneProgramSentence.strSentence<<endl;
        executeOneSentence(OneProgramSentence);
    }
}

void RobotInterpreter::step()
{
    //m_bSingle = m_bSingle ?false:true;
    m_bSingle = true;
    m_bPause = false;

}

void RobotInterpreter::stepPause()
{
    m_bPause = true;
    m_bStop = true;
    m_bSingle = false;
    //保留当前指针
    //m_tmpPointer.nFunction = m_Pointer.nFunction;
    //m_tmpPointer.nSentence = m_Pointer.nSentence;
    //需要保存大量程序运行的状态量，暂时保存指针为起始位置
    m_tmpPointer.nFunction = 0;
    m_tmpPointer.nSentence = 0;
}

bool RobotInterpreter::IsStop()
{
    return  m_bStop;
}

void RobotInterpreter::readRobotProgram(const char* path)
{

    clearFuncTable();          //清除函数表
    clearVarTable();
    VECTOR_STRING vecAllSentence;
    Common::ReadFileToVector(path,vecAllSentence);
    if(m_teachType==ABB||(vecAllSentence.size()>0&&vecAllSentence[0].find("!ABB")!=string::npos))
    {
        m_teachType=ABB;
        readFunctionABB(path);
        return;
    }
    for(int i=0; i<vecAllSentence.size();i++)
    {
        VECTOR_STRING vecFuncName;
        vecFuncName = Common::ToVectors(vecAllSentence[i]," ");

        if(m_programKeyWords[vecFuncName[0]]==VALUE)
        {
            ///读取变量，到main结束
            for(i=i+1;i<vecAllSentence.size()&&m_programKeyWords[vecAllSentence[i]]!=MAIN ;i++)
            {
                //BOOL b = TRUE
                VECTOR_STRING vecField;
                vecField = Common::ToVectors(vecAllSentence[i]," ");
                int nSize = vecField.size();
                if(nSize>3)
                {
                    vector<double> vecValue;
                    for(int i=3; i<nSize; i++)
                    {
                        string strValue = vecField[i];
                        if(strValue=="TRUE"||strValue=="ON")
                        {
                            vecValue.push_back(1);
                        }else if(strValue=="FALSE"||strValue=="OFF")
                        {
                            vecValue.push_back(0);
                        }else if(Common::isnum(strValue))
                        {
                            vecValue.push_back( atof( strValue.c_str()) );
                        }
                    }

                    m_mapVarType[vecField[1]] =  m_programKeyWords[vecField[0]];//vecField[0];
                    m_mapVarValue[vecField[1]] = vecValue;
                }
                else
                {
                    printInfo(INFO_WARNING, m_mapErrorInfo[ERROR_VARIABLE_ASSIGNMENT]);
                }

            }
        }

        ///读取main函数，到endmain结束
        if(m_programKeyWords[vecAllSentence[i]]==MAIN)
        {
            m_mapFuncIndex[vecAllSentence[i]] = m_vecFuncTable.size();
            vector<ProgramSentence> vecProgramSentence;
            for(;i<vecAllSentence.size() ;i++)
            {
                ProgramSentence OneProgramSentence;
                OneProgramSentence.strSentence = vecAllSentence[i];
                vecProgramSentence.push_back(OneProgramSentence);
                if(m_programKeyWords[vecAllSentence[i]] == ENDMAIN )
                {
                    break;
                }
            }
            m_vecFuncTable.push_back(vecProgramSentence);
            continue;
        }


        ///读取FUNC函数，到endFUNC结束
        if(vecFuncName.size()==2 && m_programKeyWords[vecFuncName[0]]==FUNC)
        {
            m_mapFuncIndex[vecFuncName[1]] = m_vecFuncTable.size();
            vector<ProgramSentence> vecProgramSentence;
            for(;i<vecAllSentence.size();i++)
            {
                ProgramSentence OneProgramSentence;
                OneProgramSentence.strSentence = vecAllSentence[i];
                vecProgramSentence.push_back(OneProgramSentence);
                if(m_programKeyWords[vecAllSentence[i]] == ENDFUNC )
                {
                    break;
                }
            }
            m_vecFuncTable.push_back(vecProgramSentence);
        }
    }

    //添加保留字变量
    VECTOR_DOUBLE vecTer;
    for (int i=0;i<6;i++) {
        vecTer.push_back(0);
    }
    m_mapVarType["VPOS1"]=TERMINAL;
    m_mapVarType["VPOS2"]=TERMINAL;
    m_mapVarType["VPOS3"]=TERMINAL;
    m_mapVarValue["VPOS1"]=vecTer;
    m_mapVarValue["VPOS2"]=vecTer;
    m_mapVarValue["VPOS3"]=vecTer;
}

void RobotInterpreter::writeRobotProgram(const char *path)
{
    //视觉保留关键字不保存
    delVariable("VPOS1");
    delVariable("VPOS2");
    delVariable("VPOS3");
    //将函数表中的程序语句全写入文件中，文件中原来内容被覆盖。
    fstream out;
    out.open(path, ios_base::out|ios_base::in|ios_base::trunc/*|ios_base::ate|ios_base::app*/);
    if(out.is_open())
    {
        out <<"VALUE"<<endl;
        auto iter = m_mapVarValue.begin();
        for(;iter!=m_mapVarValue.end();iter++)
        {
            string strVarName = iter->first;
            auto iterKey = m_programKeyWords.begin() ;
            while(iterKey!=m_programKeyWords.end())
            {
                if(m_mapVarType[strVarName]==iterKey->second)
                    break;
                iterKey++;
            }
            string strVarType = iterKey->first;
            VECTOR_DOUBLE& vecValue = iter->second;

            string strLine;
            strLine = strVarType + " ";
            strLine += strVarName + " =";
            for(int i=0;i<vecValue.size();i++)
            {
                strLine += " " ;
                if(strVarType=="BOOL")
                {
                    strLine += (vecValue[i]>-0.000001 &&vecValue[i]<0.000001)? "FALSE":"TRUE";
                }
                else if(strVarType=="INT")
                {
                    char buff[16];
                    sprintf(buff,"%.6lf",vecValue[i]);
                    strLine += buff;
                }
                else
                {
                    char buff[16];
                    sprintf(buff,"%.6lf",vecValue[i]);
                    strLine += buff;
                }

            }

            out << strLine <<endl;
        }

        for(int i =0; i<m_vecFuncTable.size(); i++)
        {
            VECTOR_SENTENCE& vecSentence = m_vecFuncTable[i];
            for(int j=0; j<vecSentence.size(); j++)
            {
                out << vecSentence[j].strSentence << endl;
            }
        }
        out.close();
    }
}

int RobotInterpreter::readFunction(const char *path)
{
    VECTOR_STRING vecAllSentence;
    Common::ReadFileToVector(path,vecAllSentence);
    cout << "size:" << vecAllSentence.size()<<endl;
    for(int i=0; i<vecAllSentence.size();i++)
    {
        VECTOR_STRING vecFuncName;
        vecFuncName = Common::ToVectors(vecAllSentence[i]," ");
        ///读取FUNC函数，到endFUNC结束
        if(vecFuncName.size()==2 && m_programKeyWords[vecFuncName[0]]==FUNC)
        {
            m_mapFuncIndex[vecFuncName[1]] = m_vecFuncTable.size();
            vector<ProgramSentence> vecProgramSentence;
            for(;i<vecAllSentence.size();i++)
            {
                ProgramSentence OneProgramSentence;
                OneProgramSentence.strSentence = vecAllSentence[i];
                vecProgramSentence.push_back(OneProgramSentence);
                if(m_programKeyWords[vecAllSentence[i]] == ENDFUNC )
                {
                    break;
                }
            }
            m_vecFuncTable.push_back(vecProgramSentence);
           //m_vecFuncTable.insert()
        }
    }
	return 0;
}

int RobotInterpreter::writeFunction(const char *path, string funcName)
{
    fstream out;
    out.open(path,ios_base::out|ios_base::app);
    if(out.is_open())
    {
        auto iter = m_mapFuncIndex.find(funcName);
        if(iter!= m_mapFuncIndex.end())
        {
            const VECTOR_SENTENCE& vecSentence = m_vecFuncTable[(*iter).second];
            for(int i = 0; i<vecSentence.size(); i++)
            {
                out << vecSentence[i].strSentence<<endl;
            }
        }
        out.close();
    }
    return 0;
}

int RobotInterpreter::deleteFunction(string funcName)
{
    int nRt = ERROR_TRUE;
    auto iter = m_mapFuncIndex.find(funcName);
    if(iter!=m_mapFuncIndex.end())
    {
        int index = iter->second;
        m_mapFuncIndex.erase(iter);
        m_vecFuncTable.erase(m_vecFuncTable.begin()+index);
    }else {
        nRt = ERROR_NO_MATCH_FUNCTION;
        printInfo(INFO_WARNING,m_mapErrorInfo[nRt]);
    }
    return  nRt;
}

int RobotInterpreter::addProgramSentence(const ProgramPointer &pointer, const ProgramSentence &sen)
{
    int nRt=0;
    if((nRt=grammarCheck(sen.strSentence))==0)
    {
        int nFun = pointer.nFunction;
        int nSentence = pointer.nSentence;
        if(nFun<m_vecFuncTable.size()&&nSentence<m_vecFuncTable[nFun].size())
        {
            VECTOR_SENTENCE& vecSentence = m_vecFuncTable[nFun];
            vecSentence.insert(vecSentence.begin()+nSentence,sen);
        }
        //函数尾插入
        else if(nFun<m_vecFuncTable.size()&&nSentence==m_vecFuncTable[nFun].size())
        {
            VECTOR_SENTENCE& vecSentence = m_vecFuncTable[nFun];
            vecSentence.push_back(sen);
            //vecSentence.insert(vecSentence.begin()+nSentence,sen);
        }
        //添加新的函数
        else if(nSentence==0&&nFun==m_vecFuncTable.size())
        {
            VECTOR_STRING vecField = Common::ToVectors(sen.strSentence," ");
            if(vecField.size()==2&& vecField[0]=="FUNC")
            {

                auto iter = m_mapFuncIndex.find(vecField[1]);
                if(iter!=m_mapFuncIndex.end())
                {
                    nRt = ERROR_FUNCTION_REDEFINITION;
                }
                else
                {
                    VECTOR_SENTENCE vecNewFunc;
                    m_mapFuncIndex[vecField[1]] = nFun;
                    vecNewFunc.push_back(sen);
                    m_vecFuncTable.push_back(vecNewFunc);
                }
            }else if(sen.strSentence=="MAIN")
            {
                VECTOR_SENTENCE vecNewFunc;
                m_mapFuncIndex["MAIN"] = nFun;
                vecNewFunc.push_back(sen);
                m_vecFuncTable.push_back(vecNewFunc);
            }
        }else
        {
            nRt = ERROR_POINTER_ACCESS_VIOLATION;
        }
    }

    if(nRt)
    {
        ProgramPointer ptr = pointer;
        printInfo(INFO_WARNING, makePrintPrefix(&ptr)+m_mapErrorInfo[nRt]);
    }

    return nRt;
}

int RobotInterpreter::modifyProgramSentence(const ProgramPointer &pointer, ProgramSentence &sen)
{
    int nRt=0;
    if((nRt=grammarCheck(sen.strSentence))==0)
    {
        int nFun = pointer.nFunction;
        int nSentence = pointer.nSentence;
        if(nFun<m_vecFuncTable.size()&&nSentence<m_vecFuncTable[nFun].size())
        {
            VECTOR_SENTENCE& vecSentence = m_vecFuncTable[nFun];
            vecSentence[nSentence] = sen;
        }
        else
        {
            nRt = ERROR_POINTER_ACCESS_VIOLATION;
        }
    }

     if(nRt)
     {
         ProgramPointer ptr = pointer;
         printInfo(INFO_WARNING, makePrintPrefix(&ptr)+m_mapErrorInfo[nRt]);
     }

    return nRt;
}

int RobotInterpreter::deleteProgramSentence(const ProgramPointer &pointer)
{
    int nFun = pointer.nFunction;
    int nSentence = pointer.nSentence;
    if(nFun<m_vecFuncTable.size()&&nSentence<m_vecFuncTable[nFun].size())
    {
        VECTOR_SENTENCE& vecSentence = m_vecFuncTable[nFun];
        vecSentence.erase(vecSentence.begin()+nSentence);
    }else {
        ProgramPointer ptr = pointer;
        printInfo(INFO_WARNING, makePrintPrefix(&ptr)+m_mapErrorInfo[ERROR_POINTER_ACCESS_VIOLATION]);
        return  ERROR_POINTER_ACCESS_VIOLATION;
    }

    return 0;
}

ProgramSentence RobotInterpreter::getProgramSentence(const ProgramPointer &pointer)
{
    int nFuncIndex = pointer.nFunction;
    int nSentence = pointer.nSentence;

    if(nFuncIndex<m_vecFuncTable.size() && nSentence<m_vecFuncTable[nFuncIndex].size())
    {
        return m_vecFuncTable[nFuncIndex][nSentence];
    }
    else
    {
        printInfo(INFO_WARNING, m_mapErrorInfo[ERROR_POINTER_ACCESS_VIOLATION] );
        return ProgramSentence("");
    }
}

std::vector<std::vector<ProgramSentence> > RobotInterpreter::getFuncTable() const
{
    return m_vecFuncTable;
}

int RobotInterpreter::addVariable(string strVariable)
{
    int nRt=0;
    VECTOR_STRING vecField = Common::ToVectors(strVariable," ");
    if(vecField.size() >=2)
    {
        auto iter = m_programKeyWords.find(vecField[0]);
        if(iter != m_programKeyWords.end())
        {
            auto iterVar = m_mapVarValue.find(vecField[1]);
            if(iterVar != m_mapVarValue.end())
            {
                //重复定义的变量
                nRt = ERROR_VARIABLE_REDEFINITION;
            }
            else
            {
                m_mapVarType[vecField[1]] = iter->second;
                VECTOR_DOUBLE vecValue;
                for(int i=3;i< vecField.size();i++)
                {
                    vecValue.push_back(atof(vecField[i].c_str()));
                }
                m_mapVarValue[vecField[1]]=vecValue;
            }
        }else
        {
            //未知类型变量
            nRt = ERROR_INVALID_KEYWORD;
        }
    }
    if(nRt)
        printInfo(INFO_WARNING, m_mapErrorInfo[nRt]);
    return nRt;
}

int RobotInterpreter::delVariable(string strName)
{
    if(m_mapVarType.find(strName)!=m_programKeyWords.end())
    {
        m_mapVarType.erase(strName);
    }
    if(m_mapVarValue.find(strName)!=m_mapVarValue.end())
    {
        m_mapVarValue.erase(strName);
    }
    return ERROR_TRUE;
}

int RobotInterpreter::modifyVariable(string strVarName, string strVariable)
{
    delVariable(strVarName);
    return addVariable(strVariable);
}

int RobotInterpreter::getVariable(string strVarName, PROGRAM_KEY_WORD &type, VECTOR_DOUBLE &vecVal)
{
    if(m_mapVarType.find(strVarName)!=m_programKeyWords.end())
    {
        type = m_mapVarType[strVarName];
    }else
    {
        return ERROR_UNDEFINE_VARIABLE;
    }

    if(m_mapVarValue.find(strVarName)!=m_mapVarValue.end())
    {
        vecVal.clear();
        for(int i=0; i<m_mapVarValue[strVarName].size();i++)
        {
            vecVal.push_back(m_mapVarValue[strVarName][i]);
        }
    }
    return ERROR_TRUE;
}

std::map<string, PROGRAM_KEY_WORD> RobotInterpreter::getVariableMap() const
{
    return m_mapVarType;
}

void RobotInterpreter::clearFuncTable()
{
    for(int i=0;i<m_vecFuncTable.size();i++)
    {
        VECTOR_SENTENCE& vecSentence = m_vecFuncTable[i];
        vecSentence.clear();
    }
    m_vecFuncTable.clear();
    m_mapFuncIndex.clear();
}

void RobotInterpreter::clearVarTable()
{
    if(!m_mapVarType.empty())
    {
        m_mapVarType.clear();
    }

    if(!m_mapVarValue.empty())
    {
        std::map<std::string, std::vector<double>>::iterator iter;
        for(iter=m_mapVarValue.begin();iter!=m_mapVarValue.end();iter++)
        {
            (iter->second).clear();
        }
        m_mapVarValue.clear();
    }
}

std::map<string, PROGRAM_KEY_WORD> RobotInterpreter::getKeyWordIndex()
{
    return m_programKeyWords;
}

std::map<string, PROGRAM_OPERATOR> RobotInterpreter::getOpertorIndex()
{
    return m_programOperator;
}

std::map<string, int> RobotInterpreter::getFuncIndex()
{
    return m_mapFuncIndex;
}

void RobotInterpreter::setTeach(TEACH_TYPE type)
{
    m_teachType = type;
}

ProgramSentence RobotInterpreter::getOneProgramSentence(const ProgramPointer &pointer)
{
    while (!m_bStop)
    {
        if(m_bPause)
        {
			sleep_ms(100);
            continue;
        }
        int nFuncIndex = pointer.nFunction;
        int nSentence = pointer.nSentence;
//        if(m_debugState==SWITCHON)
//        {
//            if(m_bSingle)
//                m_bPause = true;
//        }
        setFlag();
        updateProgramPointer(m_Pointer);
        if(nFuncIndex<m_vecFuncTable.size() && nSentence<m_vecFuncTable[nFuncIndex].size())
        {
            return m_vecFuncTable[nFuncIndex][nSentence];
        }
        else
        {
            printInfo(INFO_WARNING,  makePrintPrefix()+m_mapErrorInfo[ERROR_POINTER_ACCESS_VIOLATION] );
            break;
        }
    }

    return ProgramSentence("ENDMAIN");
}

void RobotInterpreter::setFlag()
{
    if(m_debugState==SWITCHON)
    {
        if(m_Pointer.nFunction>=m_tmpPointer.nFunction
                &&m_Pointer.nSentence>=m_tmpPointer.nSentence)
        {
            m_tmpPointer.nFunction = 0;
            m_tmpPointer.nSentence = 0;
            if(m_bSingle)
                m_bPause = true;
        }
    }
}


