#include "robotInterpreter.h"
#include "robotFile.h"
#include "functions.h"
#include <sstream>
#include <string.h>
//#include <unistd.h>
#include <algorithm>

#define SPEEDDATA   "speeddata"
#define JSONFILE   PATH_SEP_STRING"var.json"
using namespace std;
using namespace rclib;
using namespace Common;
using namespace ABBInterpreter ;

void RobotInterpreter::runABB()
{
    //setPlayState(SYSPLAY_PLAY );
    clearFuncTable();
    m_FunMan.getFunctionTable(m_vecFuncTable,m_mapFuncIndex);
    m_pCurrFuninfo = m_FunMan.getFuncinfo("main");
    if(m_pCurrFuninfo)
    {
        if(m_bSingle)
        {
            stepRun();
        }else {
            run();
        }
    }else {
        printInfo(INFO_WARNING,"main函数没找到");
    }
    setTeach(HR);
}

int RobotInterpreter::readFunctionABB(const char *path)
{
    m_VarMan.clear();
    m_FunMan.clear();

    string strPath = GetModuleFullPath(true) ;
    if(!m_jsWrapper.getJsonPredefineValue((strPath+JSONFILE).c_str()))
    {
        printInfo(INFO_WARNING,"获取abb的预定义信息失败");
        return 0;
    }


    VECTOR_STRING vecAllSentence;
    Common::ReadFileToVector(path, vecAllSentence);
    VECTOR_STRING::iterator iter = vecAllSentence.begin();
    for (;iter!=vecAllSentence.end();iter++) {
        string strSentence = *iter;
        VECTOR_STRING vecKey = Common::ToVectors(strSentence, " ");
        if(vecKey.size()==0)
        {
            continue;
        }

        //读取一个模块
        if(vecKey[0]=="MODULE"&&vecKey.size()>=2)
        {
            string strModName = vecKey[1];
            int pos = strModName.find('(');
            if(pos!=strModName.npos)
            {
                strModName= strModName.substr(0, pos);
            }
            for (iter++;iter!=vecAllSentence.end()&&(*iter!="ENDMODULE");iter++) {
                VECTOR_STRING vecField = Common::ToVectors(*iter," ");
                //读取一个程序
                if(vecField.size()>0&&
                   (vecField[0]=="PROC"||vecField[0]=="FUNC"||vecField[0]=="TRAP"))
                {
                    FunctionInfo* funInfo = new FunctionInfo;
                    if(funInfo){
                        m_FunMan.addFuncHead(*iter,funInfo);
                        funInfo->strModuleName = strModName;
                        for (iter++;iter!=vecAllSentence.end();iter++) {
                            string strIter = *iter;
                            if(strIter.empty()||strIter.at(0)=='!')
                            {
                                continue;
                            }
                            ABB2HR(strIter);
                            funInfo->vecSentence.push_back(ProgramSentence(strIter));
                            if(strIter=="ENDFUNC" ||strIter=="ENDPROC"||strIter=="ENDTRAP")
                            {
                                break;
                            }
                            //程序局部变量
                            funInfo->addVariable(strIter);
                        }
                    }
                }else {
                    string strIter = *iter;
                    if(strIter.empty()||strIter.at(0)=='!')
                    {
                        continue;
                    }
                    //全局变量、模块变量
                    m_VarMan.addVariable(*iter,strModName);
                }
            }
        }
    }

}

int RobotInterpreter::ABB2HR(string &strSentence)
{
    strSentence.erase(strSentence.find_last_not_of(';')+1);
	Common::StringReplaceAll(strSentence, "(", "( ");
	Common::StringReplaceAll(strSentence, ")", " )");
    if(!strSentence.empty())
    {
        VECTOR_STRING vecField = Common::ToVectors(strSentence," ");
        switch (m_programKeyWords[vecField[0]]) {
        case IF:
        case ELSEIF:
        {
            strSentence.erase(strSentence.find_last_not_of("THEN")+1);
            break;
        }
        case WHILE:
        {
            strSentence.erase(strSentence.find_last_not_of("DO")+1);
            break;
        }
        }
    }
    return 0;
}

void RobotInterpreter::InitParamABB()
{
    m_programKeyWords["bool"] = BOOL;
    m_programKeyWords["num"] = NUM;
    m_programKeyWords["string"] = STRING;
    m_programKeyWords["LOCAL"] = LOCAL;
    m_programKeyWords["VAR"] = VAR;
    m_programKeyWords["orient"] = ORIENT;
    m_programKeyWords["PROC"] = PROC;
    m_programKeyWords["TRAP"] = TRAP;
    m_programKeyWords["FUNC"] = FUNC;

    m_programKeyWords["AccSet"] = ACCSET;
    m_programKeyWords["Add"] = ADD;
    m_programKeyWords["Comment"] = COMMENT;
    m_programKeyWords["MoveAbsJ"] = MOVEABSJ;
    m_programKeyWords["MoveC"] = MOVEC;
    m_programKeyWords["MoveJ"] = MOVEJ;
    m_programKeyWords["MoveL"] = MOVEL;
    m_programKeyWords["PluseDO"] = PULSEDO;
    m_programKeyWords["Set"] = SET;
    m_programKeyWords["SetDO"] = SETDO;
    m_programKeyWords["TEST"] = TEST;
    m_programKeyWords["VelSet"] = VELSET;
    m_programKeyWords["WaitDI"] = WAITDI;
    m_programKeyWords["WaitDO"] = WAITDO;
    m_programKeyWords["WaitTime"] = WAITTIME;

}

string RobotInterpreter::getOneSentence(pointABB &pPoint)
{
    return m_FunMan.getOneSentence(pPoint);
}

int RobotInterpreter::executeOneSentenceABB(const ProgramSentence &sen)
{
    vector<string> vecField = Common::ToVectors(sen.strSentence, " ");
    int nRt;
    if(vecField.size()>0)
    {
        switch (m_programKeyWords[vecField[0]]) {
        case FOR:
        {
            //
            //nRt = handleFor();
            nRt = handleForABB();
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

			//判断是否是程序调用
			if (IsProcCall(vecField))
			{
				nRt = handleCallABB(vecField);
			}
			else
			{
				nRt = handleOperationABB(sen.strSentence);
			}
            
            break;
        }
        }
    }
    else
    {
        printInfo(INFO_WARNING, "代码解析错误");
    }

    if(nRt>ERROR_OVERACCELERATION)
        printInfo(INFO_WARNING, makePrintPrefix()+m_mapErrorInfo[nRt]);

    return nRt;
}

int RobotInterpreter::handleForABB()
{
    const vector<ProgramSentence>& vecSentence = m_vecFuncTable[m_Pointer.nFunction] ;
    int nSentenceSerial = m_Pointer.nSentence;

    int nRt=ERROR_TRUE;
    vector<vector<int>> vecFor;      //存放for循环体的二维表
    if((nRt=CtreateBivariateTable("FOR","ENDFOR", vecFor))!= ERROR_TRUE)
    {
        printInfo(INFO_WARNING,m_mapErrorInfo[ERROR_SYNTAX_ERROR]);
        return nRt;
    }

    //FOR i FROM 10 TO 2 STEP -2 DO
    stack<ForCount> stackFor;           //记录for的执行
    //执行for循环 到for的下一行
    //for(int i = nSentenceSerial; i<vecFor[0][1]+1/*vecSentence.size()*/; i++,m_Pointer.nSentence=i)
    while(m_Pointer.nSentence < vecFor[0][1]+1)
    {
        int& i = m_Pointer.nSentence;
        ProgramSentence oneSentence = getOneProgramSentence(m_Pointer);
        vector<string> vecField = Common::ToVectors(oneSentence.strSentence," ");
        if(vecField.size()>0)
        {
            if( vecField[0]=="FOR")
            {
                ForCount forCount;
                for (int j=2;j<vecField.size();j++) {
                    if(vecField[j]=="FROM")
                    {
                        forCount.nBegin = string_to_int(vecField[++j]);
                        forCount.nWalker = string_to_int(vecField[j]);
                    }else if(vecField[j]=="TO")
                    {
                        forCount.nEnd = string_to_int(vecField[++j]);
                    }else if(vecField[j]=="STEP"){
                        forCount.nStep = string_to_int(vecField[++j]);
                    }

                }

                //forCount.nTotal = atoi(vecField[1].c_str());
                if(forCount.nEnd>forCount.nBegin)
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
//                forTop.nCompletedTime++;
//                if(forTop.nCompletedTime>=forTop.nTotal)
                if(forTop.step())
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
//                    forTop.nCompletedTime++;
//                    if(forTop.nCompletedTime>=forTop.nTotal)
                    if(forTop.step())
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

    return nRt;
}

int RobotInterpreter::handleWhileABB()
{
    return 0;
}

int RobotInterpreter::handleIfABB()
{
    return 0;
}

int RobotInterpreter::handleCallABB(vector<string>& vecF)
{
	int nRt = ERROR_TRUE;
	LP_FUNCTIONINFO pFunc = m_FunMan.getFuncinfo(vecF[0]);
	if (pFunc)
	{
		//修改当前函数
		m_pCurrFuninfo = pFunc;
		//填充调用程序的参数
		MAP_VARIABLEINFO& mapParam = pFunc->mapParam;
		auto iter = mapParam.begin();
		int l = vecF.size();
		if (mapParam.size() != vecF.size() - 1)
		{
			return ERROR_NO_MATCH_FUNCTION;
		}
		for (int i=1; i<l; i++,iter++)
		{
			LP_VARIABLEINFO pVar = iter->second ;
			if (pVar->strType == "num")
			{
				pVar->doubValue = atof(vecF[i].c_str());
			}
			else {
				pVar->strValue = vecF[i];
			}
		}
	}
	ProgramSentence oneProgramSentence = getOneProgramSentence(m_Pointer);
	vector<string> vecField = Common::ToVectors(oneProgramSentence.strSentence, " ");
	//if (vecField.size() == 2 && vecField[0] == "CALL")
	{
		m_funPointerStack.push(m_Pointer);
        int nFunc = m_mapFuncIndex[vecField[0]];
		m_Pointer.nFunction = nFunc;
		m_Pointer.nSentence = 0;

		vector<ProgramSentence> vecSentence = m_vecFuncTable[nFunc];
		for (auto iter = vecSentence.begin(); iter != vecSentence.end(); iter++/*,m_Pointer.nSentence++*/)
		{
			ProgramSentence oneNextProgramSentence = getOneProgramSentence(m_Pointer);
			if (oneNextProgramSentence.strSentence == "RETURN" || oneNextProgramSentence.strSentence == "ENDFUNC"
				|| oneNextProgramSentence.strSentence=="ENDPROC")
			{
				if (!m_funPointerStack.empty())
				{
					ProgramPointer& pFunc = m_funPointerStack.top();
					//pFunc
					//两个元素也遵从了入栈原则
					m_Pointer.nFunction = pFunc.nFunction;
					m_Pointer.nSentence = pFunc.nSentence + 1;
					m_funPointerStack.pop();
					auto iter=m_mapFuncIndex.begin();
					for (; iter!=m_mapFuncIndex.end(); iter++)
					{
						if (iter->second == m_Pointer.nFunction)
						{
							m_pCurrFuninfo = m_FunMan.getFuncinfo(iter->first);
							break;
						}
					}
				}
				break;
			}
#ifndef QT_NO_DEBUG
			cout << "call:" << oneNextProgramSentence.strSentence << endl;
#endif
			executeOneSentence(oneNextProgramSentence);
		}
	}
    return nRt;
}
bool RobotInterpreter::IsProcCall(VECTOR_STRING& vecField)
{
	LP_FUNCTIONINFO pFunc = m_FunMan.getFuncinfo(vecField[0]);
	if (!pFunc)
	{
        pFunc=nullptr;
		return false;
	}
	return true;
}

int  RobotInterpreter::arithmometerABB(const VECTOR_STRING& vecField)
{
	int nRt = ERROR_TRUE;
	string strType;		//右值类型
	LP_VARIABLEINFO pVar = m_VarMan.getVariable(vecField[0],m_pCurrFuninfo);
	if (pVar)
	{
		strType = pVar->strType;		
		if (strType=="num")
		{
			if (vecField.size() == 3)
			{
				if (Common::isnum(vecField[2]))
				{
					pVar->doubValue = string_to_double(vecField[2]);
				}
				else
				{
					double fValue;
					if (m_jsWrapper.getJsonValue("num", vecField[2], fValue))
					{
						pVar->doubValue = fValue;
					}
					else
					{
						LP_VARIABLEINFO pVarx = m_VarMan.getVariable(vecField[2], m_pCurrFuninfo);
						if (pVarx->strType == "num")
						{
							pVar->doubValue = pVarx->doubValue;
						}
					}
				}
			}
			else
			{
				string strExp;
				for (int i=2;i<vecField.size();i++)
				{
					if (vecField[i] == "(" || vecField[i] == ")"
						|| Common::isnum(vecField[i]) ||Common::CExpresion::isOperator(vecField[i][0]))
					{
						strExp += vecField[i] + " ";
					}
					else
					{
						double fValue;
						if (m_jsWrapper.getJsonValue("num", vecField[i], fValue))
						{
							strExp += num_to_string( fValue)+" ";
							continue;
						}
						else
						{
							LP_VARIABLEINFO pVarx;
							pVarx = m_VarMan.getVariable(vecField[i], m_pCurrFuninfo);
							if (pVarx)
							{
                                strExp += num_to_string(pVarx->doubValue) + " ";
								continue;
							}
						}
						return ERROR_SYNTAX_ERROR;
					}
				}
				CExpresion cExp;
				pVar->doubValue = cExp.getValue(strExp);
			}
			
		}
		else if(vecField.size()==3&&strType=="bool")
		{
			string strBoolValue = vecField[2];
			bool bValue;
			if (m_jsWrapper.getJsonValue("bool", vecField[2],bValue))
			{
				pVar->doubValue = bValue;
			}
			else
			{
				LP_VARIABLEINFO pVarx = m_VarMan.getVariable(vecField[2],m_pCurrFuninfo);
				if (pVarx)
				{
					strBoolValue = pVarx->strValue;
				}
				if (vecField[2] == "TRUE")
				{
					pVar->doubValue = 1;
				}
				else if (vecField[2] == "FALSE")
				{
					pVar->doubValue = 0;
				}
				else
				{
					nRt = ERROR_SYNTAX_ERROR;
				}
			}
		}		
	}
	else
	{
		nRt = ERROR_SYNTAX_ERROR;
	}
	return nRt;
}

bool RobotInterpreter::makeExpress(const VECTOR_STRING& vecSour, string& strDest)
{
    if(!strDest.empty())
    {
        return  false;
    }
    for (int i=0;i<vecSour.size();i++)
	{

		if (vecSour[i] == "(" || vecSour[i] == ")"
			|| Common::isnum(vecSour[i]) || Common::CExpresion::isOperator(vecSour[i][0]))
		{
			strDest += vecSour[i] + " ";
		}
		else
        {
            double doubValue;
			if (m_jsWrapper.getJsonValue("num", vecSour[i], doubValue))
			{
			}
			else
			{
				LP_VARIABLEINFO pVarx = m_VarMan.getVariable(vecSour[i], m_pCurrFuninfo);
				if (pVarx->strType == "num")
				{
					doubValue = pVarx->doubValue;
				}
				else
				{
					return false;
				}
            }
            strDest += num_to_string(doubValue)+" ";
		}

	}
	return true;
}

bool RobotInterpreter::IsJudgeOperator(string& str, PROGRAM_OPERATOR& op)
{
	auto iter = m_programOperator.find(str);
	if (iter != m_programOperator.end())
	{
		PROGRAM_OPERATOR tmpOp = iter->second;
		switch (tmpOp)
		{
		case rclib::OPERATOR_ASS:
		case rclib::OPERATOR_ASS1:
		case rclib::OPERATOR_EQU:
		case rclib::OPERATOR_NEQ:
		case rclib::OPERATOR_LAG:
		case rclib::OPERATOR_LES:
		case rclib::OPERATOR_GEQ:
		case rclib::OPERATOR_LEQ:
		{
			op = tmpOp;
			return true;
			break;
		}
			
		default:
			break;
		}
		
	}
	return false;
}

int RobotInterpreter::handleOperationABB(string strSentence)
{
    //MoveL Offs(p2, 0, 0, 10), v1000, z50, tool1;
    double nX = 0;
    double nY = 0;
    double nZ = 0;
    int npos = strSentence.find("Offs(");
    if(npos!=strSentence.npos)
    {
        int nPosR = strSentence.find(")",npos);
        if(nPosR!=strSentence.npos)
        {
            int n = sizeof ("Offs(");
            string strSub = strSentence.substr(npos+sizeof ("Offs(")-1,nPosR-npos-sizeof ("Offs("));
            vector<string> vecSub = Common::ToVectors(strSub, "," );
            if(vecSub.size()==4)
            {
                strSentence = strSentence.substr(0,npos)+vecSub[0]+strSentence.substr(nPosR+1);
                if(Common::isnum(vecSub[1]))
                {
                    nX = atof(vecSub[1].c_str());
                }else {
                    LP_VARIABLEINFO pVar = m_VarMan.getVariable(vecSub[1], m_pCurrFuninfo);
                    if(pVar&&pVar->strType=="num")
                    {
                        nX = atof(pVar->strValue.c_str());
                    }
                }
                if(Common::isnum(vecSub[2]))
                {
                    nY = atof(vecSub[2].c_str());
                }else {
                    LP_VARIABLEINFO pVar = m_VarMan.getVariable(vecSub[2], m_pCurrFuninfo);
                    if(pVar&&pVar->strType=="num")
                    {
                        nY = atof(pVar->strValue.c_str());
                    }
                }
                if(Common::isnum(vecSub[3]))
                {
                    nZ = atof(vecSub[3].c_str());
                }else {
                    LP_VARIABLEINFO pVar = m_VarMan.getVariable(vecSub[3], m_pCurrFuninfo);
                    if(pVar&&pVar->strType=="num")
                    {
                        nZ = atof(pVar->strValue.c_str());
                    }
                }
            }
        }
    }

    vector<string> vecField = Common::ToVectors(strSentence, " " );
    int nRt=ERROR_TRUE;
    if(grammarCheck(vecField))
    {
        return  ERROR_SYNTAX_ERROR;
    }
    switch (m_programKeyWords[vecField[0]]) {
    case BREAK:
    {
        m_bStop = true;
        break;
    }
    case ACCSET:
    {
        break;
    }
    case ADD:
    {
        //Add reg1, 3; Add reg1, -reg2;
        if(vecField.size()==3)
        {
            string &reg1 = vecField[1];
            reg1.erase(reg1.find_last_not_of(',')+1);
            LP_VARIABLEINFO pVar;
            pVar = m_VarMan.getVariable(reg1,m_pCurrFuninfo);
            if(pVar&&(pVar->strType=="num"||pVar->strType=="dnum"))
            {

                if(Common::isnum(vecField[2]))
                {
                    pVar->doubValue += string_to_double( vecField[2]);
                    break;
                }else
                {
                    string reg2 = vecField[2];
                    bool bPlus = true;
                    if(vecField[2][0]=='-')
                    {
                        bPlus = false;
                        reg2 = reg2.substr(1);
                    }

                    LP_VARIABLEINFO pVar2;
                    pVar2 = m_VarMan.getVariable(reg2,m_pCurrFuninfo);
                    if(pVar2&&(pVar2->strType=="num"||pVar->strType=="dnum"))
                    {
                        if(bPlus)
                        {
                            pVar->doubValue += pVar2->doubValue;
                        }else {
                            pVar->doubValue -= pVar2->doubValue;
                        }
                        break;
                    }
                }
            }
        }
        nRt = ERROR_UNDEFINE_VARIABLE;
        break;
    }
    case COMMENT:
    {
        break;
    }
    case PULSEDO:
    {
        break;
    }
    case SET:
    {
        //Set do15;
        //Set weldon;
        //Set Signal
        if(vecField.size()==2)
        {
            string str = vecField[1];
            str.erase(str.find_last_not_of(";")+1);
            std::transform(str.begin(), str.end(), str.begin(), ::toupper);
            std::map <std::string, PORTINDEX>::iterator iter = m_mapPortIndex.find(str);
            if(iter!=m_mapPortIndex.end())
            {
                PORTINDEX index = iter->second;
                if(getIOConnect()==SWITCHON){
                    setDigitalOutput(index,SWITCHON);
                }else {
                    nRt = ERROR_IO_CONNECT;
                }
                break;
            }
        }
        nRt  = ERROR_SYNTAX_ERROR;
        break;
    }
    case SETDO:
    {
        //SetDO do15, 1;
        if(vecField.size()==3)
        {
            string str = vecField[1];
            str.erase(str.find_last_not_of(',')+1);
            std::transform(str.begin(), str.end(), str.begin(), ::toupper);
            std::map <std::string, PORTINDEX>::iterator iter = m_mapPortIndex.find(str);
            if(iter!=m_mapPortIndex.end())
            {
                PORTINDEX index = iter->second;
                string strValue = vecField[2];
                int iValue =0;
                if(Common::isnum(strValue))
                {
                    iValue = string_to_int(strValue);
                }else
                {
                    LP_VARIABLEINFO pVar = m_VarMan.getVariable(strValue,m_pCurrFuninfo);
                    if(pVar)
                    {
                        iValue = (int)pVar->doubValue;
                    }
                }
                if(getIOConnect()==SWITCHON){
                    if (iValue) {
                        setDigitalOutput(index,SWITCHON);
                    }else {
                        setDigitalOutput(index,SWITCHOFF);
                    }
                }else {
                    nRt = ERROR_IO_CONNECT;
                }
                break;
            }
        }
        nRt  = ERROR_SYNTAX_ERROR;
        break;
    }
    case TEST:
    {
        break;
    }
    case VELSET:
    {
        break;
    }
    case WAITDO:
    case WAITDI:
    {
        //WaitDI di4, 1;
        string str = vecField[1];
        str.erase(str.find_last_not_of(',')+1);
        std::transform(str.begin(), str.end(), str.begin(), ::toupper);
        std::map <std::string, PORTINDEX>::iterator iter = m_mapPortIndex.find(str);
        if(iter!=m_mapPortIndex.end())
        {
            PORTINDEX index = iter->second;
            if(getIOConnect()==SWITCHON){
                while(true)
                {
                    sleep_ms(1);
                    if(m_programKeyWords[vecField[0]]==WAITDI)
                    {
                        if(SWITCHON==getDigitalInput(index))
                        {
                            break;
                        }
                    }else {
                        if(SWITCHON==getDigitalOutput(index))
                        {
                            break;
                        }
                    }

                }
            }else {
                nRt = ERROR_IO_CONNECT;
            }
        }

        break;
    }
    case WAITTIME:
    {
        //WaitTime 0.5;
        if(vecField.size()==2)
        {
            double t =0;
            string strTime = vecField[1];
            if(Common::isnum(strTime))
            {
                t = string_to_double(strTime);
            }else
            {
                LP_VARIABLEINFO pVar = m_VarMan.getVariable(strTime,m_pCurrFuninfo);
                if(pVar)
                {
                    t = pVar->doubValue;
                }
            }
            sleep_ms(t*1000);
        }
        break;
    }
    case MOVEABSJ:
    {
        //【MOVEABSJ】【速度（0-1）】【关节路点1】【关节路点2】
        //MoveAbsJ jpos10\NoEOffs, v1000, z50, tool0;
        if(vecField.size()==5)
        {
            string strJpos = vecField[1];
            string strV = vecField[2];
            strJpos.erase(strJpos.find_first_of('\\'));
            strJpos.erase(strJpos.find_first_of(','));
            strV.erase(strV.find_first_of('\\'));
            strV.erase(strV.find_first_of(','));
            string strSpeed;

            LP_VARIABLEINFO pVar = m_VarMan.getVariable(strJpos,m_pCurrFuninfo);
            if(pVar)
            {
                string strValue = pVar->strValue;
                Joints joint;
                if(jsonWrapper::jointtargetToJoints(strValue,joint))
                {
                    //获取速度
                    if(!m_jsWrapper.getJsonValue(SPEEDDATA,strV,strSpeed))
                    {
                        LP_VARIABLEINFO pVarV = m_VarMan.getVariable(strV, m_pCurrFuninfo);
                        if(pVarV)
                        {
                            strSpeed = pVarV->strValue;
                        }else {
                            nRt = ERROR_UNDEFINE_VARIABLE;
                            break;
                        }
                    }
                    double fV ;
                    if(!jsonWrapper::speedToDouble(strSpeed,fV))
                    {
                        break;
                    }

                    if(m_bConStartFlag)
                    {
                        m_bConStartFlag = false;
                        moveEndCon();
                        waitCommandEnd();
                    }

                    fV = 0.8;
                    nRt = moveABSJoint(joint,fV);

                    if(nRt==0)
                    {
                       nRt = waitCommandEnd();
                    }
                }
            }
        }
        break;
    }
    case MOVEC:
    {
        //MoveC p1, p2, v500, z30, tool2;
        if(vecField.size()==6)
        {
            string strP1 = vecField[1];
            string strP2 = vecField[1];
            string strV = vecField[3];
            strP1.erase(strP1.find_last_not_of(',')+1);
            strP2.erase(strP2.find_last_not_of(',')+1);
            strV.erase(strV.find_last_not_of(',')+1);

            LP_VARIABLEINFO pVar1 = m_VarMan.getVariable(strP1, m_pCurrFuninfo);
            LP_VARIABLEINFO pVar2 = m_VarMan.getVariable(strP2, m_pCurrFuninfo);
            string strSpeed;
            if(pVar1&&pVar2)
            {
                string strPoint1 = pVar1->strValue;
                string strPoint2 = pVar2->strValue;
                Terminal ter1,ter2;
                bool bOK = jsonWrapper::robtargatToTerminal(strP1,ter1);
                bOK &= jsonWrapper::robtargatToTerminal(strP2,ter2);
                if(bOK)
                {
                    //获取速度
                    if(!m_jsWrapper.getJsonValue(SPEEDDATA,strV,strSpeed))
                    {
                        LP_VARIABLEINFO pVarV = m_VarMan.getVariable(strV, m_pCurrFuninfo);
                        if(pVarV)
                        {
                            strSpeed = pVarV->strValue;
                        }else {
                            nRt = ERROR_UNDEFINE_VARIABLE;
                            break;
                        }
                    }
                    double fV ;
                    if(!jsonWrapper::speedToDouble(strSpeed,fV))
                    {
                        break;
                    }

                    double acc = 0.5;
                    double jerk = 0.3;
                    if(m_debugState == SWITCHON)
                    {
                        nRt = moveCircle(PART_CIRCLE,ter1,ter2,fV,acc,jerk);

                        if(nRt == 0)
                        {
                            nRt = waitCommandEnd();
                        }
                    }else
                    {
                        //virtual int moveCircleCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE){}   // 连续轨迹圆弧运动
                        //virtual int moveCircleRCon(CIRCLETYPE cir, const Terminal& p1, const Terminal& p2, double vel, double acc = 0.8, double jerk = 0.8, double turn = 0.3, COORDINATESYSTEM frame = COORDINATE_BASE){}  // 连续轨迹相对圆弧运动
                        if(!m_bConStartFlag)
                        {
                            m_bConStartFlag = true;
                            moveStartCon();
                        }
                        if(nRt = moveCircleCon(PART_CIRCLE,ter1,ter2,fV,acc,jerk))//   // 连续轨迹圆弧运动
                        {
                            m_bConStartFlag = false;
                            moveEndCon();
                            waitCommandEnd();
                        }
                    }

                }
            }

            }
        break;
    }
    case MOVEJ:
    {
        //MoveJ p1, vmax, z30, tool2;
        //MoveJ *, vmax \T:=5, fine, grip3;
        if(vecField.size()==5)
        {
            string strP = vecField[1];
            string strV = vecField[2];
            strP.erase(strP.find_last_not_of(',')+1);
            strV.erase(strV.find_last_not_of(',')+1);
            LP_VARIABLEINFO pVar = m_VarMan.getVariable(strP, m_pCurrFuninfo);
            string strSpeed;
            if(pVar)
            {
                //转换路点
                //Quaternion

                Json::Value jsPoint;
                string strPoint = pVar->strValue;
                Terminal ter;
                if(!jsonWrapper::robtargatToTerminal(strPoint,ter))
                {
                    break;
                }

                //获取速度
                if(!m_jsWrapper.getJsonValue(SPEEDDATA,strV,strSpeed))
                {
                    LP_VARIABLEINFO pVarV = m_VarMan.getVariable(strV, m_pCurrFuninfo);
                    if(pVarV)
                    {
                        strSpeed = pVarV->strValue;
                    }else {
                        nRt = ERROR_UNDEFINE_VARIABLE;
                        break;
                    }
                }
                double fV ;
                if(!jsonWrapper::speedToDouble(strSpeed,fV))
                {
                    break;
                }
                if(m_bConStartFlag)
                {
                    m_bConStartFlag = false;
                    moveEndCon();
                     waitCommandEnd();
                }
                fV = 0.8;
                nRt = moveJoint(ter,fV);
                if(nRt == 0)
                {
                    nRt = waitCommandEnd();
                }
            }
        }
        break;
    }
    case MOVEL:
    {
        //MoveL Offs(p2, 0, 0, 10), v1000, z50, tool1;
        //MoveL p40, v1000, z50, tool0;
        if(vecField.size()==5)
        {
            string strP = vecField[1];
            string strV = vecField[2];
            strP.erase(strP.find_last_not_of(',')+1);
            strV.erase(strV.find_last_not_of(',')+1);
            LP_VARIABLEINFO pVar = m_VarMan.getVariable(strP, m_pCurrFuninfo);
            string strSpeed;
            if(pVar)
            {
                //转换路点
                //Quaternion

                Json::Value jsPoint;
                string strPoint = pVar->strValue;
                Terminal ter;
                if(!jsonWrapper::robtargatToTerminal(strPoint,ter))
                {
                    break;
                }
                ter[TERMINAL_X] += nX;
                ter[TERMINAL_Y] += nY;
                ter[TERMINAL_Z] += nZ;

                //获取速度
                if(!m_jsWrapper.getJsonValue(SPEEDDATA,strV,strSpeed))
                {
                    LP_VARIABLEINFO pVarV = m_VarMan.getVariable(strV, m_pCurrFuninfo);
                    if(pVarV)
                    {
                        strSpeed = pVarV->strValue;
                    }else {
                        nRt = ERROR_UNDEFINE_VARIABLE;
                        break;
                    }
                }
                double fV ;
                if(!jsonWrapper::speedToDouble(strSpeed,fV))
                {
                    break;
                }

                double acc = 0.5;
                double jerk =0.2;

                if(m_debugState == SWITCHON)
                {
                    nRt = moveLine(ter,fV,acc,jerk);
                    if(nRt==0)
                    {
                        nRt = waitCommandEnd();
                    }
                }else {
                    if(!m_bConStartFlag)
                    {
                        m_bConStartFlag = true;
                        moveStartCon();
                    }
                    if(nRt = moveLineCon(ter,fV,acc,jerk))
                    {
                        m_bConStartFlag = false;
                        moveEndCon();
                        waitCommandEnd();
                    }

                }
            }else {
                nRt = ERROR_UNDEFINE_VARIABLE;
                break;
            }
        }
        break;
    }
    default:
    {
		if (vecField.size()>2&&vecField[1]==":=")
		{
			nRt = arithmometerABB(vecField);
		}
        break;
    }

    }
    // 指针后移
    m_Pointer.nSentence++;
    return nRt;
}

bool RobotInterpreter::formulaComputingABB(string strFormula)
{
	//WHILE (reg1 + reg4) + reg1 = reg2 + reg3
	VECTOR_STRING vecField,vecS1, vecS2;
	bool bS2 = false;
	PROGRAM_OPERATOR op;

    vecField = Common::ToVectors(strFormula, " ");
    if(vecField.size()==2)
    {
        ///TRUE OR FALSE
        if(vecField[1]=="TRUE"){
            return  true;
        }else if (vecField[1]=="FALSE") {
            return  false;
        }
        ///变量
        LP_VARIABLEINFO pVar = m_VarMan.getVariable(vecField[1],m_pCurrFuninfo);
        if(pVar&&pVar->strType=="bool")
        {
            return  (int)(pVar->doubValue);
        }
        return false;
    }
	for (int i=1;i<vecField.size();i++)
	{
		if (IsJudgeOperator(vecField[i],op))
		{
			bS2 = true;
            continue;
		}
		if (bS2)
		{
			vecS2.push_back(vecField[i]);
		}
		else
		{
			vecS1.push_back(vecField[i]);
		}
	}

	string strExpL, strExpR;
	if (makeExpress(vecS1, strExpL) && makeExpress(vecS2, strExpR))
	{
		Common::CExpresion cExp;
		double op1,op2;
		op1 = cExp.getValue(strExpL);
		op2 = cExp.getValue(strExpR);
		return bCompare(op1, op2, op);
	}

    return  true;
}

int RobotInterpreter::grammarCheck(vector<string> &vecField)
{
    int nRt=0;
    switch (m_programKeyWords[vecField[0]]) {
    case ACCSET:
    {
        break;
    }
    case ADD:
    {
        //Add reg1, 3; Add reg1, -reg2;
        if(vecField.size()==3)
        {
            string& strParam1 = vecField[1];
            strParam1.erase(strParam1.find_last_not_of(',')+1);
            if(m_VarMan.getVariable(vecField[1],m_pCurrFuninfo))
            {

            }
        }

        break;
    }
    case COMMENT:
    {
        break;
    }
    case PULSEDO:
    {
        break;
    }
    case SET:
    {
        break;
    }
    case SETDO:
    {
        break;
    }
    case TEST:
    {
        break;
    }
    case VELSET:
    {
        break;
    }
    case WAITDI:
    {
        break;
    }
    case WAITDO:
    {
        break;
    }
    case WAITTIME:
    {
        break;
    }
    default:
    {
        break;
    }

    }
	return nRt;
}

string RobotInterpreter::makePrintPrefix( ProgramPointer*  pointer)
{
    ostringstream oStr;
    int a,b;
    if(pointer)
    {
        a = pointer->nFunction ;
        b = pointer->nSentence ;
    }else {
        a = m_Pointer.nFunction;
        b = m_Pointer.nSentence - 1;
    }


    if(m_language==ENGLISH)
    {
        oStr << "The position error:["<<m_Pointer.nFunction
             << ","<< m_Pointer.nSentence<<"],Error details:";
    }else {
        oStr << "出错位置:["<< a
             << ","<< b <<"],出错详情:";
    }
    return  oStr.str();
}

FunctionInfo::FunctionInfo()
{

}

FunctionInfo::~FunctionInfo()
{
    auto iter = mapVar.begin();
    for(;iter!=mapVar.end();iter++)
    {
        LP_VARIABLEINFO pVar = iter->second;
        if(pVar)
        {
            delete pVar;
            pVar = NULL;
        }
    }
    mapVar.clear();

    iter = m_mapVar.begin();
    for(;iter!=m_mapVar.end();iter++)
    {
        LP_VARIABLEINFO pVar = iter->second;
        if(pVar)
        {
            delete pVar;
            pVar = NULL;
        }
    }
    m_mapVar.clear();

    iter = mapParam.begin();
    for(;iter!=mapParam.end();iter++)
    {
        LP_VARIABLEINFO pVar = iter->second;
        if(pVar)
        {
            delete pVar;
            pVar = NULL;
        }
    }
    mapParam.clear();
}

int  ABBInterpreter::FunctionInfo::addVariable(string strInfo)
{
    //VAR num T_Var7:=0;
    string strName;     //变量名
    string strValue;    //变量值
    string strAS;       //作用域
    string strType;     //变量类型
    string strAttri;    //变量属性

    strInfo.erase(strInfo.find_last_not_of(';')+1);
    int nPos = strInfo.find(":=");
    if(nPos != string::npos)
    {
        strValue = strInfo.substr(nPos+sizeof (":=")-1);
        int nPosBlank = strInfo.rfind(" ", nPos);
        if(nPosBlank!=string::npos)
        {
            strName = strInfo.substr(nPosBlank, nPos -nPosBlank);
            strInfo = strInfo.substr(0, nPosBlank);
        }
    }else {
        return  ERROR_SYNTAX_ERROR;
    }

    //          VAR  num gVar:=0;
    VECTOR_STRING vecField = Common::ToVectors(strInfo, " ");
    if(vecField.size()==2)
    {
        strAS = "FUNC";
        strAttri = vecField[0];
        strType = vecField[1];
    }
    else
    {
        return  ERROR_SYNTAX_ERROR;
    }


    LP_VARIABLEINFO pVar=NULL;
    auto iter = m_mapVar.find(strName);
    if(iter!=m_mapVar.end())
    {
        //变量已存在
    }else
    {
        pVar = new VARIABLEINFO;
        pVar->strName = strName;
        pVar->strType = strType;
        pVar->strValue = strValue;
        m_mapVar.insert(make_pair(strName,pVar));
    }

    return  ERROR_TRUE;
}

LP_VARIABLEINFO ABBInterpreter::FunctionInfo::getVariable(string strInfo)
{
	auto iter = m_mapVar.find(strName);
	if (iter != m_mapVar.end())
	{
		return iter->second;
	}
	return nullptr;
}

VarManager::~VarManager()
{
    clear();
}

int VarManager::addVariable(string strInfo, string strModName)
{   
    string strName;     //变量名
    string strValue;    //变量值
    string strAS;       //作用域
    string strType;     //变量类型
    string strAttri;    //变量属性

    strInfo.erase(strInfo.find_last_not_of(";")+1);
    int nPos = strInfo.find(":=");
    if(nPos != string::npos)
    {
        strValue = strInfo.substr(nPos+sizeof (":=")-1);
        strInfo=strInfo.substr(0,nPos);
        strInfo.erase(strInfo.find_last_not_of(" ")+1);
        int nPosBlank = strInfo.rfind(" ",nPos);
        if(nPosBlank!=string::npos)
        {
            strName = strInfo.substr(nPosBlank+1, nPos -nPosBlank-1);
            strInfo = strInfo.substr(0, nPosBlank);
        }
    }else {
        return  ERROR_SYNTAX_ERROR;
    }

    //          VAR  num gVar:=0;
    //    LOCAL VAR  num lVar1:=0;
    //          PERS num lVar2:=0;
    //          CONST num gConst3:=0;
    //    TASK  VAR num T_Var6:=0;
    bool bGloble = true;
    VECTOR_STRING vecField = Common::ToVectors(strInfo, " ");
    if(vecField.size()==3)
    {
        strAS = vecField[0];
        strAttri = vecField[1];
        strType = vecField[2];
        bGloble = false;
    }else if(vecField.size()==2)
    {
        strAS = "GLOBLE";
        strAttri = vecField[0];
        strType = vecField[1];
    }
    else
    {
        return  ERROR_SYNTAX_ERROR;
    }


    LP_VARIABLEINFO pVar=NULL;
    //全局变量
    if(bGloble)
    {
        MAP_VARIABLEINFO::iterator iter;
        iter = m_mapGlobleVar.find(strName);
        if(iter!=m_mapGlobleVar.end())
        {
            pVar = iter->second;
        }else {
            pVar = new VARIABLEINFO;
            if(pVar)
            {
                pVar->strActionScope=GLOBAL;
                m_mapGlobleVar.insert(make_pair(strName,pVar));
            }
        }
    }else //    局部
    {
        map<string, MAP_VARIABLEINFO>::iterator iter;
        iter = m_mapModuleVar.find(strModName);
        if(iter!=m_mapModuleVar.end() )
        {
            MAP_VARIABLEINFO& mapVar = iter->second;
            MAP_VARIABLEINFO::iterator iterF = mapVar.find(strName);
            if(iterF==mapVar.end())
            {
                pVar = new VARIABLEINFO;
                if(pVar)
                {
                    pVar->strActionScope=MODULUS;
                    mapVar.insert(make_pair(strName,pVar));
                }
            }

        }else {
            MAP_VARIABLEINFO mapVar;
            pVar = new VARIABLEINFO;
            if(pVar)
            {
                pVar->strActionScope=MODULUS;
                mapVar.insert(make_pair(strName,pVar));
                m_mapModuleVar.insert(make_pair(strModName,mapVar));
            }
        }
    }
    if(pVar)
    {
        pVar->strName = strName;
        pVar->strType = strType;
        pVar->strAttribute = strAttri;
        pVar->strValue = strValue;
        pVar->strAS = strAS;
        //暂时把值存放到string里面
        if(strType =="num")
        {
            //pVar->vecDoubleValue.push_back(atof(vecField[3].c_str()));
			pVar->doubValue = string_to_double(strValue);
		}
        else if (strType == "bool"){
            if (strValue == "TRUE")
            {
                pVar->doubValue = 1;
            }
            else if (strValue == "FALSE")
            {
                pVar->doubValue = 0;
            }
        }
    }

}
MAP_VARIABLEINFO VarManager::getVariable(ActionScope type, string strModName)
{
    switch (type) {
    case GLOBAL:{
        return m_mapGlobleVar;
    }
    case MODULUS:{
        return m_mapModuleVar[strModName];
    }
    case FUNCTION:{
        return m_mapFuncVar[strModName];
    }
    default:
    {
        return m_mapGlobleVar;
    }
    }
}

LP_VARIABLEINFO VarManager::getVariable(string strName,LP_FUNCTIONINFO pFunc)
{
    //m_pCurre
    LP_VARIABLEINFO pVar = NULL;
    //先遍历当前程序中变量，再模块中变量，再全局变量

    if(pFunc)
    {
		pVar = pFunc->getVariable(strName);
		if (pVar)
		{
			return pVar;
		}
        string strModuleName = pFunc->strModuleName;
        string strFuncName = pFunc->strName;
        map<string, MAP_VARIABLEINFO>::iterator iterM;
        MAP_VARIABLEINFO::iterator iter ;
        iterM = m_mapFuncVar.find(strFuncName);
        if(iterM!=m_mapFuncVar.end())
        {
            MAP_VARIABLEINFO& mapVar = iterM->second;
            iter = mapVar.find(strName);
            if(iter!= mapVar.end())
            {
                pVar = iter->second;
            }
        }else {
            iterM = m_mapModuleVar.find(strModuleName);
            if(iterM!=m_mapModuleVar.end())
            {
                MAP_VARIABLEINFO& mapVar = iterM->second;
                iter = mapVar.find(strName);
                if(iter!= mapVar.end())
                {
                    pVar = iter->second;
                }
            }
        }

    }
    if(!pVar)
    {
        MAP_VARIABLEINFO::iterator iter ;
        iter = m_mapGlobleVar.find(strName);
        if(iter!=m_mapGlobleVar.end())
        {
            pVar = iter->second;
        }
    }

    return  pVar;
}

void VarManager::clear()
{
    MAP_VARIABLEINFO::iterator iter1;
    for (iter1=m_mapGlobleVar.begin();iter1!=m_mapGlobleVar.end();iter1++) {
        LP_VARIABLEINFO pVarInfo = iter1->second;
        if(pVarInfo){
            delete pVarInfo;
        }
    }
    m_mapGlobleVar.clear();
    map<string,MAP_VARIABLEINFO>::iterator iter;
    for (iter=m_mapModuleVar.begin();iter!=m_mapModuleVar.end();iter++) {
        for (iter1=(iter->second).begin();iter1!=(iter->second).end();iter1++) {
            LP_VARIABLEINFO pVarInfo = iter1->second;
            if(pVarInfo){
                delete pVarInfo;
            }
        }
        iter->second.clear();
    }
    for (iter=m_mapFuncVar.begin();iter!=m_mapFuncVar.end();iter++) {
        for (iter1=(iter->second).begin();iter1!=(iter->second).end();iter1++) {
            LP_VARIABLEINFO pVarInfo = iter1->second;
            if(pVarInfo){
                delete pVarInfo;
            }
        }
        iter->second.clear();
    }
}

FunctionManager::FunctionManager()
{
    initParam();
}

FunctionManager::~FunctionManager()
{
    clear();
}

void FunctionManager::clear()
{
    MAP_FUNCINFO::iterator iter;
    for (iter=m_mapFuncIndex.begin();iter!=m_mapFuncIndex.end();iter++) {
        LP_FUNCTIONINFO pFuncInfo = iter->second;
        if(pFuncInfo){
            delete pFuncInfo;
        }
    }
    for (iter=m_mapProcIndex.begin();iter!=m_mapProcIndex.end();iter++) {
        LP_FUNCTIONINFO pFuncInfo = iter->second;
        if(pFuncInfo){
            delete pFuncInfo;
        }
    }
    for (iter=m_mapTrapIndex.begin();iter!=m_mapTrapIndex.end();iter++) {
        LP_FUNCTIONINFO pFuncInfo = iter->second;
        if(pFuncInfo){
            delete pFuncInfo;
        }
    }
    m_mapFuncIndex.clear();
    m_mapProcIndex.clear();
    m_mapTrapIndex.clear();
}

void FunctionManager::addFuncHead(string strInfo, LP_FUNCTIONINFO pFuncInfo)
{
    if(pFuncInfo)
    {
        pFuncInfo->vecSentence.push_back(ProgramSentence(strInfo));
        //解析strInfo 参数信息
        strInfo.erase(0, strInfo.find_first_not_of(" \t\r\n"));
        strInfo.erase(strInfo.find_last_not_of(" \t\r\n")+1);
        string strParam;
        //解析参数
        string::size_type nPosL = strInfo.find('(');
        if(nPosL!=strInfo.npos)
        {
            string::size_type nPosR = strInfo.find(')', nPosL);
            if(nPosR!=strInfo.npos)
            {
                if(nPosR-nPosL>1)
                {
                    strParam = strInfo.substr(nPosL+1,nPosR-nPosL-1);
                }
                strInfo = strInfo.substr(0,nPosL);
            }
        }

        //解析参数
        //(num param0,INOUT num param1,VAR num param2,PERS num param3)
        //(num param0,\num param1 | num param2)
        VECTOR_STRING vecParam = Common::ToVectors(strParam,",");
        for(int i=0;i<vecParam.size();i++)
        {
            LP_VARIABLEINFO pVar = new VARIABLEINFO;
            if(pVar)
            {
                if(vecParam[i].find('\\')!=string::npos)
                {
                    //可选参数
                }else
                {
                    VECTOR_STRING vecElement = Common::ToVectors(vecParam[i]," ");
                    if(vecElement.size()==2)
                    {
                        pVar->strAttribute = "IN";
                        pVar->strType = vecElement[0];
                        pVar->strName = vecElement[1];
                    }else {
                        pVar->strAttribute = vecElement[0];
                        pVar->strType = vecElement[1];
                        pVar->strName = vecElement[2];
                    }
                }

                pFuncInfo->mapParam.insert(make_pair(pVar->strName,pVar));
            }
        }


        //解析函数名
//        PROC Routine7();
//        FUNC num Routine8();
//        TRAP Routine9;
//        LOCAL PROC Routine10();

        VECTOR_STRING vecField = Common::ToVectors(strInfo, " ");
        int i=0;
        string strType;     //程序类型
        string strAS;       //作用域
        string strRt;       //返回值
        string strName;     //函数名
        if(vecField.size()==2)
        {
            strType = vecField[0];
            strName = vecField[1];
        }else if(vecField.size()==3){
            if(vecField[0]=="LOCAL")
            {
                strType = vecField[1];
            }else {
                strType = vecField[0];
                strRt = vecField[1];
            }
            strName = vecField[2];
        }else if(vecField.size()==4)
        {
            strAS = vecField[0];
            strType = vecField[1];
            strRt = vecField[2];
            strName = vecField[3];
        }

        pFuncInfo->strName = strName;
        pFuncInfo->strAS = strAS;
        pFuncInfo->strType = strType;
        pFuncInfo->strRetType = strRt;
        m_mapFuncIndex.insert(make_pair(pFuncInfo->strName,pFuncInfo));
    }
}

string FunctionManager::getOneSentence(pointABB &pPoint)
{
    MAP_FUNCINFO::iterator iter = m_mapFuncIndex.find(pPoint.strFunName);

    if(iter!=m_mapFuncIndex.end())
    {
        VECTOR_SENTENCE & vecSentence = iter->second->vecSentence;
        if(vecSentence.size()<pPoint.iNum)
        {
            return vecSentence[pPoint.iNum].strSentence;
        }
    }
    return "";
}

void FunctionManager::getFunctionTable(vector<VECTOR_SENTENCE> & vecFunTable, std::map<string, int> &mapFunIndex)
{
    //MAP_FUNCINFO m_mapFuncIndex;       //功能
    MAP_FUNCINFO::reverse_iterator  iter;

//    //将main函数放在首位
//    iter = m_mapFuncIndex.find("main");
//    if(iter!=m_mapFuncIndex.end())
//    {
//        VECTOR_SENTENCE& vecSentence = iter->second->vecSentence;
//        vecFunTable.push_back(vecSentence);
//        mapFunIndex.insert(make_pair(iter->first,vecFunTable.size()-1));
//    }

    iter=m_mapFuncIndex.rbegin();
    for (;iter!=m_mapFuncIndex.rend();iter++) {
        VECTOR_SENTENCE& vecSentence = iter->second->vecSentence;
        vecFunTable.push_back(vecSentence);
        mapFunIndex.insert(make_pair(iter->first,vecFunTable.size()-1));
    }
}

LP_FUNCTIONINFO FunctionManager::getFuncinfo(string strName, int index)
{
    LP_FUNCTIONINFO pFunc = NULL;
    MAP_FUNCINFO::iterator iter ;
    switch (index) {
    case 0:
        iter = m_mapFuncIndex.find(strName);
        if(iter!=m_mapFuncIndex.end())
        {
            pFunc = iter->second;
        }
        break;
    case 1:
        iter = m_mapProcIndex.find(strName);
        if(iter!=m_mapProcIndex.end())
        {
            pFunc = iter->second;
        }
        break;
    case 2:
        iter = m_mapTrapIndex.find(strName);
        if(iter!=m_mapTrapIndex.end())
        {
            pFunc = iter->second;
        }
        break;
    default:
        break;
    }

    return pFunc;
}

void FunctionManager::addFunc(string strName,  LP_FUNCTIONINFO pFuncInfo)
{
    switch (pFuncInfo->funType) {
    case PROC:{
        m_mapProcIndex.insert(make_pair(strName,pFuncInfo));
        break;
    }
    case FUNC:{
        m_mapFuncIndex.insert(make_pair(strName,pFuncInfo));
        break;
    }
    case TRAP:{
        m_mapTrapIndex.insert(make_pair(strName,pFuncInfo));
        break;
    }
    default:{
        cout << "add falss"<<endl;
        break;
    }


    }
}

void FunctionManager::initParam()
{
    m_programKeyWords["bool"] = BOOL;
    m_programKeyWords["num"] = NUM;
    m_programKeyWords["string"] = STRING;
    m_programKeyWords["LOCAL"] = LOCAL;
    m_programKeyWords["VAR"] = VAR;
    m_programKeyWords["PERS"] = PERS;
    m_programKeyWords["orient"] = ORIENT;
}
