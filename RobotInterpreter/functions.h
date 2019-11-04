#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include<iostream>
#include <string>
#include <list>
#include <vector>
#include <stack>
#include <sstream>
#include <string.h>
#include <memory>
#include "systemLib.h"
using namespace std;
namespace Common {

typedef unsigned long       DWORD;
typedef vector<string> VECTOR_STRING;
typedef std::list<DWORD> LIST_DWORD;
typedef list<string> LIST_STRING;


// 数组元素的大小
#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))

// 空字符串时使用
#define NULL_STRING _T("")
#define SAFE_STRING(STR) (STR!=NULL?STR:NULL_STRING)


///	缓冲区尺寸
#define STANDARD_SIZE		1024
#define PAGE_SIZE			4096
#define MAX_SIZE			65536

#define ST_LIVE_NORMAL 0	///< 普通状态
#define ST_LIVE_DELETE 1	///< 删除状态

/// 秒转换
#define		SEC_TIME(X)		(X)
#define 	MIN_TIME(X)		SEC_TIME((X)*60)
#define 	HOUR_TIME(X)	MIN_TIME((X)*60)
#define 	DAY_TIME(X)		HOUR_TIME((X)*24)

#define		MUL_1000(X)		((X)*1000)
#define		DIV_1000(X)		((X)/1000)

#define _T(x)               x
#define NEW_LINE			_T("\r\n")		///< 标准换行
#define LOG_NEW_LINE		_T("\n\n")		///< LOG标准换行
#define LOG_SEPARATOR		_T("<>")		///< 日志的分割符
#define POLICY_SEPARATOR	_T("\r\n\n\n")	///< 策略分割符号
#define SMP_SEPARATOR		_T("\1\1")		///< 不可见分割符号

#define EQUAL_STRING		_T("=")
#define LEFT_MIDDLE_BRACKET_STRING	_T("[")
#define RIGHT_MIDDLE_BRACKET_STRING _T("]")
#define LEFT_BRACKET_STRING _T("(")
#define RIGHT_BRACKET_STRING _T(")")
#define TAB_STRING			_T("\t")
#define COLON_STRING		_T(":")
#define SEMICOLON			_T(";")
#define COMMA				_T(",")
#define BLANKSPACE          _T(" ")
#define BIG_BLANKSPACE		_T("　")
#define DOUBLE_QUOTATION    _T("\"")


//路径分隔符
#ifdef _WIN32
#define PATH_SEPARATOR      _T('\\')
#define PATH_SEP_STRING     _T("\\")
#else
#define PATH_SEPARATOR      _T('/')
#define PATH_SEP_STRING     _T("/")
#endif

#define DBGPRINT(LEVEL,INFO) \
    {\
        std::ostringstream StrInfo;\
        StrInfo<<INFO;\
        Common::TSingleton<Common::CDbgPrint>::Instance()->DbgPrint(LEVEL, __FILE__, __LINE__, __FUNCTION__, StrInfo.str().c_str());\
    }

const unsigned long int DBG_ERROR = 0xF0000000;

#define DBGFILE             "/tmp/hrRobot.txt"
#define DBFCONFIG           "dbgConfig"

//namespace Common {

template<typename TYPE>
bool ReadLineToArray (const char* lpFileName, TYPE &t);

bool ReadFileToList (const char* lpFileName, list<string> &List);

bool ReadFileToVector (const char* lpFileName,  vector<string> &Vec);

string ReadFileToString(const char* lpFileName);
std::string GetModuleFullPath(bool bLastPath=false);
template<class T>
struct _tag_split_string
{
    void operator()(T &t, std::string &szItem)
    {
        t.push_back (szItem);
    }
};
//  把字符串按转换为DWIRD list (16进制)
struct _tag_split_string_to_hexnumber
{
    void operator()(LIST_DWORD &t, std::string &szItem)
    {
        DWORD dwNum = 0;
        sscanf (szItem.c_str(), "%x", &dwNum);
        t.push_back (dwNum);
    }
};
template<class T, class S>
T SplitString (const std::string &szString, const char * lpChar, S s);

//  把字符串按分割符转换为vector
vector<string> ToVectors (const std::string szString, const char * lpChar);

//  把字符串按分割符转换为list
LIST_STRING ToLists(const std::string &szString,const char *  lpChar);

//判断字符串是否是数字
bool isnum(string s);

void StringReplaceAll(std::string &strIn, const std::string &strSrc, const std::string &strDest);

class CExpresion {
public:
	CExpresion();
	~CExpresion();

	double getValue(string strExp);
	static bool isOperator(char op);	//是否运算符
private:
	//中缀转前缀
	void  infix2Prefix(const string& strInfix,string& strPre);
	//中缀转后缀
	void infix2Postfix(const string& strInfix, string& strPost);
	//前缀表达式求值
	double  prefixEvaluation(const string& strPrefix);
	//后缀表达式求值
	double  postfixEvaluation(const string& strPostfix);

	int priority(char op);		//符号优先级
	
	double OP(double op1, double op2, char op);
};

///判断是否授权
bool bAuthorise();
}

#endif
