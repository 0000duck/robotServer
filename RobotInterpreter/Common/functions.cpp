#include <stdio.h>
#include "functions.h"
#include <string.h>
#include <time.h>
#include <algorithm>
#ifdef __linux__
#include <sys/time.h>
#include <sys/times.h>
#include <libgen.h>
#else
#include <windows.h>
#endif

namespace Common {

template<typename TYPE>
bool
ReadLineToArray (const char* lpFileName,
                 TYPE &t)
{
    if (!lpFileName || '\0' == *lpFileName)
    {
        return false;
    }

    FILE *lpFile = fopen (lpFileName, "r");
    if (NULL == lpFile)
    {
        return false;
    }

    char szBuffer[4096];
    memset (szBuffer, 0, sizeof (szBuffer));
    while (!feof (lpFile))
    {
        szBuffer[0] = 0;
        fgets (szBuffer, sizeof (szBuffer) - 1, lpFile);

        std::string szLine = szBuffer;
        szLine.erase(0, szLine.find_first_not_of(" \t\r\n"));
        szLine.erase(szLine.find_last_not_of(" \t\r\n") + 1);
        if (szLine.empty())
        {
            continue;
        }
        t.push_back (szLine);
    }

    fclose (lpFile);
    return true;
}

bool
ReadFileToList (const char* lpFileName,
                 list<string> &List)
{
    return ReadLineToArray (lpFileName, List);
}

bool
ReadFileToVector (const char* lpFileName,
                   vector<string> &Vec)
{
    return ReadLineToArray (lpFileName, Vec);
}



template<class T, class S>
T SplitString (const std::string &szString,
             const char* lpChar,
             S s)
{
    T t;
    std::string::size_type start = 0;
    std::string::size_type end = 0;
    while ((end = szString.find(lpChar, start)) != std::string::npos)
    {
        if (end != start)
        {
            std::string szItem = szString.substr(start, end - start);
            s (t, szItem);
        }
        start = end + strlen(lpChar);
    }
    //最后一项可能没有分隔符
    if (start < szString.length())
    {
        std::string szItem = szString.substr(start);
        s (t, szItem);
    }
    return t;
}

//  把字符串按分割符转换为vector
vector<string> ToVectors (const std::string szString, const char * lpChar)
{
    return SplitString <vector<string>>(szString, lpChar, _tag_split_string<VECTOR_STRING>());
}

//  把字符串按分割符转换为list
LIST_STRING
ToLists(const std::string &szString,
        const char *  lpChar)
{
    return SplitString <LIST_STRING>(szString, lpChar, _tag_split_string<LIST_STRING>());
}

LIST_DWORD
ToHexNumLists(const std::string &szString,
             const char* lpChar)
{
    return SplitString <LIST_DWORD>(szString, lpChar, _tag_split_string_to_hexnumber());
}

bool isnum(string s)
{
        stringstream sin(s);
        double t;
        char p;
        if(!(sin >> t))
        /*解释：
            sin>>t表示把sin转换成double的变量（其实对于int和float型的都会接收），如果转换成功，则值为非0，如果转换不成功就返回为0
        */
               return false;
        if(sin >> p)
        /*解释：此部分用于检测错误输入中，数字加字符串的输入形式（例如：34.f），在上面的的部分（sin>>t）已经接收并转换了输入的数字部分，
		在stringstream中相应也会把那一部分给清除，如果此时传入字符串是数字加字符串的输入形式，则此部分可以识别并接收字符部分，
		例如上面所说的，接收的是.f这部分，所以条件成立，返回false;如果剩下的部分不是字符，那么则sin>>p就为0,则进行到下一步else里面
          */
                return false;
        else
            return true;
}

void StringReplaceAll(std::string &strIn, const std::string &strSrc, const std::string &strDest)
{
    std::string::size_type pos = 0;
    std::string::size_type srcLen = strSrc.size();
    std::string::size_type destLen = strDest.size();
    DWORD t = 0;
    while ((pos = strIn.find(strSrc, pos)) != std::string::npos)
    {
        strIn.replace(pos, srcLen, strDest);
        pos += destLen;
        if (++t % 8 == 0)
        {
#ifdef __linux__
            usleep(1000);
#else
            Sleep(1);
#endif
        }

    }
}

string ReadFileToString(const char *lpFileName)
{
    if(!lpFileName)
    {
        return  "";
    }

    FILE *lpFile = fopen (lpFileName, "r");
    if (NULL != lpFile)
    {
        fseek (lpFile, 0, SEEK_END);
        DWORD dwSize = ftell (lpFile);
        fseek (lpFile, 0, SEEK_SET);

        char* lpChar = new char [dwSize];
        if (lpChar)
        {
            fread (lpChar, dwSize, 1, lpFile);
            std::string strValue;
            strValue.append (lpChar, dwSize);

            delete []lpChar;
            return strValue;
        }
    }
    return "";
}


std::string GetModuleFullPath ( bool bLastPath )
{
    char chPath [256] = {0};

#ifdef _WIN32
    ::GetModuleFileName ( NULL, chPath, sizeof (chPath) );
#else
    memset(chPath, 0, sizeof(chPath));
    ::readlink("/proc/self/exe", chPath, sizeof (chPath));
#endif
    std::string strModuleFileName = chPath;

    if ( bLastPath == false )
    {
        return strModuleFileName.c_str();
    }

    //如果文件名称有生僻汉字并且第二字节是'\\'时，用find_last_of的方法就会有问题
    //比如："c:\\ABC癨.txt"就会返回"c:\\ABC\\"，而用_splitpath和_makepath正确返回"c:\\"
//	size_t nPos = strModuleFileName.find_last_of(PATH_SEP_STRING);
//	std::string strLastPath = strModuleFileName.substr(0, nPos);

#ifdef _WIN32
    char szDrive[_MAX_DRIVE];
    char szDir[_MAX_DIR];
    ::_splitpath (chPath, szDrive, szDir, NULL, NULL);
    ::_makepath (chPath, szDrive, szDir, NULL, NULL);
    //去除最后的'\\'
    if (chPath[strlen(chPath) - 1] == PATH_SEPARATOR)
    {
        chPath[strlen(chPath) - 1] = '\0';
    }

#else
    ::dirname(chPath);
#endif

    std::string strLastPath = chPath;
    return strLastPath.c_str();
}
#if 0
//  调试输出
    void hrDbgPrint( unsigned long nLevel, const char* lpFile, unsigned int nLine, const char* lpFunc, const char* lpInfo)
{
    char __szBuf__[128];
    struct tm __tm__;
    time_t __t__ = time (NULL);

#if (defined _WIN32) && (_MSC_VER < 1400)
    struct tm *ptm = localtime(&__t__);
    __tm__ = *ptm;
#else
    //time_t t = time(nullptr);
    //struct tm* nowTime;
    localtime_r( &__t__,&__tm__);
    //localtime_r(&__tm__, &__tm__);
#endif

#ifdef _WIN32
    sprintf (__szBuf__,
        LEFT_MIDDLE_BRACKET_STRING _T("Level=") "0x%X" RIGHT_MIDDLE_BRACKET_STRING TAB_STRING
        LEFT_MIDDLE_BRACKET_STRING _T("ThreadId=") "%lu" RIGHT_MIDDLE_BRACKET_STRING TAB_STRING
        LEFT_MIDDLE_BRACKET_STRING _T("Tickcout=") "%lu" RIGHT_MIDDLE_BRACKET_STRING TAB_STRING
        LEFT_MIDDLE_BRACKET_STRING "%.4d-%.2d-%.2d %.2d:%.2d:%.2d" RIGHT_MIDDLE_BRACKET_STRING,
        nLevel, BaseLib::GetCurrentThreadID(), GetTickCount() / 1000,
        __tm__.tm_year + 1900, __tm__.tm_mon + 1, __tm__.tm_mday,
        __tm__.tm_hour, __tm__.tm_min, __tm__.tm_sec);
#else
    snprintf (__szBuf__, sizeof (__szBuf__),
        LEFT_MIDDLE_BRACKET_STRING "Level=" "0x%X" RIGHT_MIDDLE_BRACKET_STRING TAB_STRING
        LEFT_MIDDLE_BRACKET_STRING "ThreadId=" "%lu" RIGHT_MIDDLE_BRACKET_STRING TAB_STRING
        LEFT_MIDDLE_BRACKET_STRING "Tickcout=" "%lu" RIGHT_MIDDLE_BRACKET_STRING TAB_STRING
        LEFT_MIDDLE_BRACKET_STRING "%.4d-%.2d-%.2d %.2d:%.2d:%.2d" RIGHT_MIDDLE_BRACKET_STRING,
        nLevel, pthread_self(), times(NULL) / sysconf(_SC_CLK_TCK) / 1000,
        __tm__.tm_year + 1900, __tm__.tm_mon + 1, __tm__.tm_mday,
        __tm__.tm_hour, __tm__.tm_min, __tm__.tm_sec);
#endif

    const char* __lpFile__   = strrchr(lpFile, PATH_SEPARATOR);
    __lpFile__ = (__lpFile__ == NULL ? lpFile : __lpFile__ + 1);
    std::ostringstream __oDbgString__;
    __oDbgString__ << __szBuf__ << TAB_STRING <<
        LEFT_MIDDLE_BRACKET_STRING << __lpFile__ << COLON_STRING << nLine << RIGHT_MIDDLE_BRACKET_STRING << TAB_STRING <<
        LEFT_MIDDLE_BRACKET_STRING << lpFunc << RIGHT_MIDDLE_BRACKET_STRING << NEW_LINE << TAB_STRING << lpInfo << NEW_LINE;
    OUTPUT_DEBUG (__oDbgString__.str().c_str());
}


    int CDbgPrint::GetDebugLevel (unsigned long int id)
    {
        if (id & DBG_ERROR)
        {
            return 1;
        }
        else
        {
            m_lock.lockMutex();
            for (std::list<unsigned long int >::iterator iter = m_list.begin();
                iter != m_list.end();
                iter++)
            {
                m_lock.unlockMutex();
                if (id == *iter) return 0;
            }
            m_lock.unlockMutex();
        }

        return -1;
    }

    void
    CDbgPrint::SetDebug (const char* lpString)
    {
        if (!lpString) return ;

        m_lock.lockMutex();
        m_list.clear();
        m_textlist.clear();

        std::string szString = lpString;
        std::replace (szString.begin(),szString.end(), ',', ';');

        LIST_STRING lists = ToLists (szString, "|");

        int iCount = 0;
        for (LIST_STRING::iterator iter = lists.begin();
            iter != lists.end();
            iter++, iCount++)
        {
            if (0 == iCount)
            {
                m_list = ToHexNumLists (*iter, ";");
            }
            else if (1 == iCount)
            {
                m_textlist = ToLists (*iter, ";");
                break;
            }
        }

        m_lock.unlockMutex();
    }

    void CDbgPrint::ReadLevelFromFile()
    {
        string strFilePath = GetModuleFullPath(true) +"/" +DBFCONFIG;
        VECTOR_STRING vecLevel ;
        ReadFileToVector(strFilePath.c_str(), vecLevel);

        string strLev;
        for(int i =0 ; i<vecLevel.size();i++)
        {
            char buf[256] ;
            sprintf(buf,"0x%d,",atoi(vecLevel[i].c_str()));
            strLev += buf;
        }
        SetDebug(strLev.c_str());
    }

    void
    CDbgPrint::DbgPrint(DWORD nLevel, const char* lpFile, int nLine, const char* lpFunc, const char* lpInfo)
    {
        int iLevel = GetDebugLevel(nLevel);
        if (this->m_DbgFunction)
        {
            if (0 == iLevel)
            {
                if (m_textlist.size())
                {
                    bool bFind = false;
                    for (std::list<std::string>::iterator iter = m_textlist.begin();
                        iter != m_textlist.end();
                        iter++)
                    {
                        if (strstr (lpInfo, (*iter).c_str()))
                        {
                            bFind = true;
                            break;
                        }
                    }

                    if (false == bFind)
                    {
                        return ;
                    }
                }

                this->m_DbgFunction(nLevel, lpFile, nLine, lpFunc, lpInfo);
            }
            else if (1 == iLevel)
            {
                this->m_DbgFunction(nLevel, lpFile, nLine, lpFunc, lpInfo);
            }
        }
    }

    void
    CDbgPrint::DefaultDbgPrint(DWORD nLevel, const char* lpFile, int nLine, const char* lpFunc, const char* lpInfo)
    {
        printf ("[%d|%s|%d|%s]%s\r\n", nLevel, lpFile, nLine, lpFunc, lpInfo);
    }
#endif

	CExpresion::CExpresion() {}
	CExpresion::~CExpresion()
	{

	}

	double CExpresion::getValue(string strExp)
	{
		string strPost;
		infix2Postfix(strExp, strPost);
		return postfixEvaluation(strPost);
	}

	//中缀转前缀
	void  CExpresion::infix2Prefix(const string& strInfix, string& strPre)
	{

	}
	//中缀转后缀
	void CExpresion::infix2Postfix(const string& strInfix, string& strPost)
	{
		VECTOR_STRING vecIterm = ToVectors( strInfix, " ");
		stack<string > tmpStack;

		for (int i = 0; i < vecIterm.size(); i++)
		{
			if (isnum(vecIterm[i]))
			{
				strPost += vecIterm[i] + " ";
			}
			if (vecIterm[i] == "(")
			{
				tmpStack.push(vecIterm[i]);
			}
			while (isOperator(vecIterm[i][0]))
			{
				
				//string top = tmpStack.top();
				if (tmpStack.empty() || tmpStack.top() == "(" 
					|| priority(vecIterm[i][0]) > priority(tmpStack.top()[0]))
				{
					tmpStack.push(vecIterm[i]);
					break;
				}
				else
				{
					strPost += tmpStack.top()+" ";
					tmpStack.pop();
				}
			}
			if (vecIterm[i] == ")")
			{
				while (tmpStack.top() != "(")
				{
					strPost += tmpStack.top() + " ";
					tmpStack.pop();
				}
				tmpStack.pop();
			}
		}
		while (!tmpStack.empty())
		{
			strPost += tmpStack.top() + " ";
			tmpStack.pop();
		}
	}
	//前缀表达式求值
	double  CExpresion::prefixEvaluation(const string& strPrefix)
	{
		return 0;
	}
	//后缀表达式求值
	double  CExpresion::postfixEvaluation(const string& strPostfix)
	{
		VECTOR_STRING vecField = Common::ToVectors(strPostfix, " ");
		stack<double> cStack;
		for (int i = 0; i < vecField.size(); i++)
		{
			if (isnum(vecField[i]))
			{
				cStack.push(atof(vecField[i].c_str()));
			}
			else {
				double op1,op2,rt;
				op2 = cStack.top();
				cStack.pop();
				op1 = cStack.top();
				cStack.pop();
				rt = OP(op1,op2,vecField[i][0]);
				cStack.push(rt);
			}
		}
		if (cStack.empty())
		{
			return 0;
		}
		else
		{
			return cStack.top();
		}
	}

	int CExpresion::priority(char op)				//判断运算符级别函数；其中* /的级别为2，+ -的级别为1；
	{
		if (op == '+' || op == '-')
			return 1;
		if (op == '*' || op == '/')
			return 2;
		else
			return 0;
	}
	bool CExpresion::isOperator(char op)				//判断输入串中的字符是不是操作符，如果是返回true
	{
		return (op == '+' || op == '-' || op == '*' || op == '/');
	}
	double CExpresion::OP(double op1, double op2, char op)
	{
		double res = 0;
		if (op == '+')
			res = op1 + op2;
		else if (op == '-')
			res = op1 - op2;
		else if (op == '*')
			res = op1 * op2;
		else if (op == '/')
			res = op1 / op2;
		return res;
	}
#define  CONFIG_INI  "config\\server.ini"
#define  DLL_FILE_NAME "HRlicence.dll"
	bool bAuthorise()
	{
	#ifdef WIN32
		HMODULE handle = NULL;
		string strPath = GetModuleFullPath(true)+"\\";
		handle = ::LoadLibrary((strPath+DLL_FILE_NAME).c_str());	//加载库
		if (handle == NULL)
		{
			printf("err=%s\r\n");
			return false;
		}

		//获取接口
		int (WINAPI* getUniqueId)();
		getUniqueId = (int (WINAPI*)())GetProcAddress(handle, "getUniqueId");

		int iUniqueId = getUniqueId();
		FreeLibrary(handle);	//释放库

		string strId = ReadFileToString((strPath + CONFIG_INI).c_str());
		//printf("iUniqueId=%d\tstrId=%s\n", iUniqueId, strId.c_str());
		if (!strId.empty() && iUniqueId == atoi(strId.c_str()))
		{
			return true;
		}
	#endif
		return false;
	}
}
