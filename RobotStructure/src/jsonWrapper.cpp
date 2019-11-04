#include "jsonWrapper.h"

using namespace robsoft;
namespace rclib {
using namespace std;
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
        unsigned long dwSize = ftell (lpFile);
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

jsonWrapper::jsonWrapper()
{

}
jsonWrapper::~jsonWrapper()
{

}

bool jsonWrapper::getJsonPredefineValue(const char *path)
{
    string strInfo = ReadFileToString(path);
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    string err;
    m_jsPredefineValue.clear();
    if(reader->parse(strInfo.c_str(),strInfo.c_str()+strInfo.length(), &m_jsPredefineValue, &err))
    {
        return true;
    }else {
        cout << "Predefined variable resolution failed, error:"<<err<<endl;
    }
    return  false;
}

bool jsonWrapper::getJsonValue(const string &strType, const string &strName, string &strValue)
{
    if(!m_jsPredefineValue.empty()&&!m_jsPredefineValue[strType].empty())
    {
        if(m_jsPredefineValue[strType].isArray())
        {
            auto nSize = m_jsPredefineValue[strType].size();

            for (int i=0; i< nSize; i++) {
                if(m_jsPredefineValue[strType][i].isObject())
                {
                    if(m_jsPredefineValue[strType][i]["name"].asString()==strName)
                    {
                        strValue = m_jsPredefineValue[strType][i]["val"].asString();
                        return true;
                    }
                }
            }
        }
    }
    return  false;
}
bool jsonWrapper::getJsonValue(const string &strType, const string &strName, int& nValue)
{
    if(!m_jsPredefineValue.empty()&&!m_jsPredefineValue[strType].empty())
    {
        if(m_jsPredefineValue[strType].isArray())
        {
            auto nSize = m_jsPredefineValue[strType].size();
            for (int i=0; i<nSize; i++) {
                if(m_jsPredefineValue[strType][i]["name"].asString()==strName)
                {
                    nValue = m_jsPredefineValue[strType][i]["val"].asInt();
                    return  true;
                }
            }
        }
    }
    return  false;
}

bool jsonWrapper::getJsonValue(const string &strType, const string &strName, bool& bValue)
{
    if(!m_jsPredefineValue.empty()&&!m_jsPredefineValue[strType].empty())
    {
        if(m_jsPredefineValue[strType].isArray())
        {
            auto nSize = m_jsPredefineValue[strType].size();
            for (int i=0; i<nSize; i++) {
                if(m_jsPredefineValue[strType][i]["name"].asString()==strName)
                {
                    bValue = m_jsPredefineValue[strType][i]["val"].asBool();
                    return  true;
                }
            }
        }
    }
    return  false;
}

bool jsonWrapper::getJsonValue(const string &strType, const string &strName, double& fValue)
{
    if(!m_jsPredefineValue.empty()&&!m_jsPredefineValue[strType].empty())
    {
        if(m_jsPredefineValue[strType].isArray())
        {
            auto nSize = m_jsPredefineValue[strType].size();
            for (int i=0; i<nSize; i++) {
                if(m_jsPredefineValue[strType][i]["name"].asString()==strName)
                {
                    fValue = m_jsPredefineValue[strType][i]["val"].asDouble();
                    return  true;
                }
            }

        }
    }
    return  false;
}

bool jsonWrapper::stringToJsValue(string strValue, Json::Value &jsValue)
{
    if(!strValue.empty())
    {
        Json::CharReaderBuilder builder;
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        string err;
        if(reader->parse(strValue.c_str(),strValue.c_str()+strValue.length(), &jsValue, &err))
        {
            return true;
        }
    }

    return false;
}

bool jsonWrapper::robtargatToTerminal(string& strRobTar, robsoft::Terminal& ter)
{
    //转换路点
    //Quaternion

    Json::Value jsPoint;
    string strPoint = strRobTar;
    double x,y,z;
    double q1,q2,q3,q4;
    double A, B, C;
    if(stringToJsValue(strPoint,jsPoint))
    {
        if(jsPoint.isArray()&&jsPoint.size()==4)
        {
            if(jsPoint[0].isArray()&&jsPoint[0].size()==3)
            {
                x = jsPoint[0][0].asDouble();
                y = jsPoint[0][1].asDouble();
                z = jsPoint[0][2].asDouble();

                if(jsPoint[1].isArray()&&jsPoint[1].size()==4)
                {
                    q1 = jsPoint[1][0].asDouble();
                    q2 = jsPoint[1][1].asDouble();
                    q3 = jsPoint[1][2].asDouble();
                    q4 = jsPoint[1][3].asDouble();
                    Quaternion quater(q1,q2,q3,q4);
                    AttitudeAngle Angle = quater.getAttitudeAngle();
                    A = Angle[ATTITUDE_A];
                    B = Angle[ATTITUDE_B];
                    C = Angle[ATTITUDE_C];
                    ter.setValue(x,y,z,A,B,C);
                    return  true;
                }
            }
        }
    }
    return false;
}

bool jsonWrapper::jointtargetToJoints(string &strJoint, Joints &joint)
{
    Json::Value jsJoint;
    if(stringToJsValue(strJoint,jsJoint))
    {
        if(jsJoint.isArray()&&jsJoint.size()==2)
        {
            if(jsJoint[0].isArray()||jsJoint.size()==6)
            {
                std::vector<double> vecVal;
                for(int i=0; i<6; i++)
                {
                    //if(jsJoint[0][i].isInt())
                    vecVal.push_back(jsJoint[0][i].asDouble());
                }
                joint.setValue(6,vecVal);
                return  true;
            }
        }
    }
    return false;
}

bool jsonWrapper::speedToDouble(string& strSpeedData, double& fValue)
{
    Json::Value jsSpeed;
    if(stringToJsValue(strSpeedData,jsSpeed))
    {
        if(jsSpeed.isArray()&&jsSpeed.size()==4)
        {
            fValue = jsSpeed[0].asDouble();
            return true;
        }
    }
    return false;
}

}
