#include "json.h"
#include "json-forwards.h"
#include <iostream>
#include "RobSoft/CDataStructure.hpp"

using namespace std;
namespace rclib {

MY_CLASS jsonWrapper
{
public:
    jsonWrapper();
    ~jsonWrapper();

    //获取abb的预定义变量
    bool getJsonPredefineValue(const char *path);
    /*******************************************
     * strType:变量类型
     * strName:变量名
     * value: 返回值
     ******************************************/
    bool getJsonValue(const string& strType, const string& strName, string &strValue);
    bool getJsonValue(const string &strType, const string &strName, int& nValue);
    bool getJsonValue(const string &strType, const string &strName, bool& bValue);
    bool getJsonValue(const string &strType, const string &strName, double& fValue);
    //将string类型转换成json::value
    static bool stringToJsValue(string strValue, Json::Value& jsValue);
    //将robtargat转换成Terminal
    static bool  robtargatToTerminal(string& strRobTar, robsoft::Terminal&);
    //将jointtarget转换成joints
    static bool jointtargetToJoints(string& strJoint ,robsoft::Joints& joint);
    //将speeddata转换成Double
    static bool  speedToDouble(string& strSpeedData, double& fValue);


private:

private:
    Json::Value m_jsPredefineValue;              //预定义的变量
};

}
