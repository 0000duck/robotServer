#ifndef CERRORCODE_CPP
#define CERRORCODE_CPP

#include "CErrorCode.hpp"

#include <iostream>

using namespace std;
using namespace robsoft;

string get_error_string(int robErrno,int g_intLanguageType){
    string str;
    switch(ROBERRNO(robErrno)){
    case NOERROR_ROB:
        break;
    case OVERRANGE:
        str = g_intLanguageType? "Out of position":"超出限制位置";
        break;
    case OVERVELOCITY:
        str = g_intLanguageType? "Excess speed":"超出最大速度";
        break;
    case OVERACCELERATION:
        str = g_intLanguageType? "":"超出最大加速度";
        break;
    default:
        cout << "ROBOTICS ERROR: " << robErrno << endl;
        break;
    }

    return str;
}

#endif
