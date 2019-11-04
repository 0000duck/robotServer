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
        str = g_intLanguageType? "Out of position":"��������λ��";
        break;
    case OVERVELOCITY:
        str = g_intLanguageType? "Excess speed":"��������ٶ�";
        break;
    case OVERACCELERATION:
        str = g_intLanguageType? "":"���������ٶ�";
        break;
    default:
        cout << "ROBOTICS ERROR: " << robErrno << endl;
        break;
    }

    return str;
}

#endif
