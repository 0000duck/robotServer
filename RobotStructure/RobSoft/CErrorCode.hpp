#ifndef CERRORCODE_HPP
#define CERRORCODE_HPP

#include <string>

#define ERROR_TRUE  0
#define ERROR_OVERRANGE 1                       //超出限制位置
#define ERROR_OVERVELOCITY 2                    //超出最大速度
#define ERROR_OVERACCELERATION 3                //超出最大加速度
#define ERROR_UNDEFINE_VARIABLE         15       //未定义变量
#define ERROR_VARIABLE_ASSIGNMENT       16       //变量赋值
#define ERROR_POINTER_ACCESS_VIOLATION  17       //函数指针访问越界
#define ERROR_SYNTAX_ERROR              4       //语法错误
#define ERROR_OPERATION_ASSIGNMENT      5       //不同变量
#define ERROR_NULL_SENTENCE             6       //空语句
#define ERROR_VARIABLE_DEFINITION       7		//变量定义错误
#define ERROR_VARIABLE_REDEFINITION     8		//变量重复定义
#define ERROR_FUNCTION_REDEFINITION     9		//函数重复定义
#define ERROR_INVALID_KEYWORD           10		//无效的关键字
#define ERROR_CAMEL_CONNECT             11		//相机未连接或不存在
#define ERROR_IO_CONNECT                12		//IO未连接
#define ERROR_NO_FUNCTION               13		//The function doesn't exist
#define ERROR_NO_MATCH_FUNCTION			14		//No matching function was found

namespace robsoft{
enum ROBERRNO{NOERROR_ROB, OVERRANGE, OVERVELOCITY, OVERACCELERATION};
}

std::string get_error_string(int robErrno,int g_intLanguageType=0);

#endif
