#ifndef ROBOTFILE_H
#define ROBOTFILE_H

#include <fstream>
#include <string>
#include <vector>

#define ROBOT_PARAMETER_PATH "RobotParameter.xml"
#define ROBOT_PROGRAM_PATH "RobotProgram.xml"
#define ROBOT_FRAME_PATH "RobotFrame.xml"
#define ROBOT_PREFERENCE_PATH "RobotPreference.xml"
#define ROBOT_JOINTS_PATH "RobotJoints.xml"

int copy_file(const char* src, const char* dst);    // 复制文件
bool get_line_string(std::ifstream& p_fin, std::vector<std::string>& p_s);  // 获取一行文本以空格为分割，存成字符串向量

#endif
