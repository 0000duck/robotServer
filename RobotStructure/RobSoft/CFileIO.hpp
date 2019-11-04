#ifndef CFILEIO_HPP
#define CFILEIO_HPP

#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xmlreader.h>
#include <string>
#include <map>

#include "CDataStructure.hpp"

std::string xmlchar_to_string(const xmlChar* str);

double xml_read_num(const xmlNodePtr& node);    // 读取数值节点
void xml_judge_read_num(const char* name, const xmlNodePtr& node, double& num);    // 判断节点名称读取数值节点
void xml_judge_read_num(const char* name, const xmlNodePtr& node, int& num);   // 判断节点名称读取数值节点
xmlNodePtr xml_write_num(const char* name, double num); // 写入数值节点
void xml_addchild_num(const char* name, double num, const xmlNodePtr& node);    // 添加数值类型的子节点

std::string xml_read_string(const xmlNodePtr& node);    // 读取字符串节点
void xml_judge_read_string(const char* name, const xmlNodePtr& node, std::string& str);    // 判断节点名称读取字符串节点
xmlNodePtr xml_write_string(const char* name, std::string str); // 写入字符串节点
void xml_addchild_string(const char* name, std::string str, const xmlNodePtr& node);    // 添加字符串类型的子节点

robsoft::Joints xml_read_joints(const xmlNodePtr& node);    // 读取Joints节点
void xml_judge_read_joints(const char* name, const xmlNodePtr& node, robsoft::Joints& joint);  // 判断节点名称读取Joints节点
xmlNodePtr xml_write_joints(const char* name, const robsoft::Joints& joint);    // 写入Joints节点
void xml_addchild_joints(const char* name, const robsoft::Joints& joint, const xmlNodePtr& node);   // 添加Joints类型的子节点

robsoft::Terminal xml_read_terminal(const xmlNodePtr& node);    // 读取Terminal节点
void xml_judge_read_terminal(const char* name, const xmlNodePtr& node, robsoft::Terminal& terminal);   // 判断节点名称读取Terminal节点
xmlNodePtr xml_write_terminal(const char* name, const robsoft::Terminal& terminal);     // 写入Terminal节点
void xml_addchild_terminal(const char* name, const robsoft::Terminal& terminal, const xmlNodePtr& node);    // 添加Terminal类型的子节点

std::map<std::string, robsoft::Terminal> xml_read_terminal_map(const xmlNodePtr& node);    // 读取Terminal Map节点
void xml_judge_read_terminal_map(const char* name, const xmlNodePtr& node, std::map<std::string, robsoft::Terminal>& terminalMap); // 判断节点名称读取Terminal Map节点
xmlNodePtr xml_write_terminal_map(const char* name, std::map<std::string, robsoft::Terminal>& terminalMap);   // 写入Terminal Map节点
void xml_addchild_terminal_map(const char* name, std::map<std::string, robsoft::Terminal>& terminalMap, const xmlNodePtr& node);  // 添加Terminal Map类型的子节点

bool xml_judge_name(const char* name, const xmlNodePtr& node);  // 判断节点名称
int xml_get_child_num(const xmlNodePtr& node);  // 获取子节点数量

xmlNodePtr xml_find_child_node_content(const char* content, const xmlNodePtr& node);   // 寻找特定content的子节点

#endif
