#ifndef CMATHBASIC_HPP
#define CMATHBASIC_HPP

#include <vector>
#include <cmath>
#ifdef __linux__
#define MY_CLASS  class
#else
#define MY_CLASS	class  _declspec(dllexport)
#endif
namespace robsoft{

#define EPSLON 1e-8 // 零元
#define PI 3.14159265359
#define GOLDRATIO (sqrt(5)-1)/2

bool num_is_zero(double num);   // 判断数值是否小于零元

double sin_deg(double angle);
double cos_deg(double angle);
double tan_deg(double angle);
double acos_deg(double value);
double atan2_deg(double y, double x);

double cube(double value);  // 求解立方根

std::vector<double> solveQuadraticEquation(double a, double b, double c);   // 求解一元二次方程
std::vector<double> solveCubicEquation(double a, double b, double c, double d); // 盛金法求一元三次方程

}

#endif
