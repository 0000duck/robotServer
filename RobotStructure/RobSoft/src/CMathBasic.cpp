#ifndef CMATHBASIC_CPP
#define CMATHBASIC_CPP

#include <iostream>
#include "CMathBasic.hpp"

using namespace std;

namespace robsoft{

bool num_is_zero(double num){
    if(num < EPSLON && num > -EPSLON){
        return true;
    }
    else{
        return false;
    }
}

double sin_deg(double angle){
    return sin(angle * PI / 180);
}

double cos_deg(double angle){
    return cos(angle * PI / 180);
}

double tan_deg(double angle){
    return tan(angle * PI / 180);
}

double acos_deg(double value){
    return acos(value) * 180 / PI;
}

double atan2_deg(double y, double x){
    return atan2(y, x) * 180 / PI;
}

double cube(double value){
    if(value >= 0){
        return pow(value, 1.0/3);
    }
    else{
        return -pow(-value, 1.0/3);
    }
}

vector<double> solveQuadraticEquation(double a, double b, double c){
    if(num_is_zero(a)){
        cout << "solveQuadraticEquation: a == 0" << endl;
        exit(1);
    }

    vector<double> res;
    double delta = pow(b, 2) - 4*a*c;
    if(delta < 0){
        cout << "solveQuadraticEquation: no root" << endl;
        return res;
    }
    res.push_back((-b-sqrt(delta))/(2*a));
    res.push_back((-b+sqrt(delta))/(2*a));
    return res;
}

vector<double> solveCubicEquation(double a, double b, double c, double d){
    if(num_is_zero(a)){
        cout << "solveCubicEquation: a == 0" << endl;
        exit(1);
    }

    vector<double> res;
    double A = pow(b, 2) - 3*a*c;
    double B = b*c - 9*a*d;
    double C = pow(c, 2) - 3*b*d;
    double delta = pow(B, 2) - 4*A*C;
    if(num_is_zero(A) && num_is_zero(B)){
        res.push_back(-b / (3*a));
        return res;
    }

    if(num_is_zero(delta)){
        if(num_is_zero(A)){
            cout << "solveCubicEquation: no root" << endl;
            return res;
        }
        double K = B/A;
        res.push_back(-b/a+K);
        res.push_back(-K/2);
        return res;
    }

    if(delta > 0){
        double Y1 = A*b+3*a*((-B+sqrt(delta))/2);
        double Y2 = A*b+3*a*((-B-sqrt(delta))/2);
        res.push_back((-b-(cube(Y1)+cube(Y2)))/(3*a));
        return res;
    }

    if(delta < 0){
        if(num_is_zero(A) || A < 0){
            cout << "solveCubicEquation: no root" << endl;
            return res;
        }

        double T = (2*A*b-3*a*B)/(2*sqrt(pow(A, 3)));
        if(T<-1 || T>1){
            cout << "solveCubicEquation: no root" << endl;
            return res;
        }

        double theta = acos(T);
        res.push_back((-b-2*sqrt(A)*cos(theta/3))/(3*a));
        res.push_back((-b+sqrt(A)*(cos(theta/3)+sqrt(3)*sin(theta/3)))/(3*a));
        res.push_back((-b+sqrt(A)*(cos(theta/3)-sqrt(3)*sin(theta/3)))/(3*a));
        return res;
    }
}

}

#endif
