#ifndef CPOLYNOMIAL_CPP
#define CPOLYNOMIAL_CPP

#include "CPolynomial.hpp"
#include "CMathBasic.hpp"

#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace robsoft;

NewtonPolynomial::NewtonPolynomial(){

}

NewtonPolynomial::NewtonPolynomial(const CMatrix<double> &x, const CMatrix<double> &y){
    this->setPoints(x, y);
}

NewtonPolynomial::~NewtonPolynomial(){

}

void NewtonPolynomial::setPoints(const CMatrix<double> &x, const CMatrix<double> &y){
    m_x = x;

    m_dividedDifference.setValue(y.rows(), y.rows());
    m_dividedDifference.zeros();
    for(int i=0; i<y.rows(); i++){
        m_dividedDifference.at(i, 0) = y.at(i, 0);
    }

    for(int j=1; j<m_dividedDifference.cols(); j++){
        for(int i=j; i<m_dividedDifference.rows(); i++){
            if(num_is_zero(m_x.at(i, 0) - m_x.at(i-j, 0))){
                throw string("Error: NewtonPolynomial::same point");
            }
            double value = (m_dividedDifference.at(i, j-1) - m_dividedDifference.at(i-1, j-1));
            value = value / (m_x.at(i, 0) - m_x.at(i-j, 0));
            m_dividedDifference.at(i, j) = value;
        }
    }
}

double NewtonPolynomial::getValue(double x) const{
    double res = 0;
    for(int i=0; i<m_dividedDifference.rows(); i++){
        double value = m_dividedDifference.at(i, i);
        for(int j=0; j<i; j++){
            value = value*(x-m_x.at(j, 0));
        }
        res += value;
    }

    return res;
}

double NewtonPolynomial::getDerivative(double x, int n) const{
    double res = 0;
    switch(n){
    case 0:{
        res = getValue(x);
    }
        break;
    case 1:{
        for(int m=1; m<m_dividedDifference.rows(); m++){
            double sum = 0;
            for(int i=0; i<m; i++){
                double prod = 1;
                for(int j=0; j<m; j++){
                    if(i == j){
                        continue;
                    }
                    prod = prod*(x-m_x.at(j, 0));
                }
                sum += prod;
            }
            res += m_dividedDifference.at(m, m) * sum;
        }
    }
        break;
    case 2:{
        for(int m=2; m<m_dividedDifference.rows(); m++){
            double sum1 = 0;
            for(int i=0; i<m; i++){
                double sum2 = 0;
                for(int j=0; j<m; j++){
                    if(i == j){
                        continue;
                    }
                    double prod = 1;
                    for(int k=0; k<m; k++){
                        if(k==i || k==j){
                            continue;
                        }
                        prod = prod*(x-m_x.at(k, 0));
                    }
                    sum2 += prod;
                }
                sum1 += sum2;
            }
            res += m_dividedDifference.at(m, m) * sum1;
        }
    }
        break;
    default:
        break;
    }
    return res;
}

void NewtonPolynomial::print(const char *str) const{
    m_dividedDifference.print(str);
}

QuinticPolynomial::QuinticPolynomial(){

}

QuinticPolynomial::QuinticPolynomial(const CMatrix<double> &coefficient){
    this->setCoefficient(coefficient);
}

QuinticPolynomial::~QuinticPolynomial(){

}

void QuinticPolynomial::setCoefficient(const CMatrix<double> &coefficient){
    m_coefficient = coefficient;
}

void QuinticPolynomial::setBoundaryCondition(const CMatrix<double> &boundaryCondition){
    double x1 = boundaryCondition.at(0, 0);
    double f1 = boundaryCondition.at(0, 1);
    double fd1 = boundaryCondition.at(0, 2);
    double fdd1 = boundaryCondition.at(0, 3);
    double x2 = boundaryCondition.at(1, 0);
    double f2 = boundaryCondition.at(1, 1);
    double fd2 = boundaryCondition.at(1, 2);
    double fdd2 = boundaryCondition.at(1, 3);

    if(num_is_zero(x1-x2)){
        m_coefficient = CMatrix<double>(6, 1, {f2, 0, 0, 0, 0, 0});
        return;
//        throw string("Error: QuinticPolynomial::same boundary point");
    }

    double value1[] = {1, x1, pow(x1, 2),   pow(x1, 3),    pow(x1, 4),    pow(x1, 5),
                       1, x2, pow(x2, 2),   pow(x2, 3),    pow(x2, 4),    pow(x2, 5),
                       0,  1,       2*x1, 3*pow(x1, 2),  4*pow(x1, 3),  5*pow(x1, 4),
                       0,  1,       2*x2, 3*pow(x2, 2),  4*pow(x2, 3),  5*pow(x2, 4),
                       0,  0,          2,         6*x1, 12*pow(x1, 2), 20*pow(x1, 3),
                       0,  0,          2,         6*x2, 12*pow(x2, 2), 20*pow(x2, 3)};
    CMatrix<double> matrix1(6, 6, value1);

    double value2[] = {f1, f2, fd1, fd2, fdd1, fdd2};
    CMatrix<double> matrix2(6, 1, value2);

    try{
        m_coefficient = matrix1.inv() * matrix2;
    }
    catch(string e){
        m_coefficient = CMatrix<double>(6, 1, {f2, 0, 0, 0, 0, 0}); // 出现无逆解，可能原因是两点距离过近，以常值返回
    }
}

double QuinticPolynomial::getValue(double x) const{
    double value = 0;
    for(int i=0; i<m_coefficient.rows(); i++){
        value += m_coefficient.at(i, 0)*pow(x, i);
    }
    return value;
}

double QuinticPolynomial::getDerivative(double x, int n) const{
    double value = 0;
    switch (n) {
    case 0:{
        value = getValue(x);
    }
        break;
    case 1:{
        for(int i=1; i<m_coefficient.rows(); i++){
            value += i*m_coefficient.at(i, 0)*pow(x, i-1);
        }
    }
        break;
    case 2:{
        for(int i=2; i<m_coefficient.rows(); i++){
            value += (i-1)*i*m_coefficient.at(i, 0)*pow(x, i-2);
        }
    }
        break;
    default:
        break;
    }
    return value;
}

void QuinticPolynomial::print(const char *str) const{
    m_coefficient.print(str);
}

QuinticNewtonPolynomial::QuinticNewtonPolynomial(){

}

QuinticNewtonPolynomial::QuinticNewtonPolynomial(double period){
    this->setPeriod(period);
}

QuinticNewtonPolynomial::QuinticNewtonPolynomial(double period, const CMatrix<double> &x, const CMatrix<double> &y){
    this->setPeriod(period);
    this->setPoints(x, y);
}

QuinticNewtonPolynomial::QuinticNewtonPolynomial(double period, const CMatrix<double> &x, const CMatrix<double> &y, const CMatrix<double> &yd, const CMatrix<double> &ydd){
    this->setPeriod(period);
    this->setPoints(x, y, yd, ydd);
}

QuinticNewtonPolynomial::~QuinticNewtonPolynomial(){

}

void QuinticNewtonPolynomial::setPeriod(double period){
    m_period = period;
}

void QuinticNewtonPolynomial::setPoints(const CMatrix<double> &x, const CMatrix<double> &y){
    if(x.rows() < 2){
        throw string("Error: QuinticNewtonPolynomial:: less than 2 points");
    }
    m_x = x;
    m_y = y;

    resetState();
    if(x.rows() > 2){
        m_middleSegment.setPoints(x, y);
    }

    CMatrix<double> firstBoundaryCondition(2, 4), endBoundaryCondition(2, 4);
    firstBoundaryCondition.at(0, 0) = x.at(0, 0);
    firstBoundaryCondition.at(0, 1) = y.at(0, 0);
    firstBoundaryCondition.at(0, 2) = (double)0;
    firstBoundaryCondition.at(0, 3) = (double)0;

    endBoundaryCondition.at(1, 0) = x.at(x.rows()-1, 0);
    endBoundaryCondition.at(1, 1) = y.at(y.rows()-1, 0);
    endBoundaryCondition.at(1, 2) = (double)0;
    endBoundaryCondition.at(1, 3) = (double)0;

    if(x.rows() == 2){
        firstBoundaryCondition.at(1, 0) = x.at(x.rows()-1, 0);
        firstBoundaryCondition.at(1, 1) = y.at(y.rows()-1, 0);
        firstBoundaryCondition.at(1, 2) = (double)0;
        firstBoundaryCondition.at(1, 3) = (double)0;

        endBoundaryCondition.at(0, 0) = x.at(0, 0);
        endBoundaryCondition.at(0, 1) = y.at(0, 0);
        endBoundaryCondition.at(0, 2) = (double)0;
        endBoundaryCondition.at(0, 3) = (double)0;
    }
    else{
        firstBoundaryCondition.at(1, 0) = x.at(1, 0);
        firstBoundaryCondition.at(1, 1) = y.at(1, 0);
        firstBoundaryCondition.at(1, 2) = m_middleSegment.getDerivative(x.at(1, 0), 1);
        firstBoundaryCondition.at(1, 3) = m_middleSegment.getDerivative(x.at(1, 0), 2);

        endBoundaryCondition.at(0, 0) = x.at(x.rows()-2, 0);
        endBoundaryCondition.at(0, 1) = y.at(y.rows()-2, 0);
        endBoundaryCondition.at(0, 2) = m_middleSegment.getDerivative(x.at(x.rows()-2, 0), 1);
        endBoundaryCondition.at(0, 3) = m_middleSegment.getDerivative(x.at(x.rows()-2, 0), 2);
    }

    m_firstSegment.setBoundaryCondition(firstBoundaryCondition);
    m_endSegment.setBoundaryCondition(endBoundaryCondition);
}

void QuinticNewtonPolynomial::setPoints(const CMatrix<double> &x, const CMatrix<double> &y, const CMatrix<double> &yd, const CMatrix<double> &ydd){
    if(x.rows() < 2){
        throw string("Error: QuinticNewtonPolynomial:: less than 2 points");
    }
    m_x = x;
    m_y = y;

    resetState();
    if(x.rows() > 2){
        m_middleSegment.setPoints(x, y);
    }

    CMatrix<double> firstBoundaryCondition(2, 4), endBoundaryCondition(2, 4);
    firstBoundaryCondition.at(0, 0) = x.at(0, 0);
    firstBoundaryCondition.at(0, 1) = y.at(0, 0);
    firstBoundaryCondition.at(0, 2) = yd.at(0, 0);
    firstBoundaryCondition.at(0, 3) = ydd.at(0, 0);

    endBoundaryCondition.at(1, 0) = x.at(x.rows()-1, 0);
    endBoundaryCondition.at(1, 1) = y.at(y.rows()-1, 0);
    endBoundaryCondition.at(1, 2) = yd.at(yd.rows()-1, 0);
    endBoundaryCondition.at(1, 3) = ydd.at(ydd.rows()-1, 0);

    if(x.rows() == 2){
        firstBoundaryCondition.at(1, 0) = x.at(x.rows()-1, 0);
        firstBoundaryCondition.at(1, 1) = y.at(y.rows()-1, 0);
        firstBoundaryCondition.at(1, 2) = yd.at(yd.rows()-1, 0);
        firstBoundaryCondition.at(1, 3) = ydd.at(ydd.rows()-1, 0);

        endBoundaryCondition.at(0, 0) = x.at(0, 0);
        endBoundaryCondition.at(0, 1) = y.at(0, 0);
        endBoundaryCondition.at(0, 2) = yd.at(0, 0);
        endBoundaryCondition.at(0, 3) = ydd.at(0, 0);
    }
    else{
        firstBoundaryCondition.at(1, 0) = x.at(1, 0);
        firstBoundaryCondition.at(1, 1) = y.at(1, 0);
        firstBoundaryCondition.at(1, 2) = m_middleSegment.getDerivative(x.at(1, 0), 1);
        firstBoundaryCondition.at(1, 3) = m_middleSegment.getDerivative(x.at(1, 0), 2);

        endBoundaryCondition.at(0, 0) = x.at(x.rows()-2, 0);
        endBoundaryCondition.at(0, 1) = y.at(y.rows()-2, 0);
        endBoundaryCondition.at(0, 2) = m_middleSegment.getDerivative(x.at(x.rows()-2, 0), 1);
        endBoundaryCondition.at(0, 3) = m_middleSegment.getDerivative(x.at(x.rows()-2, 0), 2);
    }

    m_firstSegment.setBoundaryCondition(firstBoundaryCondition);
    m_endSegment.setBoundaryCondition(endBoundaryCondition);
}

void QuinticNewtonPolynomial::resetState(){
    m_index = 1;
    m_time = 0;
}

int QuinticNewtonPolynomial::getSegmentNum() const{
    return m_x.rows()-1;
}

CMatrix<double> QuinticNewtonPolynomial::getNextSegment(){
    if(m_index > getSegmentNum()){
        return CMatrix<double>(0, 0);
    }

    vector<double> pointStateList;
    if(m_index == 1){
        double rightboundary = m_x.at(m_index, 0);
        while(1){
            if(getSegmentNum() == 1){
                if(rightboundary-m_time < GOLDRATIO*m_period){
                    for(int i=0; i<3; i++){
                        pointStateList.push_back(m_firstSegment.getDerivative(rightboundary, i));
                    }
                    break;
                }
            }
            else{
                if(m_time > rightboundary){
                    break;
                }
            }
//            if(m_time > rightboundary){
//                if(getSegmentNum() == 1 && m_time < (rightboundary+m_period-EPSLON)){
//                    for(int i=0; i<3; i++){
//                        pointStateList.push_back(m_firstSegment.getDerivative(rightboundary, i));
//                    }
//                }
//                break;
//            }

            for(int i=0; i<3; i++){
                pointStateList.push_back(m_firstSegment.getDerivative(m_time, i));
            }

            m_time += m_period;
        }
    }
    else if(m_index == getSegmentNum()){
        double rightboundary = m_x.at(m_index, 0);
        while(1){
            if(rightboundary-m_time < GOLDRATIO*m_period){
                for(int i=0; i<3; i++){
                    pointStateList.push_back(m_endSegment.getDerivative(rightboundary, i));
                }
                break;
            }

            for(int i=0; i<3; i++){
                pointStateList.push_back(m_endSegment.getDerivative(m_time, i));
            }

            m_time += m_period;
        }
    }
    else if(m_index < getSegmentNum()){
        double rightboundary = m_x.at(m_index, 0);
        while(1){
            if(m_time > rightboundary){
                break;
            }

            for(int i=0; i<3; i++){
                pointStateList.push_back(m_middleSegment.getDerivative(m_time, i));
            }

            m_time += m_period;
        }
    }

    m_index++;

    return CMatrix<double>(pointStateList.size()/3, 3, pointStateList);
}

CMatrix<double> QuinticNewtonPolynomial::getAllSegment(){
    vector<double> pointStateList;

    while(m_index <= getSegmentNum()){
        if(m_index == 1){
            double rightboundary = m_x.at(m_index, 0);
            while(1){
                if(getSegmentNum() == 1){
                    if(rightboundary-m_time < GOLDRATIO*m_period){
                        for(int i=0; i<3; i++){
                            pointStateList.push_back(m_firstSegment.getDerivative(rightboundary, i));
                        }
                        break;
                    }
                }
                else{
                    if(m_time > rightboundary){
                        break;
                    }
                }

                for(int i=0; i<3; i++){
                    pointStateList.push_back(m_firstSegment.getDerivative(m_time, i));
                }

                m_time += m_period;
            }
        }
        else if(m_index == getSegmentNum()){
            double rightboundary = m_x.at(m_index, 0);
            while(1){
                if(rightboundary-m_time < GOLDRATIO*m_period){
                    for(int i=0; i<3; i++){
                        pointStateList.push_back(m_endSegment.getDerivative(rightboundary, i));
                    }
                    break;
                }

                for(int i=0; i<3; i++){
                    pointStateList.push_back(m_endSegment.getDerivative(m_time, i));
                }

                m_time += m_period;
            }
        }
        else if(m_index < getSegmentNum()){
            double rightboundary = m_x.at(m_index, 0);
            while(1){
                if(m_time > rightboundary){
                    break;
                }

                for(int i=0; i<3; i++){
                    pointStateList.push_back(m_middleSegment.getDerivative(m_time, i));
                }

                m_time += m_period;
            }
        }


        m_index++;
    }

    return CMatrix<double>(pointStateList.size()/3, 3, pointStateList);
}


QuinticContinuousPolynomial::QuinticContinuousPolynomial(){

}

QuinticContinuousPolynomial::QuinticContinuousPolynomial(double period){
    this->setPeriod(period);
}

QuinticContinuousPolynomial::QuinticContinuousPolynomial(double period, const CMatrix<double> &x, const CMatrix<double> &y){
    this->setPeriod(period);
    this->setPoints(x, y);
}

QuinticContinuousPolynomial::QuinticContinuousPolynomial(double period, const CMatrix<double> &x, const CMatrix<double> &y, const CMatrix<double> &yd, const CMatrix<double> &ydd){
    this->setPeriod(period);
    this->setPoints(x, y, yd, ydd);
}

QuinticContinuousPolynomial::~QuinticContinuousPolynomial(){

}

void QuinticContinuousPolynomial::setPeriod(double period){
    m_period = period;
}

void QuinticContinuousPolynomial::setPoints(const CMatrix<double> &x, const CMatrix<double> &y){
    CMatrix<double> yd(y.rows(), y.cols());
    CMatrix<double> ydd(y.rows(), y.cols());
    yd.zeros();
    ydd.zeros();

    this->setPoints(x, y, yd, ydd);
}

void QuinticContinuousPolynomial::setPoints(const CMatrix<double> &x, const CMatrix<double> &y, const CMatrix<double> &yd, const CMatrix<double> &ydd){
    m_x = x;
    m_y = y;
    m_yd = yd;
    m_ydd = ydd;

    resetState();

    for(int i=1; i<m_yd.rows()-1; i++){
        double l1 = m_y.at(i, 0)-m_y.at(i-1, 0);
        double l2 = m_y.at(i+1, 0)-m_y.at(i, 0);
        double v1 = l1/(m_x.at(i, 0)-m_x.at(i-1, 0));
        double v2 = l2/(m_x.at(i+1, 0)-m_x.at(i, 0));
        if(num_is_zero(fabs(l1)+fabs(l2))){
            m_yd.at(i, 0) = (v1 + v2)/2;
        }
        else{
            m_yd.at(i, 0) = (v2-v1)*(fabs(l1))/(fabs(l1)+fabs(l2))+v1;
        }
    }
    for(int i=1; i<m_ydd.rows()-1; i++){
        double l1 = m_y.at(i, 0)-m_y.at(i-1, 0);
        double l2 = m_y.at(i+1, 0)-m_y.at(i, 0);
        double a1 = (m_yd.at(i, 0)-m_yd.at(i-1, 0))/(m_x.at(i, 0)-m_x.at(i-1, 0));
        double a2 = (m_yd.at(i+1, 0)-m_yd.at(i, 0))/(m_x.at(i+1, 0)-m_x.at(i, 0));
        if(num_is_zero(fabs(l1)+fabs(l2))){
            m_ydd.at(i, 0) = (a1 + a2)/2;
        }
        else{
            m_ydd.at(i, 0) = (a2-a1)*(fabs(l1))/(fabs(l1)+fabs(l2))+a1;
        }
    }

//    CMatrix<double> tmpPrint;
//    tmpPrint = m_x;
//    tmpPrint.appendCol(m_y);
//    tmpPrint.appendCol(m_yd);
//    tmpPrint.appendCol(m_ydd);
//    tmpPrint.print();
}

void QuinticContinuousPolynomial::resetState(){
    m_index = 1;
    m_time = 0;
}

int QuinticContinuousPolynomial::getSegmentNum() const{
    return m_x.rows()-1;
}

CMatrix<double> QuinticContinuousPolynomial::getNextSegment(){
    if(m_index > getSegmentNum()){
        return CMatrix<double>(0, 0);
    }

    vector<double> pointStateList;

    double boundaryCondition[] = {m_x.at(m_index-1, 0), m_y.at(m_index-1, 0), m_yd.at(m_index-1, 0), m_ydd.at(m_index-1, 0),
                                 m_x.at(m_index, 0), m_y.at(m_index, 0), m_yd.at(m_index, 0), m_ydd.at(m_index, 0)};
    QuinticPolynomial qp;
    qp.setBoundaryCondition(CMatrix<double>(2, 4, boundaryCondition));

    double rightboundary = m_x.at(m_index, 0);

    while(1){
        if(m_index<getSegmentNum() && m_time > rightboundary){
            break;
        }
        if(m_index==getSegmentNum() && rightboundary-m_time < GOLDRATIO*m_period){
            for(int i=0; i<3; i++){
                pointStateList.push_back(qp.getDerivative(rightboundary, i));
            }
            break;
        }

        for(int i=0; i<3; i++){
            pointStateList.push_back(qp.getDerivative(m_time, i));
        }

        m_time += m_period;
    }

    m_index++;

    return CMatrix<double>(pointStateList.size()/3, 3, pointStateList);
}

CMatrix<double> QuinticContinuousPolynomial::getAllSegment(){
    vector<double> pointStateList;

    while(m_index <= getSegmentNum()){
        double boundaryCondition[] = {m_x.at(m_index-1, 0), m_y.at(m_index-1, 0), m_yd.at(m_index-1, 0), m_ydd.at(m_index-1, 0),
                                     m_x.at(m_index, 0), m_y.at(m_index, 0), m_yd.at(m_index, 0), m_ydd.at(m_index, 0)};
        QuinticPolynomial qp;
        qp.setBoundaryCondition(CMatrix<double>(2, 4, boundaryCondition));

        double rightboundary = m_x.at(m_index, 0);
        while(1){
            if(m_index<getSegmentNum() && m_time > rightboundary){
                break;
            }
            if(m_index==getSegmentNum() && rightboundary-m_time < GOLDRATIO*m_period){
                for(int i=0; i<3; i++){
                    pointStateList.push_back(qp.getDerivative(rightboundary, i));
                }
                break;
            }

            for(int i=0; i<3; i++){
                pointStateList.push_back(qp.getDerivative(m_time, i));
            }

            m_time += m_period;
        }

        m_index++;
    }

    return CMatrix<double>(pointStateList.size()/3, 3, pointStateList);
}


JOGPolynomial::JOGPolynomial(){

}

JOGPolynomial::JOGPolynomial(double period){
    this->setPeriod(period);
}

JOGPolynomial::JOGPolynomial(double period, double vel, double acc, double jerk){
    this->setPeriod(period);
    this->setMotionCoefficient(vel, acc, jerk);
}

JOGPolynomial::~JOGPolynomial(){

}

void JOGPolynomial::setPeriod(double period){
    m_period = period;
}

void JOGPolynomial::setMotionCoefficient(double vel, double acc, double jerk){
    m_vmax = vel;
    m_amax = acc;
    m_jmax = jerk;

    if(m_vmax*m_jmax <= pow(m_amax, 2)){
        m_t1 = sqrt(m_vmax/m_jmax);
        m_t2 = m_t1;
        m_t3 = m_t1+m_t2;
    }
    else{
        m_t1 = m_amax / m_jmax;
        m_t2 = (m_vmax-m_jmax*pow(m_t1, 2))/m_amax+m_t1;
        m_t3 = m_t1+m_t2;
    }

    m_v1 = m_jmax*pow(m_t1, 2)/2;
    m_v2 = m_v1+m_amax*(m_t2-m_t1);
    m_v3 = m_v1+m_v2;

    m_p1 = m_jmax*pow(m_t1, 3)/6;
    m_p2 = m_p1+m_v3*(m_t2-m_t1)/2;
    m_p3 = m_v3*m_t3/2;

    resetState();
}

void JOGPolynomial::resetState(){
    m_time = 0;
    m_vs = 0;
    m_ps = 0;
    m_moveEndFlag = 0;
}

bool JOGPolynomial::isEnd() const{
    if(m_moveEndFlag == 2){
        return true;
    }
    return false;
}

CMatrix<double> JOGPolynomial::getNextPoint(){
    CMatrix<double> samplePoint(1, 3);
    double pos, vel, acc;

    if(m_moveEndFlag == 0){
        if(m_time < m_t1){
            m_ps = m_jmax*pow(m_time, 3)/6;
            m_vs = m_jmax*pow(m_time, 2)/2;
            acc = m_jmax*m_time;
        }
        else if(m_time < m_t2){
            m_ps = m_p1+(2*m_v1+m_amax*(m_time-m_t1))*(m_time-m_t1)/2;
            m_vs = m_v1+m_amax*(m_time-m_t1);
            acc = m_amax;
        }
        else if(m_time < m_t3){
            m_ps = m_p3-(m_v3*(m_t3-m_time)-m_jmax*pow(m_t3-m_time, 3)/6);
            m_vs = m_v3-m_jmax*pow(m_t3-m_time, 2)/2;
            acc = m_jmax*(m_t3-m_time);
        }
        else{
            m_ps = m_p3+m_v3*(m_time-m_t3);
            m_vs = m_v3;
            acc = 0;
        }
        pos = m_ps;
        vel = m_vs;
    }
    else{
        if(m_time >= m_t5){
            m_time = m_t5;
            m_moveEndFlag = 2;
        }

        pos = m_ps+(2*m_vs-m_amax*(m_time-m_t4))*(m_time-m_t4)/2;
        vel = m_vs-m_amax*(m_time-m_t4);
        acc = -m_amax;
    }

    samplePoint.at(0, 0) = pos;
    samplePoint.at(0, 1) = vel;
    samplePoint.at(0, 2) = acc;

    m_time += m_period;

    return samplePoint;
}

void JOGPolynomial::setEnd(){
    if(m_moveEndFlag == 0){
        m_t4 = m_time - m_period;
        m_t5 = m_t4 + m_vs/m_amax;

        m_moveEndFlag = 1;
    }
}


SPolynomial::SPolynomial(){

}

SPolynomial::SPolynomial(double period){
    this->setPeriod(period);
}

SPolynomial::~SPolynomial(){

}

void SPolynomial::setPeriod(double period){
    m_period = period;
}

void SPolynomial::setMotionCoefficientWithVel(double len, double vel, double acc, double jerk){
    m_length = len;
    if(num_is_zero(m_length)){
        return;
    }

    m_vmax = vel;
    m_amax = acc;
    m_jmax = jerk;

    if(fabs(m_vmax*m_jmax) <= fabs(m_amax*m_amax)){
        if(fabs(m_length) <= fabs(2*m_vmax*sqrt(m_vmax / m_jmax))){
//            cout << "step situation 1-1" << endl;
            m_t1 = pow(m_length / (2 * m_jmax), 1.0 / 3);
            m_t2 = m_t1;
            m_t3 = m_t2 + m_t1;
            m_t4 = m_t3;
            m_t5 = m_t4 + m_t1;
            m_t6 = m_t5;
            m_t7 = m_t6 + m_t1;
        }
        else{
//            cout << "step situation 1-2" << endl;
            m_t1 = sqrt(m_vmax / m_jmax);
            m_t2 = m_t1;
            m_t3 = m_t2 + m_t1;
            m_t4 = m_length / m_vmax;
            m_t5 = m_t4 + m_t1;
            m_t6 = m_t5;
            m_t7 = m_t6 + m_t1;
        }
    }
    else{
        if(fabs(m_length) <= fabs(2 * pow(m_amax, 3) / pow(m_jmax, 2))){
//            cout << "step situation 2-1" << endl;
            m_t1 = pow(m_length / (2 * m_jmax), 1.0 / 3);
            m_t2 = m_t1;
            m_t3 = m_t2 + m_t1;
            m_t4 = m_t3;
            m_t5 = m_t4 + m_t1;
            m_t6 = m_t5;
            m_t7 = m_t6 + m_t1;
        }
        else if(fabs(m_length) > fabs(m_vmax * (m_amax / m_jmax + m_vmax / m_amax))){
//            cout << "step situation 2-3" << endl;
            m_t1 = m_amax / m_jmax;
            m_t2 = m_t1 + (m_vmax - m_jmax * pow(m_t1, 2)) / m_amax;
            m_t3 = m_t2 + m_t1;
            m_t4 = m_length / m_vmax;
            m_t5 = m_t4 + m_t1;
            m_t6 = m_t4 + m_t2;
            m_t7 = m_t6 + m_t1;
        }
        else{
//            cout << "step situation 2-2" << endl;
            m_t1 = m_amax / m_jmax;
            double a = m_amax;
            double b = m_jmax * pow(m_t1, 2);
            double c = - m_length;
            if(m_vmax > 0){
                m_t2 = (-b+sqrt(pow(b, 2)-4*a*c))/(2*a);
            }
            else{
                m_t2 = (-b-sqrt(pow(b, 2)-4*a*c))/(2*a);
            }
            m_t3 = m_t2 + m_t1;
            m_t4 = m_t3;
            m_t5 = m_t4 + m_t1;
            m_t6 = m_t4 + m_t2;
            m_t7 = m_t6 + m_t1;
        }
    }

    m_v1 = m_jmax * pow(m_t1, 2) / 2;
    m_v2 = m_v1 + m_amax * (m_t2 - m_t1);
    m_v3 = m_v1 + m_v2;

    m_p1 = m_jmax * pow(m_t1, 3) / 6;
    m_p2 = m_p1 + m_v3 * (m_t2 - m_t1) / 2;
    m_p3 = m_v3 * m_t3 / 2;
    m_p4 = m_p3 + m_v3 * (m_t4 - m_t3);
    m_p5 = m_p4 + m_p3 - m_p2;
    m_p6 = m_p4 + m_p3 - m_p1;
    m_p7 = m_length;
}

int SPolynomial::setMotionCoefficientWithTime(double len, double time, double acc, double jerk){
    m_length = len;
    if(num_is_zero(m_length)){
        return 0;
    }

    if(time > 4*acc/jerk){
        if(fabs(len) > fabs(acc * pow(time, 2)/4 - (pow(acc, 2)*time)/(2*jerk))){
            return 1;
        }
    }
    else{
        if(fabs(len) > fabs(jerk*pow(time, 3)/32)){
            return 1;
        }
    }

    double vel;
    if(time > 4*acc/jerk && fabs(len) > fabs(time - 2*acc/jerk)*pow(acc, 2)/jerk){    // 加速到最大加速度的情况
        double a = 1;
        double b = 3*acc/jerk - time;
        double c = len/acc + 2*pow(acc, 2)/pow(jerk, 2) - acc*time/jerk;
        vector<double> res = solveQuadraticEquation(a, b, c);
        if(res.empty()){
            return 1;
        }
        for(int i=0; i<res.size(); i++){
            if(res[i]>0 && res[i]<((time-4*acc/jerk)/2)){
                double t = res[i];
                vel = pow(acc, 2)/jerk + acc*t;
                break;
            }
            if(i == res.size()-1){
                return 1;
            }
        }
    }
    else{   // 不能加速到最大加速度的情况
        double a = 2*jerk;
        double b = -jerk*time;
        double c = 0;
        double d = len;
        vector<double> res = solveCubicEquation(a, b, c, d);
        if(res.empty()){
            return 1;
        }
        for(int i=0; i<res.size(); i++){
            if(res[i]>0 && res[i]<(time/4)){
                double t = res[i];
                vel = jerk*pow(t, 2);
                break;
            }
            if(i == res.size()-1){
                return 1;
            }
        }
    }


    this->setMotionCoefficientWithVel(len, vel, acc, jerk);
    return 0;
}

double SPolynomial::getTime(double len) const{
    if(num_is_zero(m_length)){
        return 0;
    }

    if(len>0 && len<=m_p1){
        double a = m_jmax/6;
        double b = 0;
        double c = 0;
        double d = -len;
        vector<double> value = solveCubicEquation(a, b, c, d);
        for(int i=0; i<value.size(); i++){
            if(value[i]>=0-EPSLON && value[i]<=m_t1+EPSLON){
                return value[i];
            }
        }
    }
    else if(len>m_p1 && len<=m_p2){
        double a = m_amax/2;
        double b = m_v1;
        double c = m_p1-len;
        vector<double> value = solveQuadraticEquation(a, b, c);
        for(int i=0; i<value.size(); i++){
            value[i] = m_t1+value[i];
            if(value[i]>=m_t1-EPSLON && value[i]<=m_t2+EPSLON){
                return value[i];
            }
        }
    }
    else if(len>m_p2 && len<=m_p3){
        double a = m_jmax/6;
        double b = 0;
        double c = -m_v3;
        double d = m_p3-len;
        vector<double> value = solveCubicEquation(a, b, c, d);
        for(int i=0; i<value.size(); i++){
            value[i] = m_t3-value[i];
            if(value[i]>=m_t2-EPSLON && value[i]<=m_t3+EPSLON){
                return value[i];
            }
        }
    }
    else if(len>m_p3 && len<=m_p4){
        double value = (len-m_p3)/m_v3+m_t3;
        if(value>=m_t3-EPSLON && value<=m_t4+EPSLON){
            return value;
        }
    }
    else if(len>m_p4 && len<=m_p5){
        double a = -m_jmax/6;
        double b = 0;
        double c = m_v3;
        double d = m_p4-len;
        vector<double> value = solveCubicEquation(a, b, c, d);
        for(int i=0; i<value.size(); i++){
            value[i] = m_t4+value[i];
            if(value[i]>=m_t4-EPSLON && value[i]<=m_t5+EPSLON){
                return value[i];
            }
        }
    }
    else if(len>m_p5 && len<=m_p6){
        double a = -m_amax/2;
        double b = m_v2;
        double c = m_p5-len;
        vector<double> value = solveQuadraticEquation(a, b, c);
        for(int i=0; i<value.size(); i++){
            value[i] = m_t5+value[i];
            if(value[i]>=m_t5-EPSLON && value[i]<=m_t6+EPSLON){
                return value[i];
            }
        }
    }
    else if(len>m_p6 && len<=m_p7+EPSLON){
        double a = -m_jmax/6;
        double b = 0;
        double c = 0;
        double d = m_p7-len;
        vector<double> value = solveCubicEquation(a, b, c, d);
        for(int i=0; i<value.size(); i++){
            value[i] = m_t7-value[i];
            if(value[i]>=m_t6-EPSLON && value[i]<=m_t7+EPSLON){
                return value[i];
            }
        }
    }

    return -1;
}

double SPolynomial::getValue(double x) const{
    if(num_is_zero(m_length)){
        return 0;
    }

    if(x>0 && x<=m_t1){
        return m_jmax*pow(x,3)/6;
    }
    else if(x>m_t1 && x<=m_t2){
        return m_p1+(2*m_v1+m_amax*(x-m_t1))*(x-m_t1)/2;
    }
    else if(x>m_t2 && x<= m_t3){
        return m_p3-(m_v3*(m_t3-x)-m_jmax*pow(m_t3-x,3)/6);
    }
    else if(x>m_t3 && x<= m_t4){
        return m_p3+m_v3*(x-m_t3);
    }
    else if(x>m_t4 && x<= m_t5){
        return m_p4+(m_v3*(x-m_t4)-m_jmax*pow(x-m_t4,3)/6);
    }
    else if(x>m_t5 && x<= m_t6){
        return m_p5+(2*m_v2-m_amax*(x-m_t5))*(x-m_t5)/2;
    }
    else if(x>m_t6){
        return m_p7-m_jmax*pow(m_t7-x,3)/6;
    }
}

double SPolynomial::getDerivative(double x, int n) const{
    if(num_is_zero(m_length)){
        return 0;
    }

    double value = 0;
    switch (n) {
    case 0:{
        value = getValue(x);
    }
        break;
    case 1:{
        if(x>0 && x<=m_t1){
            value = m_jmax*pow(x,2)/2;
        }
        else if(x>m_t1 && x<=m_t2){
            value = m_v1+m_amax*(x-m_t1);
        }
        else if(x>m_t2 && x<= m_t3){
            value = m_v3-m_jmax*pow(m_t3-x,2)/2;
        }
        else if(x>m_t3 && x<= m_t4){
            value = m_v3;
        }
        else if(x>m_t4 && x<= m_t5){
            value = m_v3-m_jmax*pow(x-m_t4,2)/2;
        }
        else if(x>m_t5 && x<= m_t6){
            value = m_v1+m_amax*(m_t6-x);
        }
        else if(x>m_t6){
            value = m_jmax*pow(m_t7-x,2)/2;
        }
    }
        break;
    case 2:{
        if(x>0 && x<=m_t1){
            value = m_jmax*x;
        }
        else if(x>m_t1 && x<=m_t2){
            value = m_amax;
        }
        else if(x>m_t2 && x<= m_t3){
            value = m_jmax*(m_t3-x);
        }
        else if(x>m_t3 && x<= m_t4){
            value = 0;
        }
        else if(x>m_t4 && x<= m_t5){
            value = -m_jmax*(x-m_t4);
        }
        else if(x>m_t5 && x<= m_t6){
            value = -m_amax;
        }
        else if(x>m_t6){
            value = -m_jmax*(m_t7-x);
        }
    }
        break;
    default:
        break;
    }
    return value;
}

double SPolynomial::getAccLength() const{
    if(num_is_zero(m_length)){
        return 0;
    }
    return m_p3;
}

void SPolynomial::resetState(){

}

CMatrix<double> SPolynomial::getAllSegment(){
    double time = 0;
    vector<double> pointStateList;
    double p, v, a;

    if(num_is_zero(m_length)){
        pointStateList.push_back(0);
        pointStateList.push_back(0);
        pointStateList.push_back(0);
        pointStateList.push_back(0);
        pointStateList.push_back(0);
        pointStateList.push_back(0);
        return CMatrix<double>(pointStateList.size()/3, 3, pointStateList);
    }

    while(1){
        if(time <= m_t1){
            p = m_jmax * pow(time, 3) / 6;
            v = m_jmax * pow(time, 2) / 2;
            a = m_jmax * time;
        }
        else if(time <= m_t2){
            p = m_p1 + (2 * m_v1 + m_amax * (time - m_t1)) * (time - m_t1) / 2;
            v = m_v1 + m_amax * (time - m_t1);
            a = m_amax;
        }
        else if(time <= m_t3){
            p = m_p3 - (m_v3 * (m_t3 - time) - m_jmax * pow(m_t3 - time, 3) / 6);
            v = m_v3 - m_jmax * pow(m_t3 - time, 2) / 2;
            a = m_jmax * (m_t3 -time);
        }
        else if(time <= m_t4){
            p = m_p3 + m_v3 * (time - m_t3);
            v = m_v3;
            a = 0;
        }
        else if(time <= m_t5){
            p = m_p4 + (m_v3 * (time - m_t4) - (m_jmax * pow(time - m_t4, 3)) / 6);
            v = m_v3 - m_jmax * pow(time - m_t4, 2) / 2;
            a = -m_jmax * (time - m_t4);
        }
        else if(time <= m_t6){
            p = m_p5 + (2 * m_v2 - m_amax * (time - m_t5)) * (time - m_t5) / 2;
            v = m_v1 + m_amax * (m_t6 - time);
            a = -m_amax;
        }
        else if(time <= m_t7){
            p = m_p7 - m_jmax * pow(m_t7 - time, 3) / 6;
            v = m_jmax * pow(m_t7 - time, 2) / 2;
            a = -m_jmax * (m_t7 -time);
        }
        else{
            if(time < m_t7 + m_period - EPSLON){
                p = m_p7;
                v = 0;
                a = 0;
            }
            else{
                break;
            }
        }

        pointStateList.push_back(p);
        pointStateList.push_back(v);
        pointStateList.push_back(a);
        time += m_period;
    }
    return CMatrix<double>(pointStateList.size()/3, 3, pointStateList);
}

SPolynomialGen::SPolynomialGen(){

}

SPolynomialGen::SPolynomialGen(double period){
    this->setPeriod(period);
}

SPolynomialGen::SPolynomialGen(double period, double len, double vel, double acc, double jerk){
    this->setPeriod(period);
    this->setMotionCoefficient(len, vel, acc, jerk);
}

SPolynomialGen::~SPolynomialGen(){

}

void SPolynomialGen::setPeriod(double period){
    m_period = period;
}

void SPolynomialGen::setMotionCoefficient(double len, double vel, double acc, double jerk){
    m_length = len;
    m_vmax = vel;
    m_amax = acc;
    m_jmax = jerk;
}

void SPolynomialGen::setVelMode(double vel, double startVel, double startAcc, double endVel, double endAcc){
    m_mode = 0;
    m_vel = vel;
    m_sVel = startVel;
    m_sAcc = startAcc;
    m_eVel = endVel;
    m_eAcc = endAcc;
}

void SPolynomialGen::setTimeMode(double time, double startVel, double startAcc, double endVel, double endAcc){
    m_mode = 1;
    m_time = time;
    m_sVel = startVel;
    m_sAcc = startAcc;
    m_eVel = endVel;
    m_eAcc = endAcc;
}

void SPolynomialGen::resetState(){

}

CMatrix<double> SPolynomialGen::getAllSegment(){
	return CMatrix<double>();
}

#endif
