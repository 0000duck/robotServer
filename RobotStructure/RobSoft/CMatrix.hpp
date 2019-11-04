#ifndef CMATRIX_CPP
#define CMATRIX_CPP

#include "CMathBasic.hpp"
#include "CMatrixH.hpp"
#include <iostream>
#include <string>
#include <string.h>
#include <cmath>
#include <typeinfo>
#include <iomanip>

namespace robsoft{

template<typename T>
CMatrix<T>::CMatrix(){
    indentifyType();

    m_row = 0;
    m_col = 0;
    m_dim = 1;
    m_array = new T[m_row * m_col * m_dim];

    this->zeros();
}

template<typename T>
CMatrix<T>::CMatrix(int row, int col){
    indentifyType();

    m_row = row;
    m_col = col;
    m_dim = 1;
    m_array = new T[m_row * m_col * m_dim];

    this->zeros();
}

template<typename T>
CMatrix<T>::CMatrix(int row, int col, int dim){
    indentifyType();

    if(row+col+dim == 0){
        dim = 1;
    }

    m_row = row;
    m_col = col;
    m_dim = dim;
    m_array = new T[m_row * m_col * m_dim];

    this->zeros();
}

template<typename T>
CMatrix<T>::CMatrix(int row, int col, T* array){
//    if(row * col != arraySize(array)){
//        throw std::string("Error: CMatrix<T>::invalid dimension");
//    }

    indentifyType();

    m_row = row;
    m_col = col;
    m_dim = 1;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
CMatrix<T>::CMatrix(int row, int col, int dim, T* array){
//    if(row * col * dim != arraySize(array)){
//        throw std::string("Error: CMatrix<T>::invalid dimension");
//    }

    indentifyType();

    if(row+col+dim == 0){
        dim = 1;
    }

    m_row = row;
    m_col = col;
    m_dim = dim;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
CMatrix<T>::CMatrix(int row, int col, std::vector<T> array){
    if(row * col != array.size()){
        throw std::string("Error: CMatrix<T>::invalid dimension");
    }

    indentifyType();

    m_row = row;
    m_col = col;
    m_dim = 1;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
CMatrix<T>::CMatrix(int row, int col, int dim, std::vector<T> array){
    if(row * col * dim != array.size()){
        throw std::string("Error: CMatrix<T>::invalid dimension");
    }

    indentifyType();

    m_row = row;
    m_col = col;
    m_dim = dim;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
CMatrix<T>::CMatrix(const CMatrix& matrix){
    indentifyType();

    m_row = matrix.rows();
    m_col = matrix.cols();
    m_dim = matrix.dims();
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = matrix.m_array[i];
    }
}

template<typename T>
CMatrix<T>::~CMatrix(){
    delete[] m_array;
}

template<typename T>
void CMatrix<T>::setValue(int row, int col){
    m_row = row;
    m_col = col;
    m_dim = 1;

    delete[] m_array;
    m_array = new T[m_row * m_col * m_dim];

    this->zeros();
}

template<typename T>
void CMatrix<T>::setValue(int row, int col, int dim){
    m_row = row;
    m_col = col;
    m_dim = dim;

    delete[] m_array;
    m_array = new T[m_row * m_col * m_dim];

    this->zeros();
}

template<typename T>
void CMatrix<T>::setValue(int row, int col, T* array){
//    if(row * col != arraySize(array)){
//        throw std::string("Error: CMatrix<T>::invalid dimension");
//    }

    m_row = row;
    m_col = col;
    m_dim = 1;

    delete[] m_array;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
void CMatrix<T>::setValue(int row, int col, int dim, T* array){
//    if(row * col * dim != arraySize(array)){
//        throw std::string("Error: CMatrix<T>::invalid dimension");
//    }

    m_row = row;
    m_col = col;
    m_dim = dim;

    delete[] m_array;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
void CMatrix<T>::setValue(int row, int col, std::vector<T> array){
    if(row * col != array.size()){
        throw std::string("Error: CMatrix<T>::invalid dimension");
    }

    m_row = row;
    m_col = col;
    m_dim = 1;

    delete[] m_array;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
void CMatrix<T>::setValue(int row, int col, int dim, std::vector<T> array){
    if(row * col * dim != array.size()){
        throw std::string("Error: CMatrix<T>::invalid dimension");
    }

    m_row = row;
    m_col = col;
    m_dim = dim;

    delete[] m_array;
    m_array = new T[m_row * m_col * m_dim];

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = array[i];
    }
}

template<typename T>
void CMatrix<T>::zeros(){
    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = 0;
    }
}

template<typename T>
void CMatrix<T>::zeros(int row, int col){
    if(m_row != row || m_col != col || m_dim != 1){
        m_row = row;
        m_col = col;
        m_dim = 1;

        delete[] m_array;
        m_array = new T[m_row * m_col * m_dim];
    }

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = 0;
    }
}

template<typename T>
void CMatrix<T>::zeros(int row, int col, int dim){
    if(m_row != row || m_col != col || m_dim != dim){
        m_row = row;
        m_col = col;
        m_dim = dim;

        delete[] m_array;
        m_array = new T[m_row * m_col * m_dim];
    }

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = 0;
    }
}

template<typename T>
void CMatrix<T>::eye(){
    if(m_row != m_col){
        throw std::string("Error: CMatrix<T>::invalid dimension, rows should equal column");
    }

    zeros();
    for(int i=0; i<m_row; i++){
        for(int k=0; k<m_dim; k++){
            m_array[k*m_col*m_dim+i*m_col+i] = 1;
        }
    }
}

template<typename T>
void CMatrix<T>::eye(int row, int col){
    if(row != col){
        throw std::string("Error: CMatrix<T>::invalid dimension, rows should equal column");
    }

    if(m_row != row || m_col != col || m_dim != 1){
        m_row = row;
        m_col = col;
        m_dim = 1;

        delete[] m_array;
        m_array = new T[m_row * m_col * m_dim];
    }

    zeros();
    for(int i=0; i<m_row; i++){
        for(int k=0; k<m_dim; k++){
            m_array[k*m_col*m_row+i*m_col+i] = 1;
        }
    }
}

template<typename T>
void CMatrix<T>::eye(int row, int col, int dim){
    if(row != col){
        throw std::string("Error: CMatrix<T>::invalid dimension, rows should equal column");
    }

    if(m_row != row || m_col != col || m_dim != dim){
        m_row = row;
        m_col = col;
        m_dim = dim;

        delete[] m_array;
        m_array = new T[m_row * m_col * m_dim];
    }

    zeros();
    for(int i=0; i<m_row; i++){
        for(int k=0; k<m_dim; k++){
            m_array[k*m_col*m_row+i*m_col+i] = 1;
        }
    }
}

template<typename T>
void CMatrix<T>::appendCol(const CMatrix &matrix){
    if(this->rows() != matrix.rows() || this->dims() != matrix.dims()){
        throw std::string("Error: CMatrix<T>:: number of rows or dims not equal");
    }

    CMatrix<T> temp(m_row, m_col+matrix.cols(), m_dim);

    for(int i=0; i<temp.rows(); i++){
        for(int j=0; j<temp.cols(); j++){
            for(int k=0; k<temp.dims(); k++){
                if(j < m_col){
                    temp.at(i, j, k) = this->at(i, j, k);
                }
                else{
                    temp.at(i, j, k) = matrix.at(i, j-m_col, k);
                }
            }
        }
    }

    *this = temp;
}

template<typename T>
void CMatrix<T>::appendRow(const CMatrix &matrix){
    if(this->cols() != matrix.cols() || this->dims() != matrix.dims()){
        throw std::string("Error: CMatrix<T>:: number of rows or dims not equal");
    }

    CMatrix<T> temp(m_row+matrix.rows(), m_col, m_dim);

    for(int i=0; i<temp.rows(); i++){
        for(int j=0; j<temp.cols(); j++){
            for(int k=0; k<temp.dims(); k++){
                if(i < m_row){
                    temp.at(i, j, k) = this->at(i, j, k);
                }
                else{
                    temp.at(i, j, k) = matrix.at(i-m_row, j, k);
                }
            }
        }
    }

    *this = temp;
}

template<typename T>
int CMatrix<T>::getElementNumber() const{
    return m_row * m_col * m_dim;
}

template<typename T>
T& CMatrix<T>::at(int i, int j){
    if(m_dim > 1){
        throw std::string("Error: CMatrix<T>::please specify the channel index");
    }
    if(i<0 || i>=m_row || j<0 || j>=m_col){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }
    return m_array[i*m_col+j];
}

template<typename T>
T& CMatrix<T>::at(int i, int j, int k){
    if(i<0 || i>=m_row || j<0 || j>=m_col || k<0 || k>=m_dim){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }
    return m_array[k*m_row*m_col+i*m_col+j];
}

template<typename T>
const T& CMatrix<T>::at(int i, int j) const{
    if(m_dim > 1){
        throw std::string("Error: CMatrix<T>::please specify the channel index");
    }
    if(i<0 || i>=m_row || j<0 || j>=m_col){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }
    return m_array[i*m_col+j];
}

template<typename T>
const T& CMatrix<T>::at(int i, int j, int k) const{
    if(i<0 || i>=m_row || j<0 || j>=m_col || k<0 || k>=m_dim){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }
    return m_array[k*m_row*m_col+i*m_col+j];
}

template<typename T>
void CMatrix<T>::replace(int i, int j, T value){
    if(m_dim > 1){
        throw std::string("Error: CMatrix<T>::please specify the channel index");
    }
    if(i<0 || i>=m_row || j<0 || j>=m_col){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }
    m_array[i*m_col+j] = value;
}

template<typename T>
void CMatrix<T>::replace(int i, int j, int k, T value){
    if(i<0 || i>=m_row || j<0 || j>=m_col || k<0 || k>=m_dim){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }
    m_array[k*m_row*m_col+i*m_col+j] = value;
}

template<typename T>
int CMatrix<T>::rows() const{
    return m_row;
}

template<typename T>
int CMatrix<T>::cols() const{
    return m_col;
}

template<typename T>
int CMatrix<T>::dims() const{
    return m_dim;
}

template<typename T>
DATATYPE CMatrix<T>::types() const{
    return m_type;
}

template<typename T>
CMatrix<T> CMatrix<T>::clone() const{
    return CMatrix<T>(*this);
}

template<typename T>
CMatrix<T> CMatrix<T>::operator *(double value) const{
    return this->mul(value);
}

template<typename T>
CMatrix<T> CMatrix<T>::operator *(const CMatrix& matrix) const{
    return this->mul(matrix);
}

template<typename T>
CMatrix<T> CMatrix<T>::operator +(const CMatrix& matrix) const{
    return this->add(matrix);
}

template<typename T>
CMatrix<T> CMatrix<T>::operator -(const CMatrix& matrix) const{
    return this->sub(matrix);
}

template<typename T>
void CMatrix<T>::operator =(const CMatrix& matrix){
    if(m_row != matrix.rows() || m_col != matrix.cols() || m_dim != matrix.dims()){
        m_row = matrix.rows();
        m_col = matrix.cols();
        m_dim = matrix.dims();

        if(m_array!=nullptr){
            delete[] m_array;
            m_array = nullptr;
        }
        m_array = new T[m_row * m_col * m_dim];
    }

    for(int i=0; i<m_row * m_col * m_dim; i++){
        m_array[i] = matrix.m_array[i];
    }
}

template<typename T>
bool CMatrix<T>::operator ==(const CMatrix& matrix) const{
    if(m_row != matrix.rows() || m_col != matrix.cols() || m_dim != matrix.dims() || m_type != matrix.types()){
        return false;
    }

    for(int i=0; i<m_row*m_col*m_dim; i++){
        if(m_array[i] != matrix.m_array[i]){
            return false;
        }
    }

    return true;
}

template<typename T>
bool CMatrix<T>::operator !=(const CMatrix& matrix) const{
    return !(*this==matrix);
}

template<typename T>
CMatrix<T> CMatrix<T>::mul(double value) const{
    int row = m_row;
    int col = m_col;
    int dim = m_dim;
    CMatrix<T> dstMat;
    dstMat.zeros(row, col, dim);
    for(int i=0; i<row*col*dim; i++){
        dstMat.m_array[i] = m_array[i]*value;
    }
    return dstMat;
}

template<typename T>
CMatrix<T> CMatrix<T>::mul(const CMatrix& matrix) const{
    if(m_col != matrix.rows() || m_dim != matrix.dims() || m_type != matrix.types()){
        throw std::string("Error: CMatrix<T>::invalid dimension or type");
    }

    int row = m_row;
    int col = matrix.cols();
    int dim = m_dim;
    CMatrix<T> dstMat;
    dstMat.zeros(row, col, dim);
    for(int i=0; i<row; i++){
        for(int j=0; j<col; j++){
            for(int k=0; k<dim; k++){
                T element = 0;
                for(int m=0; m<m_col; m++){
                    element += this->at(i, m, k) * matrix.at(m, j, k);
                }
                dstMat.at(i, j, k) = element;
            }
        }
    }
    return dstMat;
}

template<typename T>
CMatrix<T> CMatrix<T>::add(const CMatrix& matrix) const{
    if(m_row != matrix.rows() || m_col != matrix.cols() || m_dim != matrix.dims() || m_type != matrix.types()){
        throw std::string("Error: CMatrix<T>::invalid dimension or type");
    }

    int row = m_row;
    int col = m_col;
    int dim = m_dim;
    CMatrix<T> dstMat;
    dstMat.zeros(row, col, dim);
    for(int i=0; i<row*col*dim; i++){
        dstMat.m_array[i] = m_array[i] + matrix.m_array[i];
    }
    return dstMat;
}

template<typename T>
CMatrix<T> CMatrix<T>::sub(const CMatrix& matrix) const{
    if(m_row != matrix.rows() || m_col != matrix.cols() || m_dim != matrix.dims() || m_type != matrix.types()){
        throw std::string("Error: CMatrix<T>::invalid dimension or type");
    }

    int row = m_row;
    int col = m_col;
    int dim = m_dim;
    CMatrix<T> dstMat;
    dstMat.zeros(row, col, dim);
    for(int i=0; i<row*col*dim; i++){
        dstMat.m_array[i] = m_array[i] - matrix.m_array[i];
    }
    return dstMat;
}

template<typename T>
T CMatrix<T>::dot(const CMatrix& matrix) const{
    if(m_row != matrix.rows() || m_col != matrix.cols() || m_dim != matrix.dims() || m_type != matrix.types()){
        throw std::string("Error: CMatrix<T>::invalid dimension or type");
    }

    T res = 0;
    for(int i=0; i<m_row*m_col*m_dim; i++){
        res += m_array[i] * matrix.m_array[i];
    }
    return res;
}

template<typename T>
CMatrix<T> CMatrix<T>::cross(const CMatrix& matrix) const{
    if(m_dim != 1 || m_col != 1 || matrix.m_dim != 1 || matrix.m_col != 1){
        throw std::string("Error: CMatrix<T>::must be 1-dim horizontal vector");
    }
    if(m_row != 3 || matrix.m_row != 3){
        throw std::string("Error: CMatrix<T>::must include 3 elements");
    }
    CMatrix<T> dstMat;
    dstMat.zeros(3, 1, 1);
    dstMat.m_array[0] = m_array[1]*matrix.m_array[2]-m_array[2]*matrix.m_array[1];
    dstMat.m_array[1] = m_array[2]*matrix.m_array[0]-m_array[0]*matrix.m_array[2];
    dstMat.m_array[2] = m_array[0]*matrix.m_array[1]-m_array[1]*matrix.m_array[0];
    return dstMat;
}

template<typename T>
CMatrix<T> CMatrix<T>::trans() const{
    CMatrix<T> dstMat(m_col, m_row, m_dim);
    for(int i=0; i<m_col; i++){
        for(int j=0; j<m_row; j++){
            for(int k=0; k<m_dim; k++){
                dstMat.at(i, j, k) = this->at(j,i,k);
            }
        }
    }
    return dstMat;
}

template<typename T>
T CMatrix<T>::norm() const{
    T res = 0;
    for(int i=0; i<m_row; i++){
        for(int j=0; j<m_col; j++){
            for(int k=0; k<m_dim; k++){
                res += pow(this->at(i, j, k), 2);
            }
        }
    }
    return sqrt(res);
}

template<typename T>
CMatrix<T> CMatrix<T>::normalization() const{
    T n = this->norm();

    if(num_is_zero(n)){
        throw std::string("Error: CMatrix<T>::the norm equals zero!");
    }

    CMatrix<T> dstMat(m_row, m_col, m_dim);
    for(int i=0; i<m_row; i++){
        for(int j=0; j<m_col; j++){
            for(int k=0; k<m_dim; k++){
                dstMat.at(i, j, k) = this->at(i, j, k)/n;
            }
        }
    }
    return dstMat;
}

template<typename T>
CMatrix<T> CMatrix<T>::inv(DECOMPMETHOD method) const{
    if(m_dim != 1){
        throw std::string("Error: CMatrix<T>::must be 1-dim matrix");
    }
    if(m_row != m_col){
        throw std::string("Error: CMatrix<T>::must be a square matrix");
    }
    if(m_type < ROBSOFT_32F){
        throw std::string("Error: CMatrix<T>::typename must be float or double");
    }

    if(method == ROBSOFT_DECOMP_GAUSS){
        return gaussInv();
    }
    if(method == ROBSOFT_DECOMP_LU){
        return luInv();
    }
}

template<typename T>
CMatrix<T> CMatrix<T>::gaussInv() const{
    CMatrix<T> dstMat;
    dstMat.eye(m_row, m_row);

    CMatrix<T> matrixCopy(*this);
    for(int i=0; i<m_row; i++){
        int j;
        for(j=i; j<=m_row; j++){
            if(j == m_row){
                throw std::string("Error: CMatrix<T>::singular matrix");
            }
            if(!num_is_zero(matrixCopy.at(j, i))){
                break;
            }
        }
        if(i != j){
            exchangeHorzontal(matrixCopy, i, j);
            exchangeHorzontal(dstMat, i, j);
        }

        T coe1 = 1.0 / matrixCopy.at(i, i);

        if(coe1 != 1){
            zoomHorzontal(matrixCopy, i, coe1);
            zoomHorzontal(dstMat, i, coe1);
        }

        for(j=i+1; j<m_row; j++){
            T coe2 = matrixCopy.at(j, i) / matrixCopy.at(i, i);
            coe2 = 0 - coe2;
            addHorzontal(matrixCopy, j, i, coe2);
            addHorzontal(dstMat, j, i, coe2);
        }
    }

    for(int i=m_row-2; i>=0; i--){
        for(int j=m_row-1; j>i; j--){
            T coe = matrixCopy.at(i, j) / matrixCopy.at(j, j);
            coe = 0 - coe;
            addHorzontal(matrixCopy, i, j, coe);
            addHorzontal(dstMat, i, j, coe);
        }
    }

    return dstMat;
}

template<typename T>
CMatrix<T> CMatrix<T>::luInv() const{
	return CMatrix<T>();
}

template<typename T>
void CMatrix<T>::exchangeHorzontal(CMatrix& matrix, int m, int n) const{
    if(m < 0 || n < 0 || m >= matrix.rows() || n >= matrix.rows()){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }

    T temp;
    for(int i=0; i<matrix.cols(); i++){
        temp = matrix.at(m, i);
        matrix.at(m, i) = matrix.at(n, i);
        matrix.at(n, i) = temp;
    }
}

template<typename T>
void CMatrix<T>::zoomHorzontal(CMatrix& matrix, int m, T coe) const{
    if(m < 0 || m >= matrix.rows()){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }

    for(int i=0; i<matrix.cols(); i++){
        matrix.at(m, i) = coe * matrix.at(m, i);
    }
}

template<typename T>
void CMatrix<T>::addHorzontal(CMatrix& matrix, int m, int n, T coe) const{
    if(m < 0 || n < 0 || m >= matrix.rows() || n >= matrix.rows()){
        throw std::string("Error: CMatrix<T>::overstep the boundary of the matrix");
    }

    for(int i=0; i<matrix.cols(); i++){
        matrix.at(m, i) = matrix.at(m, i) + coe * matrix.at(n, i);
    }
}

template<typename T>
T CMatrix<T>::det() const{
    if(m_dim != 1){
        throw std::string("Error: CMatrix<T>::must be 1-dim matrix");
    }
    if(m_row != m_col){
        throw std::string("Error: CMatrix<T>::must be a square matrix");
    }
    if(m_type < ROBSOFT_32F){
        throw std::string("Error: CMatrix<T>::typename must be float or double");
    }

    return determinant(m_row, m_array);
}

template<typename T>
T CMatrix<T>::determinant(int rank, T* array) const{
    T det=0;
    if(rank==1){
        return array[0];
    }
    else{
        for(int i=0; i<rank; i++){
            T* temp;
            temp = new T((rank-1)*(rank-1));

            int num = 0;
            for(int m=1; m<rank; m++)
                for(int n=0; n<rank; n++)
                    if(n != i)
                        temp[num++] = array[m * rank + n];
            det = det + array[i] * pow(-1,i) * determinant(rank-1, temp);
            delete[] temp;
        }
    }
    return det;
}

template<typename T>
void CMatrix<T>::print(const char* str) const{
    if(strlen(str) != 0){
        std::cout << std::string(str) << ": ";
        std::cout << "ROW = " << m_row << ", COL = " << m_col << ", DIM = " << m_dim << ", TYPE = ";
        switch(m_type){
        case 0:std::cout << "unsigned char" << std::endl;
            break;
        case 1:std::cout << "char" << std::endl;
            break;
        case 2:std::cout << "unsigned short" << std::endl;
            break;
        case 3:std::cout << "short" << std::endl;
            break;
        case 4:std::cout << "int" << std::endl;
            break;
        case 5:std::cout << "float" << std::endl;
            break;
        case 6:std::cout << "double" << std::endl;
            break;
        default:
            break;
        }
    }

    for(int k=0; k<m_dim; k++){
        for(int i=0; i<m_row; i++){
            for(int j=0; j<m_col; j++){
                std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(6)<< std::setw(15) << this->at(i, j, k) << "  ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

template<typename T>
void CMatrix<T>::indentifyType(){
    if(typeid(T).name() == typeid(unsigned char).name()){
        m_type =  ROBSOFT_8U;
    }
    else if(typeid(T).name() == typeid(char).name()){
        m_type =  ROBSOFT_8S;
    }
    else if(typeid(T).name() == typeid(unsigned short).name()){
        m_type =  ROBSOFT_16U;
    }
    else if(typeid(T).name() == typeid(short).name()){
        m_type =  ROBSOFT_16S;
    }
    else if(typeid(T).name() == typeid(int).name()){
        m_type =  ROBSOFT_32S;
    }
    else if(typeid(T).name() == typeid(float).name()){
        m_type =  ROBSOFT_32F;
    }
    else if(typeid(T).name() == typeid(double).name()){
        m_type =  ROBSOFT_64F;
    }
    else{
        throw std::string("Error: CMatrix<T>::invalid data type!");
    }
}

template<typename T>
int CMatrix<T>::arraySize(T* array) const{
    if(typeid(T).name() == typeid(unsigned char).name() || typeid(T).name() == typeid(char).name()){
        return sizeof(array)/sizeof(array[0])-1;
    }
    else{
        return sizeof(array)/sizeof(array[0]);
    }
}

}

#endif
