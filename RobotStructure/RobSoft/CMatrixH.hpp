#ifndef CMATRIXH_HPP
#define CMATRIXH_HPP

#include <vector>

namespace robsoft{

enum DATATYPE{ ROBSOFT_8U,  ROBSOFT_8S,  ROBSOFT_16U,  ROBSOFT_16S,  ROBSOFT_32S,  ROBSOFT_32F,  ROBSOFT_64F};
enum DECOMPMETHOD{ ROBSOFT_DECOMP_GAUSS,  ROBSOFT_DECOMP_LU,  ROBSOFT_DECOMP_CHOLESKY,  ROBSOFT_DECOMP_SVD};

template<typename T>
class CMatrix{
public:
    CMatrix();
    CMatrix(int row, int col);
    CMatrix(int row, int col, int dim);
    CMatrix(int row, int col, T* array);
    CMatrix(int row, int col, int dim, T* array);
    CMatrix(int row, int col, std::vector<T> array);
    CMatrix(int row, int col, int dim, std::vector<T> array);
    CMatrix(const CMatrix& matrix);
    ~CMatrix();

    // 设置矩阵的值
    void setValue(int row, int col);
    void setValue(int row, int col, int dim);
    void setValue(int row, int col, T* array);
    void setValue(int row, int col, int dim, T* array);
    void setValue(int row, int col, std::vector<T> array);
    void setValue(int row, int col, int dim, std::vector<T> array);

    // 零矩阵
    void zeros();
    void zeros(int row, int col);
    void zeros(int row, int col, int dim);

    // 单位矩阵
    void eye();
    void eye(int row, int col);
    void eye(int row, int col, int dim);

    // 拼接
    void appendCol(const CMatrix& matrix);
    void appendRow(const CMatrix& matrix);

    int getElementNumber() const;

    T& at(int i, int j);   // 可修改返回元素
    T& at(int i, int j, int k);    // 可修改返回元素
    const T& at(int i, int j) const;   // 返回元素
    const T& at(int i, int j, int k) const;    // 返回元素
    void replace(int i, int j, T value); // 替换元素
    void replace(int i, int j, int k, T value);  // 替换元素
    int rows() const;   // 返回矩阵的行数
    int cols() const;   // 返回矩阵的列数
    int dims() const;   // 返回矩阵的维度
    DATATYPE types() const; // 返回矩阵元素数据类型
    CMatrix<T> clone() const;   // 克隆矩阵

    CMatrix<T> operator *(double value) const;  // 乘常数
    CMatrix<T> mul(double value) const;    // 乘常数
    CMatrix<T> operator *(const CMatrix& matrix) const;
    CMatrix<T> operator +(const CMatrix& matrix) const;
    CMatrix<T> operator -(const CMatrix& matrix) const;
    void operator =(const CMatrix& matrix);
    bool operator ==(const CMatrix& matrix) const;
    bool operator !=(const CMatrix& matrix) const;
    CMatrix<T> mul(const CMatrix& matrix) const;    // 乘法
    CMatrix<T> add(const CMatrix& matrix) const;    // 加法
    CMatrix<T> sub(const CMatrix& matrix) const;    // 减法
    T dot(const CMatrix& matrix) const; // 点积
    CMatrix<T> cross(const CMatrix& matrix) const;  // 叉乘
    CMatrix<T> trans() const;   // 转置
    CMatrix<T> inv(DECOMPMETHOD method =  ROBSOFT_DECOMP_GAUSS) const;  // 求逆
    T det() const;  // 行列式的值
    T norm() const; // 二范数
    CMatrix<T> normalization() const;   // 归一化

    void print(const char* str = "CMatrix") const;

private:
    T* m_array;
    int m_row;
    int m_col;
    int m_dim;
    enum DATATYPE m_type;

    void indentifyType();   // 识别矩阵元素的数据类型
    int arraySize(T* array) const;  // 返回数组的元素个数
    T determinant(int rank, T* array) const;    // 递归求行列式的值
    void exchangeHorzontal(CMatrix& matrix, int m, int n) const;    // 交换行
    void zoomHorzontal(CMatrix& matrix, int m, T coe) const;    // m*coe
    void addHorzontal(CMatrix& matrix, int m, int n, T coe) const;  // m + n*coe
    CMatrix<T> gaussInv() const;    // 高斯消元法求逆
    CMatrix<T> luInv() const;   // lu分解求逆
};

}

#endif
