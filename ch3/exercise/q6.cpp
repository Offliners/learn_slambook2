#include<iostream>
#include<ctime>
#include<Eigen/Core>
#include<Eigen/Dense>

#define MATRIX_SIZE 50

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    // Method 1 : normal inverse
    clock_t time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x1 = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;

    // Method 2 :  QR decomposition
    time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x2 = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time of QR decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;

    // Method 3 : Ldlt decomposition (Cholesky)
    time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x3 = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;

    // Method 4 : Least Squares
    time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x4 = (matrix_NN.transpose() * matrix_NN).inverse() * (matrix_NN.transpose() * v_Nd);
    cout << "time of Least Squares is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;

    // Method 5 : LU decomposition
    time_stt = clock();
    Matrix<double, MATRIX_SIZE, 1> x5 = matrix_NN.lu().solve(v_Nd);
    cout << "time of LU decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;

    // Method 6 : Jacobi Iteration

    if(x1.isApprox(x2, .6f) && x1.isApprox(x3, .6f) && x1.isApprox(x4, .6f) && x1.isApprox(x5, .6f))
        cout << "Pass" << endl;
    else
        cout << "Error" << endl;

    return 0;
}
