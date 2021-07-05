#include<iostream>
#include<ctime>

// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩陣的代數運算（逆，特徵值等）
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 50

/****************************
* 本程序演示了 Eigen 基本類型的使用
****************************/

int main(int argc, char **argv)
{
    // Eigen 中所有向量和矩陣都是Eigen::Matrix，它是一個模板類。它的前三個參數為：數據類型，行，列
    // 聲明一個2*3的float矩陣
    Matrix<float, 2, 3> matrix_23;

    // 同時，Eigen 通過 typedef 提供了許多內置類型，不過底層仍是Eigen::Matrix
    // 例如 Vector3d 實質上是 Eigen::Matrix<double, 3, 1>，及三維向量
    Vector3d v_3d;
    // 這與上者相同
    Matrix<float, 3, 1> vd_3d;

    // Matrix3d 實質上是 Eigen::Matrix<double, 3, 3>
    Matrix3d matrix_33 = Matrix3d::Zero(); //初始化為零
    // 如果不確定矩陣大小，可以使用動態大小的矩陣
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    // 更簡單的
    MatrixXd matrix_x;
    // 這種類型還有很多，我们不一一列舉

    // 下面是對Eigen陣的操作
    // 輸入數據（初始化）
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // 输出
    cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

    // 用()訪問矩陣中的元素
    cout << "print matrix 2x3: " << endl;
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 3; ++j)
            cout << matrix_23(i, j) << "\t";
        cout << endl;
    }

    // 矩阵和向量相乘（实际上仍是矩阵和矩阵）
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    // 但是在Eigen裡不能混合兩種不同類型的矩陣，像這樣是錯的
    // Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 應該顯式轉型
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << endl;

    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;

    // 同同樣你不能搞錯矩陣的維度
    // 試著取消下面的註解，看看Eigen會報什麼錯
    // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    // 一些矩陣運算
    // 四則運算就不演示了，直接用+-*/即可。
    matrix_33 = Matrix3d::Random();      // 隨機數矩陣
    cout << "random matrix: \n" << matrix_33 << endl;
    cout << "transpose: \n" << matrix_33.transpose() << endl;      // 轉置
    cout << "sum: " << matrix_33.sum() << endl;                    // 各元素和
    cout << "trace: " << matrix_33.trace() << endl;                // 跡
    cout << "times 10: \n" << 10 * matrix_33 << endl;              // 數乘
    cout << "inverse: \n" << matrix_33.inverse() << endl;          // 逆
    cout << "det: " << matrix_33.determinant() << endl;            // 行列式

    // 特徵值
    // 實對稱矩陣可以保證對角化成功
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

    // 解方乘
    // 我們求解 matrix_NN * x = v_Nd 這個方程
    // N的大小在前边的宏里定义，它由随机数生成
    // 直接求逆自然是最直接的，但是求逆運算量大

    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();  // 保證半正定
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock(); // 計時
    // 直接求逆
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    // 通常用矩陣分解來求，例如QR分解，速度會快很多
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time of Qr decomposition is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    // 對於正定矩陣，還可以用cholesky分解來解方程
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of ldlt decomposition is "
        << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    return 0;
}
