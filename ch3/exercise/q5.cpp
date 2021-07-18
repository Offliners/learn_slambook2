#include<iostream>
#include<Eigen/Core>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    MatrixXd mat1 = MatrixXd::Random(10,10);
    cout << "Mat 1 Before : " << endl;
    cout << mat1 << endl;

    mat1.block(0, 0, 3, 3) = Matrix3d::Identity();

    cout << "Mat 1 After : " << endl;
    cout << mat1 << endl;

    return 0;
}
