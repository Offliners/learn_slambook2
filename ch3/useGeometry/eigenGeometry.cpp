#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

// 本程序演示了 Eigen 幾何模組的使用方法

int main(int argc, char** argv)
{
    // Eigen/Geometry 模組提供了各種旋轉和平移的表示
    // 3D 旋轉矩陣直接使用 Matrix3d 或 Matrix3f
    Matrix3d rotation_matrix = Matrix3d::Identity();
    // 旋轉向量使用 AngleAxis, 它底層不直接是Matrix，但運算可以當作矩陣（因為多載了運算符）
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     //沿 Z 軸旋轉 45 度
    cout.precision(3);
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;   //用matrix()轉換成矩陣
    // 也可以直接賦值
    rotation_matrix = rotation_vector.toRotationMatrix();
    // 用 AngleAxis 可以進行座標變換
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
    // 或者用旋轉矩陣
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

    // 歐拉角: 可以將旋轉矩陣直接轉換成歐拉角
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX順序，即yaw-pitch-roll順序
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // 歐式變換矩陣使用 Eigen::Isometry
    Isometry3d T = Isometry3d::Identity();                // 虽然稱為3d，實值上是4＊4的矩陣
    T.rotate(rotation_vector);                                     // 按照rotation_vector進行旋轉
    T.pretranslate(Vector3d(1, 3, 4));                     // 把平移向量設成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // 用變換矩陣進行座標變換
    Vector3d v_transformed = T * v;                              // 相當於R*v+t
    cout << "v tranformed = " << v_transformed.transpose() << endl;

    // 對於仿射和射影變換，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

    // 四元數
    // 可以直接把AngleAxis賦值给四元數，反之亦然
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion from rotation vector = " << q.coeffs().transpose() << endl;   // 請注意coeffs的順序是(x,y,z,w)，w為實部，前三者為虛部
    // 也可以把旋轉矩陣賦給它
    q = Quaterniond(rotation_matrix);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
    // 使用四元數旋轉一個向量，使用多载的乘法即可
    v_rotated = q * v; // 注意數學上是qvq^{-1}
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
    // 用常規向量乘法表示，則應該如下計算
    cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

    return 0;
}
