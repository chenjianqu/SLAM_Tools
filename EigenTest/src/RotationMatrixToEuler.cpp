#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

bool isRotationMatirx(Eigen::Matrix3d R);
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

const double ARC_TO_DEG = 57.29577951308238;
const double DEG_TO_ARC = 0.0174532925199433;

int main()
{
    // 使用Eigen库将欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix1=Eigen::Matrix3d::Identity();

    // 使用Eigen将旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle1 = rotation_matrix1.eulerAngles(2,1,0); // ZYX顺序，yaw,pitch,roll
    cout << "roll_1 pitch_1 yaw_1 = " << eulerAngle1[2] << " " << eulerAngle1[1]
         << " " << eulerAngle1[0] << endl << endl;

    // 使用自定义函数将旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle2 = rotationMatrixToEulerAngles(rotation_matrix1); // roll,pitch,yaw
    cout << "roll_2 pitch_2 yaw_2 = " << eulerAngle2[0] << " " << eulerAngle2[1]
         << " " << eulerAngle2[2] << endl << endl;

    return 0;
}


bool isRotationMatirx(Eigen::Matrix3d R)
{
    double err=1e-6;
    Eigen::Matrix3d shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
    return (shouldIdenity - I).norm() < err;
}

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    assert(isRotationMatirx(R));
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}