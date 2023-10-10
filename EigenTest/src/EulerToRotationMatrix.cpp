#include <iostream>
#include<iomanip>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);

const double ARC_TO_DEG = 57.29577951308238;
const double DEG_TO_ARC = 0.0174532925199433;

void StdLowerboundTest(){
    vector<int> matchesAll={1,2,3,4,6,7,8,10};
    auto del_start = std::lower_bound(matchesAll.begin(),matchesAll.end(),7,
                                      [](const auto& a,const auto& b){
                                          return a < b; //a是matchesAll的元素，b是比较值
                                      });
    matchesAll.erase(del_start,matchesAll.end());
    for(int e:matchesAll)
        cout<<e<<" ";
    cout<<endl;
}

int main()
{
    StdLowerboundTest();

    // 设定车体欧拉角(角度)，绕固定轴
    auto [
            roll_deg,// 绕X轴
            pitch_deg,// 绕Y轴
            yaw_deg// 绕Z轴
          ]
          = tuple<double,double,double> {-90.1091, -0.105778, 0.754896};

    // 转化为弧度
    double roll_arc = roll_deg * DEG_TO_ARC;    // 绕X轴
    double pitch_arc = pitch_deg * DEG_TO_ARC;  // 绕Y轴
    double yaw_arc = yaw_deg * DEG_TO_ARC;      // 绕Z轴

    // 初始化欧拉角（rpy）,对应绕x轴，绕y轴，绕z轴的旋转角度
    Eigen::Vector3d euler_angle(roll_arc, pitch_arc, yaw_arc);

    // 使用Eigen库将欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
    rotation_matrix1 = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());

    cout<<std::fixed<<std::setprecision(12);
    cout << "\nrotation matrix1 =\n" << rotation_matrix1 << endl << endl;

    // 使用自定义函数将欧拉角转换为旋转矩阵
    rotation_matrix2 = eulerAnglesToRotationMatrix(euler_angle);
    cout << "rotation matrix2 =\n" << rotation_matrix2 << endl << endl;

    return 0;
}

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

