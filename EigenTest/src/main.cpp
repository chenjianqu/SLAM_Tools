#include <iostream>
#include <filesystem>
#include <string>
#include <Eigen/Dense>

using std::cout;
using std::endl;

int main() {
    std::cout << "Hello, World!" << std::endl;

    std::filesystem::path test_path("AA");
    cout<<test_path.append("BB")<<endl;
    test_path += "CC";
    cout<<test_path<<endl;

    Eigen::Matrix3d m=Eigen::Matrix3d::Identity();

    Eigen::Quaterniond q(m);

    Eigen::Matrix3d m_inv=q.inverse().toRotationMatrix();

    cout<<m_inv<<endl;

    Eigen::Matrix4d T;
    T<<0.999911501369,  0.001871097035, -0.013171500546, 0.0424539,
            0.013175017070, -0.001879664431,  0.999911438972, 1.19722,
            0.001846173327, -0.999996482923, -0.001904149818,-0.528046,
            0, 0, 0, 1;

    cout<<T.inverse()<<endl;



    return 0;
}
