#include <iostream>

#include <Eigen/Dense>

using std::cout;
using std::endl;

int main() {
    std::cout << "Hello, World!" << std::endl;

    Eigen::Matrix3d m=Eigen::Matrix3d::Identity();

    Eigen::Quaterniond q(m);

    Eigen::Matrix3d m_inv=q.inverse().toRotationMatrix();

    cout<<m_inv<<endl;

    return 0;
}
