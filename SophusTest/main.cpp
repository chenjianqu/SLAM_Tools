#include <iostream>

#include <Eigen/Dense>
#include <sophus/so3.hpp>

using namespace std;

void HatTest()
{
    Eigen::Vector3d vec(1,2,3);
    Eigen::Matrix3d hat_v = Sophus::SO3d::hat(vec);
    cout<<hat_v<<endl;
}



int main() {
    HatTest();
    return 0;
}
