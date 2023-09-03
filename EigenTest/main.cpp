#include <iostream>

#include <Eigen/Dense>

using std::cout;
using std::endl;

using namespace Eigen;




template <typename EigenType>
using EigenContainer = std::vector< EigenType ,Eigen::aligned_allocator<EigenType>>;

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec7d = Eigen::Matrix<double, 7, 1>;
using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat24d = Eigen::Matrix<double, 2, 4>;
using Mat28d = Eigen::Matrix<double, 2, 8>;
using Mat34d = Eigen::Matrix<double, 3, 4>;
using Mat35d = Eigen::Matrix<double, 3, 5>;
using Mat36d = Eigen::Matrix<double, 3, 6>;
using Mat37d = Eigen::Matrix<double, 3, 7>;
using Mat38d = Eigen::Matrix<double, 3, 8>;
using Quatd = Eigen::Quaterniond;
using Eigen::Quaterniond;

using VecVector3d = EigenContainer<Eigen::Vector3d>;
using VecMatrix3d = EigenContainer<Eigen::Matrix3d>;




void Basic()
{
    std::cout << "Hello, World!" << std::endl;

    Eigen::Matrix3d m=Eigen::Matrix3d::Identity();

    Eigen::Quaterniond q(m);

    Eigen::Matrix3d m_inv=q.inverse().toRotationMatrix();

    cout<<m_inv<<endl;
}


/**
 * 获取最大最小值
 */
void MaxMinTest()
{
    MatrixXd mMat(4,4);
    mMat << 11, 10, 13, 15,
    3, 24, 56,	1,
    2, 12, 45,	0,
    8, 5,	6,	4;

    ///获取最大最小值,并获取其所在的索引
    MatrixXd::Index maxRow, maxCol;
    MatrixXd::Index minRow, minCol;
    double min = mMat.minCoeff(&minRow,&minCol);
    double max = mMat.maxCoeff(&maxRow,&maxCol);
    cout << "Max = " << max << endl;
    cout << "Min = " << min << endl;
    cout << "minRow = " << minRow << " minCol = " <<minCol<<endl;
    cout << "maxRow = " << maxRow << " maxCol = " << maxCol << endl;


    ///获取每行或每列的最大最小值
    Vector4d row_max;
    row_max =  mMat.rowwise().maxCoeff();
    cout<<row_max<<endl;

}

/**
 * 绝对值测试
 */
void AbsTest(){
    MatrixXd mMat(4,4);
    mMat << 11, -10, 13, 15,
    -3, 24, 56,	1,
    -2, 12, -45,	0,
    8, 5,	6,	4;

    auto abs_m = mMat.cwiseAbs();
    cout<<abs_m<<endl;

    auto abs_m2 = mMat.cwiseAbs2();//计算每个元素的平方
    cout<<abs_m2<<endl;
}


void NormTest(){
    Vector3d v{0,0,0};
    cout<<v.norm()<<endl;

    v=Vector3d(3,4,5);
    cout<<v.norm()<<endl;
    cout<<v.squaredNorm()<<endl;


    v=Vector3d(0,0,0.2);
    double theta = v.norm();//右乘矩阵取负号
    Vector3d a = v.normalized();
    cout<<"theta:"<<theta<<endl;
    cout<<"a:"<<a<<endl;

}

struct EigenClass{
    Eigen::Vector3d v{1,2,3};
};
void ClassTest(){
    EigenClass a;
    cout<<a.v<<endl;
}


double ComputeKittiBetaAngle(){
    //Car 0 1 2.028554 203.502164 180.473014 263.679422 205.794990 1.315632 1.513865 3.547118 -20.685186 1.743709 39.729248 1.551033
    //Car 0 0 2.063027 156.851198 181.790196 227.981275 212.460756 1.411824 1.480149 3.630527 -20.524265 1.863427 35.556181 1.542530
    double rotation_y = 1.551033;
    double rotation_y2 = 1.542530;

    Eigen::Vector3d P(-20.685186 ,0 ,39.729248);
    Eigen::Vector3d P2(-20.524265, 0, 35.556181);


    /// 计算alpha角度
    double alpha_1 = rotation_y;
    alpha_1 += atan2(P.z(),P.x()) + 1.5 * M_PI;
    while(alpha_1 > M_PI){
        alpha_1-=M_PI;
    }
    while(alpha_1 < -M_PI){
        alpha_1+=M_PI;
    }
    cout<<"alpha_1:"<<alpha_1<<endl;


    ///计算rotation_y
    Vector3d obj_z(-0.5,0,-0.5);

    Vector3d cam_x(1,0,0);

    double theta = atan2(obj_z.z(),obj_z.x()) - atan2(cam_x.z(),cam_x.x());

    cout<<theta<<endl;

    return alpha_1;
}


void BugTestForBoxEncloseStereoPointFactor(){


    Vec3d P_woj(36.08, 0.03, 22.42);
    Quatd Q_woj(1.,0,-0.09,-0.01);

    Vec3d pts_w(34.90 ,0.46 ,22.92);

    Vec3d dims(4.43, 1.63, 1.90);

    ///误差计算
    Vec3d pts_obj_j=Q_woj.inverse()*(pts_w-P_woj);//k点在j时刻的物体坐标
    Vec3d abs_v=pts_obj_j.cwiseAbs();
    Vec3d vec_err = abs_v - dims/2;
    vec_err *=10;

    std::array<double,3> residuals{};
    residuals[0]= std::max(0.,vec_err.x());
    residuals[1]= std::max(0.,vec_err.y());
    residuals[2]= std::max(0.,vec_err.z());

    cout<<"P_woj:"<<P_woj.transpose()<<endl;
    cout<<"Q_woj:"<<Q_woj.x()<<" "<<Q_woj.y()<<" "<<Q_woj.z()<<" "<<Q_woj.w()<<" "<<endl;
    cout<<"dims:"<<dims.transpose()<<endl;
    cout<<"pts_w:"<<pts_w.transpose()<<endl;
    cout<<"abs_v:"<<abs_v.transpose()<<endl;
    cout<<"vec_err:"<<vec_err.transpose()<<endl;



}



int main() {
    //MaxMinTest();
    //AbsTest();
    //NormTest();
    //ClassTest();

    //ComputeKittiBetaAngle();

    BugTestForBoxEncloseStereoPointFactor();


    return 0;
}
