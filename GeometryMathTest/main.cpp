#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Vec3d = Eigen::Vector3d;
using namespace std;

/**
 * 1. 背景
    为了能够快速判断一个点是否在3维空间中的任意姿态的长方体中，虽然，对于平行坐标平面的长方体(例如平行于oxy平面）的判断方法非常简单，
    但是，想要判断一个点是否包含在3维空间中的任意姿态的长方体比较复杂。

2. 判断原理
    如下图所示，我们其实只需要判断一个点是否在3组平行平面的同一侧即可以判断(前/后面，左/右面，上/下面)，判断的方法比较简单：即利用如图所示中的与法线的夹角，
    当一个点(如图中点B)在左右面的同一侧时，与法线的夹角都是锐角或钝角；当一个点(如图中点A)在左右面的不同侧时，与法线的夹角必定一个为锐角一个为钝角；实际上，
    我们不需要判断角度，我们只需要判断余弦值的符号即可，即可以判断向量的内积的符号即可， 总结如下：
      判断点是否在两个平行面的同一侧 <=> 判断点与法线的夹角的大小 <=> 判断余弦值的符号 <=> 判断向量内积的符号
 * @return
 */

bool IsBox3D(Vec3d &px)
{
    /**
             .. code-block:: none

                             front z
                                    /
                                   /
                   p1(x0, y0, z1) + -----------  + p5(x1, y0, z1)
                                 /|            / |
                                / |           /  |
                p0(x0, y0, z0) + ---------p4 +   + p6(x1, y1, z1)
                               |  /      .   |  /
                               | / origin    | /
                p3(x0, y1, z0) + ----------- + -------> x right
                               |             p7(x1, y1, z0)
                               |
                               v
                        down y
     输入的点序列:p0:0,0,0, p1: 0,0,1,  p2: 0,1,1,  p3: 0,1,0,  p4: 1,0,0,  p5: 1,0,1,  p6: 1,1,1,  p7: 1,1,0;
     */

    Eigen::Matrix<double,8,3> corners8x3;
    corners8x3<< 0,0,0,  0,0,1,  0,1,1,   0,1,0,  1,0,0,  1,0,1,  1,1,1,  1,1,0;
    Eigen::Matrix<double,3,8> corners = corners8x3.transpose();

    //构建p0p3p4平面法向量,同时也是p1p2p5的法向量
    Vec3d p0p3 = corners.col(3) - corners.col(0);
    Vec3d p0p4 = corners.col(4) - corners.col(0);
    Vec3d p0p3p4_n = p0p3.cross(p0p4);//根据右手定则,法向量指向左边

    Vec3d pxp0 = corners.col(0) - px;
    Vec3d pxp1 = corners.col(1) - px;

    //向量内积公式: a.b = |a| |b| cos(theta),若a.b>0,则角度在0-90,若a.b<0,则角度在90-180度
    double direction_0 = p0p3p4_n.dot(pxp0);
    double direction_1 = p0p3p4_n.dot(pxp1);
    if((direction_0>0 && direction_1>0) ||(direction_0<0 && direction_1<0)){ //方向一致,表明不在box内
        return false;
    }

    //构建p0p1p3平面法向量,同时也是p4p5p7的法向量
    Vec3d p0p1 = corners.col(1) - corners.col(0);
    Vec3d p0p1p3_n = p0p1.cross(p0p3);
    Vec3d pxp4 = corners.col(4) - px;

    //向量内积公式: a.b = |a| |b| cos(theta),若a.b>0,则角度在0-90,若a.b<0,则角度在90-180度
    double direction_2 = p0p1p3_n.dot(pxp0);
    double direction_3 = p0p1p3_n.dot(pxp4);
    if((direction_2>0 && direction_3>0) ||(direction_2<0 && direction_3<0)){ //方向一致,表明不在box内
        return false;
    }

    Vec3d p0p1p4_n = p0p1.cross(p0p4);
    Vec3d pxp3 = corners.col(3) - px;
    double direction_4 = p0p1p4_n.dot(pxp0);
    double direction_5 = p0p1p4_n.dot(pxp3);
    if((direction_4>0 && direction_5>0) ||(direction_4<0 && direction_5<0)){ //方向一致,表明不在box内
        return false;
    }


    return true;
}




int main() {
    Vec3d point(0.5,0.9,0.1);

    cout<<IsBox3D(point)<<endl;


    return 0;
}
