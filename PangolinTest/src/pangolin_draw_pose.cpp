#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdio.h>
#include <unistd.h>

using namespace std;


// 法一  法二 说明:
//  - 法一是通过设置旋转矩阵一步一步求解
//  - 法二是直接都转换为欧氏变换矩阵 进行处理
// 相比较而言，转换为欧氏变换矩阵之后好处理一些

int Demo1(){

    FILE *fp_gt;
    // EuRoc数据集
    // 数据集的解读：https://blog.csdn.net/shyjhyp11/article/details/115334614
    /* groundtruth 输出格式
        - p  代表position，指的是MAV的空间3D坐标
        - RS 代表这个坐标是在R坐标系的值，也就是LEICA位姿跟踪坐标系下测到的值
        - S  指的是从Sensor坐标系下得到的，后来又变换到R坐标系
        - R  可能代表LEICA坐标系，
        - x代表3D位置的x轴方向上的真值，单位为 m
            - p_RS_R_x [m]
            - p_RS_R_y [m]
            - p_RS_R_z [m]


        - q  代表 quaternion 四元数，表达了 MAV 的朝向信息，
        - RS 代表这个坐标是在R坐标系的值，也就是LEICA位姿跟踪坐标系下测到的值
        - w  四元数的实部
        - xyz 四元数的虚部
            - q_RS_w []
            - q_RS_x []
            - q_RS_y []
            - q_RS_z []


        - v  表示这是MAV的速度信息，而且是在R坐标系下的速度信息，单位 m/s
            - v_RS_R_x  [m s^-1]
            - v_RS_R_y  [m s^-1]
            - v_RS_R_z  [m s^-1]


        - w  表示这是 MAV 在R坐标系下的角速度信息，单位 rad/s
            - b_w_RS_S_x [rad s^-1]
            - b_w_RS_S_y [rad s^-1]
            - b_w_RS_S_z [rad s^-1]


        - a  表示这是 MAV 在R坐标系下的线加速度信息，单位 m/s^2
            - b_a_RS_S_x [m s^-2]
            - b_a_RS_S_y [m s^-2]
            - b_a_RS_S_z [m s^-2]
    */
    fp_gt = fopen("/home/bck20/DataSet/EuRoc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv", "r");
    if (fp_gt == nullptr)
    {
        cout << "failed to open file !\n";
        return -1;
    }

    // 跳过第一行
    char fl_buf[1024];
    fgets(fl_buf, sizeof(fl_buf), fp_gt);
    // 创建数据寄存器
    ulong time_stamp(0);
    double px(0.0f), py(0.0f), pz(0.0f);                                     // position 3D坐标
    double qw(0.0f), qx(0.0f), qy(0.0f), qz(0.0f);                           // 四元数
    double vx(0.0f), vy(0.0f), vz(0.0f);                                     // 速度
    double bwx(0.0f), bwy(0.0f), bwz(0.0f), bax(0.0f), bay(0.0f), baz(0.0f); // 角加速度，线加速度

    // 法一:
    vector<Eigen::Vector3d> traj;

    // 法二:
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

    // 初始化视窗
    pangolin::CreateWindowAndBind("camera_pose", 752 * 2, 480 * 2);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(752 * 2, 480 * 2, 420, 420, 320, 240, 0.1, 1000),
            pangolin::ModelViewLookAt(5, -3, 5, 0, 0, 0, pangolin::AxisZ));

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0f, 1.0f, 0.0f, 1.0f, -752 / 480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (!feof(fp_gt))
    {
        // ============== 常规操作 ===============
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        // ======================================

        fscanf(fp_gt, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
               &time_stamp, &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &bwx, &bwy, &bwz,
               &bax, &bay, &baz);

        // 法一:
        Eigen::Quaterniond quat(qw, qx, qy, qz);
        Eigen::Vector3d pos(px, py, pz);
        traj.push_back(pos);

        // 法二:
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(px, py, pz));
        poses.push_back(Twr);

        // ==============  绘制坐标系  =============
        glLineWidth(3);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        glColor3f(0.f, 1.0f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();

        // -------- 绘制随位姿变化的相机模型 -------- //
        // 构建位姿变换矩阵，pangolin中为列主序
        Eigen::Matrix3d R = quat.toRotationMatrix();

        /*  形式如下:  第一个 R =
            -0.382623   0.340983   -0.85868
            0.165471   0.939666    0.29941
            0.908967 -0.0275257  -0.415961

            相当于是 Twr.rotation()
        */

        // glPushMatrix和glPopMatrix的作用： https://blog.csdn.net/passtome/article/details/7768379
        // glPushMatrix, glPopMatrix 操作其实就相当于栈里的入栈和出栈

        // 首先需要使用 glPushMatrix() 告诉pangolin我们需要使用一个矩阵,
        // 然后我们使用 glMulmatrixd() 告诉pangolin后续绘制中的所有坐标均需要乘以这个矩阵,
        // 最后再glPopMatrix() 弹出矩阵,便于下一次循环填入新的矩阵数值,
        // 不同于Eigen等矩阵库, pangolin里的矩阵是按照列主序存储的
        // - 行主序：在数组中按照a[0][0]、a[0][1]、a[0][2]…a[1][0]、a[1][1]、a[1][2]…依次存储数据
        // - 列主序：在数组中按照a[0][0]、a[1][0]、a[2][0]…a[0][1]、a[1][1]、a[2][1]…依次存储数据
        glPushMatrix();

        // 这个是vector容器,是按照每列每列排的
        std::vector<GLdouble> Twc = {R(0, 0), R(1, 0), R(2, 0), 0.,
                                     R(0, 1), R(1, 1), R(2, 1), 0.,
                                     R(0, 2), R(1, 2), R(2, 2), 0.,
                                     pos.x(), pos.y(), pos.z(), 1.};
        // 让相机模型动起来，最简单的想法是在每次获取相机的位姿后，对上述八点线段的坐标进行相应的变换，
        // 进而绘制出当前时刻的相机模型,但是如果每次都需要去计算变换后的位姿,这无疑是非常麻烦且容易出错的.
        // opengl 提供了 glMultMatrix() 函数自动处理图像点的位姿转换

        // 法一:
        glMultMatrixd(Twc.data());

        // 法二:
        // glMultMatrixd(Twr.data());

        // 绘制相机轮廓线
        const float w = 0.2;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();

        // -------- 绘制相机轨迹 --------//
        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(0.f, 1.f, 0.f);

        // 法一:
        for (size_t i = 0; i < traj.size() - 1; i++)
        {
            glVertex3d(traj[i].x(), traj[i].y(), traj[i].z());
            glVertex3d(traj[i + 1].x(), traj[i + 1].y(), traj[i + 1].z());
        }

        // 法二:
        // for (size_t i = 0; i < poses.size() - 1; i++)
        // {
        //     glVertex3d(poses[i].translation()[0], poses[i].translation()[1], poses[i].translation()[2]);
        //     glVertex3d(poses[i + 1].translation()[0], poses[i + 1].translation()[1], poses[i + 1].translation()[2]);
        // }

        glEnd();

        pangolin::FinishFrame();

        if (pangolin::ShouldQuit())
            break;
    }

    return 0;
}







int Demo2(){
    string trajectory_file = "/home/cjq/CLionProjects/PangolinTest/data/vio.txt";

    ifstream fin(trajectory_file);
    if (!fin)
    {
        cout << "cannot find trajectory file at " << trajectory_file << endl;
        return -1;
    }

    // 初始化视窗
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    double time, tx, ty, tz, qx, qy, qz, qw;
    vector<Eigen::Vector3d> traj;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

    int i = 0;
    while (!fin.eof())
    {
        // ============== 常规操作 ===============
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);

        // 绘制坐标系
        glLineWidth(3);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0.3, 0, 0);
        glColor3f(0.f, 1.0f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0.3, 0);
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 0.3);
        glEnd();

        // ------------------- 绘制随位姿变换的相机模型 ----------
        glPushMatrix();

        glMultMatrixd(Twr.data());

        // 绘制相机轮廓线
        const float w = 0.2;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();

        // -------- 绘制相机轨迹 --------//
        glLineWidth(2);
        glBegin(GL_LINES);

        glColor3f(1.0f, 0.0f, 0.0f);
        for (size_t i = 0; i < poses.size() - 1; i++)
        {
            auto p1 = poses[i], p2 = poses[i + 1];
            /* 这边说明一下:
              p1 是 4x4的齐次矩阵,比如这个是  poses[2].matrix() 打印出来的
                0.997863 -0.00369598   0.0652398    0.013979
                0.00932699    0.996232  -0.0862205  -0.0130823
                -0.0646754   0.0866447    0.994138  -0.0108696
                0           0           0           1

                而下面用到的poses[i].translation() 表示从4x4的欧式变换矩阵中提取出 平移向量
                同理,poses[i].rotation() 表示从4x4的欧式变换矩阵中提取出  旋转向量

            */
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        }

        if (i++ == 2)
        {
            std::cout << poses[2].matrix() << std::endl;
            std::cout << std::endl
                      << poses[2].translation().transpose() << std::endl;
            std::cout << std::endl
                      << poses[2].rotation() << std::endl;
        }

        glEnd();

        pangolin::FinishFrame();

        if (pangolin::ShouldQuit())
            break;

        usleep(30000);
    }


    return 0;
}




int main(int argc, char *argv[])
{
    //Demo1();

    Demo2();

    return 0;
}
