#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h>

int main( int /*argc*/, char** /*argv*/ )
{
    // 创建名称为“Main”的GUI窗口，尺寸为 640*480
    // 通过 CreateWindowAndBind 命令创建一个视图对象，函数的入口的参数依次为视图的名称、宽度、高度
    // 该命令类似于opencv的namewindow，即创建一个用于显示的窗体
    pangolin::CreateWindowAndBind("Main",640,480);
    // 启动深度测试功能
    // 该功能会使得pangolin只会绘制朝向镜头的那一面像素点，避免容易混淆的透视关系出现，
    // 因此在任何3D透视可视化中都应该开启该功能
    glEnable(GL_DEPTH_TEST);


    // 创建一个观察相机视图
    // 放置一个观察的假想相机（也就是我们从这个相机可以看到的画面），
    // ProjectionMatrix表示假想相机的内参，在我们对视图进行交互操作时，Pangolin会自动根据内参矩阵完成对应的透视变换
    // 另外还需要给出相机初始时刻所处的位置，以及相机视点的位置（相机的光轴朝向哪一个点）以及相机的本身哪一轴朝上
    // ProjectMatrix(int h, int w, int fu, int fv, int cu, int cv,int znear, int zfar)
    // 参数依次为观察相机的图像高度、宽度、4个内参以及最近和最远视距
    // ModelViewLookAt(double x, double y, double z, double lx, double ly, double lz, AxisDirection Up)
    // 参数依次为相机所在的位置，以及相机所看的视点位置（一般会设置在原点）
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // 创建交互视图
    // 用于显示上一步相机所“拍摄”到的内容，setBounds()函数前四个参数依次表示视图在视窗中的范围（下、上、左、右）
    // 可以采用相对坐标 以及 绝对坐标（使用 Attach 对象）
    pangolin::Handler3D handler(s_cam);// 交互相机视图句柄
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // 激活之前设定好的视窗对象（否则视窗内会保留上一帧的图形）
        d_cam.Activate(s_cam);

        // 在原点绘制一个立方体
        pangolin::glDrawColouredCube();


        // 绘制坐标系
        glLineWidth(3);
        glBegin(GL_LINES);
        // X 轴 red
        glColor3f(0.8f, 0.0f, 0.0f); // 颜色，红色
        glVertex3f(-1, -1, -1);      // 原点 那个顶点
        glVertex3f(0, -1, -1);       // 往x方向伸出 一个单位的顶点
        // Y 轴 green
        glColor3f(0.0f, 0.8f, 0.0f);
        glVertex3f(-1, -1, -1);
        glVertex3f(-1, 0, -1);
        // Z 轴 blue
        glColor3f(0.0f, 0.0f, 0.8f);
        glVertex3f(-1, -1, -1);
        glVertex3f(-1, -1, 0);
        glEnd();

         // 运行帧循环以推进窗口事件
        // 使用 FinishFrame命令刷新视窗
        pangolin::FinishFrame();
    }

    return 0;
}