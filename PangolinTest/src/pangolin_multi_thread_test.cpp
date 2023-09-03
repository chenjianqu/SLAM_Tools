// https://blog.csdn.net/weixin_43991178/article/details/105119610
// Task2 pangolin与多线程

#include <pangolin/pangolin.h>
#include <thread>

static const std::string window_name = "HelloPangolinThreads";


// 创建一个视窗用于后续的显示，这个视窗是在主线程中创建的，因此在主线程调用后，需要使用GetBoundWindow()->RemoveCurrent()将其解绑
void setup()
{
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    // enabel depth
    glEnable(GL_DEPTH_TEST);

    // 从主线程取消设置当前上下文
    // GetBoundWindow()  返回指向当前pangolin window 上下文的指针，如果没有绑定则返回nullptr
    pangolin::GetBoundWindow()->RemoveCurrent();
}

// 新开一个线程，运行run()函数，在run函数中首先将之前解绑的视窗绑定到当前线程，
// 随后需要重新设置视窗的属性(启动深度测试),同样,在线程结束时,需要解绑视窗
void run()
{
    // 获取上下文并将它绑定到这个线程
    pangolin::BindToContext(window_name);

    // 我们需要手动恢复上下文的属性
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);

    // 定义投影和初始模型视图矩阵
    // 创建观察相机
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

    // 创建交互视图
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
            .SetHandler(&handler);

    while(!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render info
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    // 解绑视窗
    pangolin::GetBoundWindow()->RemoveCurrent();
}

int main(int argc, char *argv[])
{
    // create window and context in the main thread
    setup();

    // use the coontext in a separate rendering thread
    std::thread render_loop;
    render_loop = std::thread(run);
    render_loop.join();


    return 0;
}

