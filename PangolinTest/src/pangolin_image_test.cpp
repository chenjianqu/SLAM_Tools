#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <iostream>

// 多视图图片显示
// pangolin中提供了SimpleMultiDisplay 例子用于演示多视图分割
// 我们首先创建在视窗中创建了三个视图，其中一个是我们很熟悉的相机视图，
// 在本例中我们特意让相机视图充满了整个视窗，以演示我们前面说明的这里的多视图其实是通过视图“叠加”实现的。
// 紧接着我们创建了另外两个视图用于显示图片，其中一个视图位于左上角，一个视图位于右下角

int main(int argc, char *argv[])
{
    std::cout << "OpenCV Version" << CV_VERSION << std::endl;
    pangolin::CreateWindowAndBind("MultiImage", 640, 480);// 创建视窗
    glEnable(GL_DEPTH_TEST);// 启动深度测试

    // 设置摄像机
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 320, 0.1, 1000),
            pangolin::ModelViewLookAt(-2, 0, -2, 0, 0, 0, pangolin::AxisY));

    // --------------- 创建三个视图 ---------------
    // SetHandler 是设置交互视图用的，是设置视图句柄
    pangolin::View &d_cam = pangolin::Display("cam")
            .SetBounds(0.0, 1.0, 0.0, 1.0, -752.0 / 480.0)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // 第五个参数，创建图片的是正值，创建三维图的是负值，这个参数实际上表征的是视图的 分辨率
    // 当该参数取正值时，pangolin会将由前四个参数设置的视图大小进行裁减，以满足所设置的分辨率
    // 当该参数取负值时，pangolin会将图片拉伸以充满由前四个参数设置的视图范围
    // 使用SetLock()函数设置了视图锁定的位置，该函数会在我们缩放整个视窗后，按照设定的lock选项自动锁定对齐位置
    // 将左上角的视图设置为left和top，右下角的视图设置为right和buttom锁定
    pangolin::View &cv_img_1 = pangolin::Display("image_1")
            .SetBounds(2.0 / 3.0f, 1.0f, 0.0f, 1 / 3.0f, 752.0 / 480.0f)
            .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::View &cv_img_2 = pangolin::Display("image_2")
            .SetBounds(0.f, 1 / 3.f, 2 / 3.f, 1.f, 752 / 480.f)
            .SetLock(pangolin::LockRight, pangolin::LockBottom);

    // 从文件读取图像
    cv::Mat img1 = cv::imread("/home/cjq/图片/1.jpg");
    cv::Mat img2 = cv::imread("/home/cjq/图片/2.png");

    // 创建glTexture容器用于读取图像
    // 需要创建两个图像纹理容器 pangolin::GlTexture 用于向上面创建的视图装载图像
    // 入口参数依次为：图像宽度，图像高度，pangolin的内部图像存储格式，是否开启现行采样，边界大小（像素），gl图像存储格式，gl数据存储格式
    // 因为是使用Opencv从文件中读取并存储图像，cv::Mat的图像存储顺序为BGR,而数据存储格式为uint型
    // 因此最后两个参数分别设置为 GL_BGR 和 GL_UNSIGNED_BYTE
    // 至于pangolin的内部存储格式，对图片的显示影响不大，因此一般设置为GL_RGB
    // 这边的图像的宽度和高度要设置为和原图像一致，否则会导致图像无法正常显示
    // 另外两个参数默认设置为 false和0
    pangolin::GlTexture imgTexture1(img1.cols, img2.rows, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    pangolin::GlTexture imgTexture2(img2.cols, img2.rows, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);

    while (!pangolin::ShouldQuit())
    {
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // 启动相机
        d_cam.Activate(s_cam);
        glColor3f(1.0f, 1.0f, 1.0f);
        pangolin::glDrawColouredCube();

        // 向GPU装载图像
        // 因为该对象只接受 uchar* 对象，所以需要传递 cv::Mat的data成员，而不能传递cv::Mat本身
        // 另外两个参数 则是在创建 pangolin::GlTexture 对象时使用的最后两个参数一致。
        imgTexture1.Upload(img1.data, GL_BGR, GL_UNSIGNED_BYTE);
        imgTexture2.Upload(img2.data, GL_BGR, GL_UNSIGNED_BYTE);

        // 显示图像
        // 依次激活视窗、设置默认背景色、最后渲染显示图像
        // 这里原始渲染出的图像是倒着的，因此我们反转了 Y 轴
        cv_img_1.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);         // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture1.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的

        cv_img_2.Activate();
        glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture2.RenderToViewportFlipY();

        pangolin::FinishFrame();
    }

    return 0;
}
