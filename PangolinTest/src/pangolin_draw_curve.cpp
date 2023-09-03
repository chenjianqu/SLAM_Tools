// Task5 pangolin 绘制数据曲线

#include <iostream>
#include <pangolin/pangolin.h>

int main(int argc, char *argv[])
{

    // create OpenGL window in single line
    pangolin::CreateWindowAndBind("Main", 640, 480);

    // Data logger object 数据记录器对象
    // 待可视化的数据全部存储在 pangolin::DataLog 对象中，所以我们首先创建一个pangolin::DataLog对象
    // 并使用对应的成员函数 SetLabels()设置对应数据的名称  即图例
    pangolin::DataLog log;

    // Optionally add named labels 可选择添加命名标签
    std::vector<std::string> labels;
    labels.push_back(std::string("sin(t)"));
    labels.push_back(std::string("cos(t)"));
    labels.push_back(std::string("sin(t) + cos(t)"));
    labels.push_back(std::string("cos(2t)"));
    labels.push_back(std::string("tan(t)"));
    log.SetLabels(labels);

    const float tinc = 0.02f;

    // OpenGL 'view' of data, We might have many views of the same data
    // 数据的OpenGL“视图”，我们可能有许多相同数据的视图
    // 数据的可视化通过 pangolin::Plotter 对象来实现
    // 该对象的构造参数的第一个参数为需要绘制的 pangolin::DataLog 对象
    // 随后4个参数依次 Plotter 的左边界、右边界、下边界、上边界(即 Plotter中 X轴 Y轴的范围)
    // 最后两个参数依次为 x轴，y轴的坐标轴刻度大小
    pangolin::Plotter plotter(&log, 0.0f, 4.0f * (float)M_PI / tinc,
                              -4.0f, 4.0f, (float)M_PI / (4.0f * tinc), 0.5f);
    plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
    plotter.Track("$i"); // 坐标轴自动滚动

    // Add some sample annotations to the plot（为区域着色）
    // 使用 plotter 的成员函数 AddMarker 添加一些标志块的功能
    // 函数的入口参数依次为 标志块的方向，标志块的数值， 标志块的判别方式以及标志块的颜色

    // eg. 第一个Marker  标志块的方向为垂直方向  数值为50pi  判断方式为小于  颜色为带透明度的蓝色
    plotter.AddMarker(pangolin::Marker::Vertical, 50 * M_PI, pangolin::Marker::LessThan,
                      pangolin::Colour::Blue().WithAlpha(0.2f));
    // eg. 第二个Marker  将y轴大于3的区域标记为红色
    plotter.AddMarker(pangolin::Marker::Horizontal, 3, pangolin::Marker::GreaterThan,
                      pangolin::Colour::Red().WithAlpha(0.2f));
    // eg. 第三个Marker  由于是等于，因此只将 y = 3 这一条线标记为绿色
    plotter.AddMarker(pangolin::Marker::Horizontal, 3, pangolin::Marker::Equal,
                      pangolin::Colour::Green().WithAlpha(0.2f));

    // 将构建好的plotter添加到Display中
    pangolin::DisplayBase().AddDisplay(plotter);

    float t = 0;

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 帧循环中，只需要使用 DataLog::Log() 函数不断更新 DataLog 中的数据
        // pangolin就会根据之前创建的 plotter 自动在视窗中绘制数据
        log.Log(sin(t), cos(t), sin(t) + cos(t), cos(2*t), tan(t));
        t += tinc;

        pangolin::FinishFrame();
    }

    return 0;
}

