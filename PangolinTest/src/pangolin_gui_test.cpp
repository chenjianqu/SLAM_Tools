#include <pangolin/pangolin.h>
#include <string>
#include <iostream>
// ------------------------------------- //

void SampleMethod()
{
    std::cout << "You typed ctrl-r or pushed reset " << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout << " ======== Task3 ========" << std::endl;

    // 创建视窗
    pangolin::CreateWindowAndBind("Main", 640, 480);
    glEnable(GL_DEPTH_TEST);// 启动深度测试
    // 创建一个相机
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
            pangolin::ModelViewLookAt(-0, 0.5, -3, 0, 0, 0, pangolin::AxisY));

    // 分割视窗
    const int UI_WIDTH = 150;// 这边的150也就是宽度的意思，和上面创建界面 宽度为 640是一个概念

    // 右侧用于显示视窗
    // 这边的 pangolin::Attach::Pix(UI_WIDTH) 也就是计算UI_WIDTH占总宽度的多少
    // 因为从 task1 知道 SetBounds()函数前四个入口参数是 下 上 左 右，也就是从下到上占多少，从左到右占多少，这个范围是[0,1]
    // 所以这边通过 pangolin::Attach::Pix(UI_WIDTH) 计算一下实际的比例是多少
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH),
                       1.0, -640.0f / 480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // 左侧用于创建控制面板
    // 在坐标180像素宽度位置使用CreatePanel()命令创建一个面板,并给这个面板命名为"ui",这里"ui"是面板的tag名称
    // 后续所有控制的操作都通过这个tag绑定到对应的面板上
    // 视窗的其余部分则为用于显示 viewport
    pangolin::CreatePanel("ui")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    // 创建控制面板的控件对象，pangolin
    // 创建一系列控件
    // 将所有"控件"视为一个 pangolin::var 对象,该对象是一个模板类,我们可以向其中传递自定义的类型模板
    // 常用的 pangolin::Var 对象整理如下:
    /*
        pangolin::Var<bool> : bool型 Var对象,创建参数依次为控件的tag(名字),初始值,以及是否可以toggle.
        - 当 toggle 设置为 true 时,该对象表示一个选框(Checkbox);设置为 false 时则表示一个按钮(Button)
        - 初始值对于设置为 true 或 false 会影响选框是否被选中,对于按钮来说没有影响,但是习惯性一般都会设置为false,
        - 控件的tag是唯一的,命名格式为 panel_tag.controller_tag,
          例如,我们所有控件需要板顶的面板为"ui",因此所有的控件tag都命名为 ui.xxx 的形式

        pangolin::Var<int/double/float> : 这一类 Var 对象为常见的滑条对象,创建参数依次为 tag, 初始值, 最小值, 最大值 和 logsacle
        - logsacle 表示是否以对数坐标形式显示,
        - 最大最小值控制滑动条的范围,如果不设置的话默认最小值为0,最大值为1
        - 初始值是滑条上初始显示的数字,因此其不需要在滑条的范围内,只不过在用户移动滑条后,显示的数字会更新为滑条当前位置对应的数字

        pangolin::Var<std::function<void()>> 这一类控件同样实现按钮控件的功能,只是其在创建时传入一个std::function函数对象,
        因此不需要在后续的循环中进行回调函数的书写.
        不过如果回调函数中如果需要进行参数的传入和传出,使用std::function会比较麻烦,因此其常用来编写一些void(void)类型的简单功能函数,即没有输入输出的函数.


        上面所有控件的必要参数只有控件tag和初始值,其他参数不存在时 pangolin 会自动调用默认参数进行处理.
    */

    pangolin::Var<bool> A_Button("ui.a_button", false, false);    // 按钮
    pangolin::Var<bool> A_Checkbox("ui.a_checkbox", false, true); // 选框
    pangolin::Var<double> Double_Slider("ui.a_slider", 3, 0, 5);  // double滑条
    pangolin::Var<int> Int_Slider("ui.b_slider", 2, 0, 5);        // int 滑条
    pangolin::Var<std::string> A_string("ui.a_string", "hello pangolin");

    pangolin::Var<bool> SAVE_IMG("ui.save_img", false, false);     // 按钮
    pangolin::Var<bool> SAVE_WIN("ui.save_win", false, false);     // 按钮
    pangolin::Var<bool> RECORD_WIN("ui.record_win", false, false); // 按钮

    pangolin::Var<std::function<void()>> reset("ui.Reset", SampleMethod); // 通过案件调用函数

    // 绑定键盘快捷键
    // 演示我们如何使用一个键盘快捷方式来改变一个var
    // 这条函数的意思是通过 ctrl+b 将 a_slider 的滑动条的值变成 3.5
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', pangolin::SetVarFunctor<double>("ui.a_slider", 3.5));

    // 使用键盘快捷方式来调用函数
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', SampleMethod);

    // 几个默认的快捷方式是：esc 表示退出，tab 表示全屏

    while (!pangolin::ShouldQuit())
    {

        // clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 各控件的回调函数
        // button对象则需要使用  pangolin::Pushed(string tag) 函数判断其是否按下
        if (pangolin::Pushed(A_Button))
            std::cout << "Push button A." << std::endl;

        // checkbox 判断其本身的状态为 true 和 false
        if (A_Checkbox)
            Int_Slider = Double_Slider;

        // 保存整个win
        if (pangolin::Pushed(SAVE_WIN))
            pangolin::SaveWindowOnRender("window");

        // 保存view
        if (pangolin::Pushed(SAVE_IMG))
            d_cam.SaveOnRender("cube");

        // 录像
        // if( pangolin::Pushed(RECORD_WIN))
        // pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");

        d_cam.Activate(s_cam);

        pangolin::glDrawColouredCube();

        pangolin::FinishFrame();
    }

    return 0;
}

