#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

//代价函数计算模型
struct CURVE_FITTING_COST{
    CURVE_FITTING_COST(double x,double y):_x(x),_y(y){} //构造函数

    template<typename T>
    //残差的计算(通过重载()运算符, 这样该类就成了一个拟函数, 对于该类的莫某个对象a,ceres会调用a<double>()方法,
    // ceres会通过传入雅可比矩阵作为类型参数传入该函数,从而实现自动求导)
    bool operator()(
            const T *const abc,//模型参数
            T *residual  //
            ) const
    {
        //里面的exp要用ceres自带的
        residual[0]=T(_y)-ceres::exp(abc[0]*T(_x)*T(_x)+abc[1]*T(_x)+abc[2]);// y-exp(ax^2+bx+c)
        return true;
    }

    const double _x,_y; //x,y数据
};




int main(int argc, char **argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
    int N = 100;                                 // 数据点
    double w_sigma = 1.0;                        // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                 // OpenCV随机数产生器

    vector<double> x_data, y_data;      // 数据
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    double abc[3] = {ae, be, ce};

    ///使用Ceres求解非线性问题的步骤
    ///1.定义参数块，即优化变量
    ///2.定义残差块，即代价函数
    ///3.定义雅可比矩阵的计算方式，
    ///4.配置求解器

    ceres::Problem problem;

    for (int i = 0; i < N; i++) {
        auto cost_factor=new CURVE_FITTING_COST(x_data[i], y_data[i]);
        // 使用自动求导，模板参数：误差类型，输出维度(误差的维度)，输入维度(a b c)，维数要与前面struct中一致
        auto cost_function= new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(cost_factor);
        // 向问题中添加误差项
        problem.AddResidualBlock(cost_function,nullptr,abc); // 核函数，这里不使用，为空, abc为待估计参数
    }

    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    auto t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始优化
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出结果
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c = ";
    for (auto a:abc) cout << a << " ";
    cout << endl;

    return 0;
}
