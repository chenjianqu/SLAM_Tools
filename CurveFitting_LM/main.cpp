#include <iostream>

#include "mysolver/def.h"
#include "mysolver/vertex.h"
#include "mysolver/solver.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace mysolver;
using namespace std;


// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex(): Vertex(3) {}  // abc: 三个参数， Vertex 是 3 维的

    std::string TypeInfo() const override{
        return "abc";
    }
};


// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x, double y)
        :Edge(1,1, std::vector<std::string>{"abc"})
    {
        x_ = x;
        y_ = y;
    }

    // 计算曲线模型误差
    void ComputeResidual() override
    {
        Vec3 param = verticies_[0]->Parameters();  // 估计的参数
        //计算预测值y_pre = exp (a*x*x + b*x +c)
        auto y_pre=std::exp( param(0)*x_*x_ + param(1)*x_ + param(2) );
        // 残差 = 预测值-观测值
        residual_(0) =  y_pre - y_;
    }

    // 计算残差对优化变量的导数,即雅克比
    void ComputeJacobians() override
    {
        Vec3 abc = verticies_[0]->Parameters();
        double exp_y = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) );

        // 误差为1维，状态量 3 个，所以是 1x3 的雅克比矩阵
        //模型为exp(a*x*x + b*x + c), 对a求导为y*x*x,对b求导为y*x, 对c求导为y*1
        Eigen::Matrix<double, 1, 3> jaco_abc;
        jaco_abc <<
            x_ * x_ * exp_y,
            x_ * exp_y ,
            1 * exp_y;
        jacobians_[0] = jaco_abc;
    }

    //返回边的类型信息
    std::string TypeInfo() const override {
        return "CurveFittingEdge";
    }
public:
    double x_,y_;  // x 值， y 值为 _measurement
};




int main()
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N = 100;                          // 数据点
    double w_sigma= 1.;                 // 噪声Sigma值

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    // 构建 problem
    Solver problem(Solver::ProblemType::GENERIC_PROBLEM);

    std::shared_ptr< CurveFittingVertex > vertex(new CurveFittingVertex());
    vertex->SetParameters(Eigen::Vector3d (0.,0.,0.));// 设定待估计参数 a, b, c初始值

    // 将待估计的参数加入最小二乘问题
    problem.AddVertex(vertex);

    // 构造 N 次观测,添加边
    for (int i = 0; i < N; ++i) {
        double x = i/100.;
        double n = noise(generator);
        double y = std::exp( a*x*x + b*x + c ) + n;// 观测 y

        // 每个观测对应的残差函数
        std::shared_ptr<CurveFittingEdge> edge(new CurveFittingEdge(x,y));
        //边添加顶点
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);
        // 把这个残差添加到最小二乘问题
        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;

    problem.Solve(30);/// 使用 LM 求解

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout << "1.0,  2.0,  1.0" << std::endl;
    // std
    return 0;
}
