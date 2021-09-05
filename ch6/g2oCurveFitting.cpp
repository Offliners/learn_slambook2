#include<iostream>
#include<g2o/core/g2o_core_api.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<chrono>
#include<fstream>

using namespace std;

// 曲線模型的頂點，模板參數：優化變量維度和數據類型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置
    virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    // 更新
    virtual void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector3d(update);
    }

    // 存取和讀寫：留空
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

// 誤差模型 模板參數：觀測值維度，類型，連接頂點類型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    // 計算曲線模型誤差
    virtual void computeError() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }

    // 計算雅可比矩陣
    virtual void linearizeOplus() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

public:
    double _x;  // x 值， y 值為 _measurement
};

int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;         // 真實參數值
    double ae = 2.0, be = -1.0, ce = 5.0;        // 估計參數值
    int N = 100;                                 // 數據點
    double w_sigma = 1.0;                        // 噪聲Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                 // OpenCV隨機數產生器
    ofstream output;
    output.open("output/g2oCurveFitting_result.txt");

    vector<double> x_data, y_data;      // 數據
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
        output << x_data[i] << " " << y_data[i] << endl;
    }

    // 構建圖優化，先設定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;  // 每個誤差項優化變量維度為3，誤差值維度為1
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 線性求解器類型

    // 梯度下降方法，可以從GN, LM, DogLeg 中選
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 圖模型
    optimizer.setAlgorithm(solver);   // 設置求解器
    optimizer.setVerbose(true);       // 打開調適输出

    // 往圖中增加頂點
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    // 往圖中增加邊
    for (int i = 0; i < N; ++i)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);                // 設置連接的頂點
        edge->setMeasurement(y_data[i]);      // 觀測數值
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // 信息矩陣：協方差矩陣之逆
        optimizer.addEdge(edge);
    }

    // 執行優化
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出優化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;

    output << ar << " " << br << " " << cr << endl;
    output << abc_estimate.transpose() << endl;
    output << time_used.count() << endl;
    output.close();

    return 0;
}