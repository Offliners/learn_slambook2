#include<iostream>
#include<opencv2/core/core.hpp>
#include<ceres/ceres.h>
#include<chrono>

using namespace std;

// 代價函數的計算模型
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    // 殘差的計算
    template<typename T>
    bool operator()(
    const T *const abc, // 模型參數，有3维
    T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
    return true;
    }

    const double _x, _y;    // x,y數據
};

int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;         // 真實參數值
    double ae = 2.0, be = -1.0, ce = 5.0;        // 估計參數值
    int N = 100;                                 // 數據點
    double w_sigma = 1.0;                        // 噪聲Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                 // OpenCV隨機數產生器

    vector<double> x_data, y_data;      // 數據
    for (int i = 0; i < N; ++i)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    double abc[3] = {ae, be, ce};

    // 構建最小二乘問題
    ceres::Problem problem;
    for (int i = 0; i < N; ++i)
    {
        problem.AddResidualBlock(     // 向問題中添加誤差項
            // 使用自動求導，模板參數：誤差類型，輸出维度，輸入维度，維數與前面struct中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
            new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,            // 核函数，這裡不使用，為空
            abc                 // 待估計參數
        );
    }

    // 配置求解器
    ceres::Solver::Options options;     // 這裡有很多配置項可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 輸出到cout

    ceres::Solver::Summary summary;                // 優化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 開始優化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 輸出結果
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c = ";
    for (auto a:abc) cout << a << " ";
    cout << endl;

    return 0;
}