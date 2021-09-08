#include<iostream>
#include<fstream>
#include<string>
#include<Eigen/Core>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_binary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/eigen/linear_solver_eigen.h>
#include<sophus/se3.hpp>

using namespace std;
using namespace Eigen;
using Sophus::SE3d;
using Sophus::SO3d;

/************************************************
 * 本程序演示如何用g2o solver進行位姿圖優化
 * sphere.g2o是人工生成的一個Pose graph，我們來優化它。
 * 儘管可以直接通過load函数讀取整個圖，但我們還是自己来實現讀取代碼，以期獲得更深刻的理解
 * 本節使用李代數表達位姿圖，節點和邊的方式為自定義
 * **********************************************/

typedef Matrix<double, 6, 6> Matrix6d;

// 给定誤差求J_R^{-1}的近似
Matrix6d JRInv(const SE3d &e)
{
    Matrix6d J;
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());
    // J = J * 0.5 + Matrix6d::Identity();
    J = Matrix6d::Identity();    // try Identity if you want
    return J;
}

// 李代數頂點
typedef Matrix<double, 6, 1> Vector6d;

class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(istream &is) override {
        double data[7];
        for (int i = 0; i < 7; i++)
            is >> data[i];
        setEstimate(SE3d(
            Quaterniond(data[6], data[3], data[4], data[5]),
            Vector3d(data[0], data[1], data[2])
        ));
    }

    virtual bool write(ostream &os) const override {
        os << id() << " ";
        Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;
        return true;
    }

    virtual void setToOriginImpl() override {
        _estimate = SE3d();
    }

    // 左乘更新
    virtual void oplusImpl(const double *update) override {
        Vector6d upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = SE3d::exp(upd) * _estimate;
    }
};

// 两個李代數節點之邊
class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(istream &is) override {
        double data[7];
        for (int i = 0; i < 7; ++i)
            is >> data[i];
        Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(SE3d(q, Vector3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); ++i)
        {
            for (int j = i; j < information().cols() && is.good(); ++j) 
            {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    virtual bool write(ostream &os) const override {
        VertexSE3LieAlgebra *v1 = static_cast<VertexSE3LieAlgebra *> (_vertices[0]);
        VertexSE3LieAlgebra *v2 = static_cast<VertexSE3LieAlgebra *> (_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix 
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << information()(i, j) << " ";

        os << endl;
        return true;
    }

    // 誤差計算與書中推導一致
    virtual void computeError() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    // 雅可比計算
    virtual void linearizeOplus() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        Matrix6d J = JRInv(SE3d::exp(_error));
        // 嘗試把J近似為I？
        _jacobianOplusXi = -J * v2.inverse().Adj();
        _jacobianOplusXj = J * v2.inverse().Adj();
    }
};

int main(int argc, char **argv) 
{
    if (argc != 2) 
    {
        cout << "Usage: pose_graph_g2o_SE3_lie sphere.g2o" << endl;
        return 1;
    }

    ifstream fin(argv[1]);
    if (!fin) 
    {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    // 設定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 圖模型
    optimizer.setAlgorithm(solver);   // 設置求解器
    optimizer.setVerbose(true);       // 打開調試輸出

    int vertexCnt = 0, edgeCnt = 0; // 頂點和邊的數量

    vector<VertexSE3LieAlgebra *> vectices;
    vector<EdgeSE3LieAlgebra *> edges;
    while(!fin.eof()) 
    {
        string name;
        fin >> name;
        if(name == "VERTEX_SE3:QUAT") 
        {
            // 頂點
            VertexSE3LieAlgebra *v = new VertexSE3LieAlgebra();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            vectices.push_back(v);
            if (index == 0)
                v->setFixed(true);
        } 
        else if(name == "EDGE_SE3:QUAT") 
        {
            // SE3-SE3 邊
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();
            int idx1, idx2;     // 關聯的两個頂點
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }

        if(!fin.good()) 
            break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;

    // 因為用了自定義頂點且没有向g2o註冊，這裡保存自己來實現
    // 偽裝成 SE3 頂點和邊，讓 g2o_viewer 可以認出
    ofstream fout("result_lie.g2o");
    for(VertexSE3LieAlgebra *v:vectices) 
    {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for(EdgeSE3LieAlgebra *e:edges) 
    {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();
    return 0;
}