#include<iostream>
#include<fstream>
#include<string>
#include<g2o/types/slam3d/types_slam3d.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

/************************************************
 * 本程序演示如何用g2o solver進行位姿圖優化
 * sphere.g2o是人工生成的一個Pose graph，我們来優化它。
 * 儘管可以直接通過load函數讀取整個圖，但我們還是自己來實現讀取代碼，以期獲得更深刻的理解
 * 這裡使用g2o/types/slam3d/中的SE3表示位姿，它實質上是四元數而非李代數.
 * **********************************************/

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
        return 1;
    }

    ifstream fin(argv[1]);
    if(!fin)
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
    while(!fin.eof())
    {
        string name;
        fin >> name;
        if(name == "VERTEX_SE3:QUAT")
        {
            // SE3 頂點
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if (index == 0)
                v->setFixed(true);
        } 
        else if(name == "EDGE_SE3:QUAT") 
        {
            // SE3-SE3 邊
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2;     // 關聯的两個頂點
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        
        if (!fin.good()) 
            break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;
    optimizer.save("result.g2o");

    return 0;
}