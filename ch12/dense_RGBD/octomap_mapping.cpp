#include<iostream>
#include<fstream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<octomap/octomap.h>    // for octomap 
#include<Eigen/Geometry>
#include<boost/format.hpp>  // for formating strings

using namespace std;

int main(int argc, char **argv) 
{
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色圖和深度圖
    vector<Eigen::Isometry3d> poses;         // 相機位姿

    ifstream fin("./dense_RGBD/data/pose.txt");
    if(!fin) 
    {
        cerr << "cannot find pose file" << endl;
        return 1;
    }

    for(int i = 0; i < 5; ++i) 
    {
        boost::format fmt("./dense_RGBD/data/%s/%d.%s"); //圖像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "png").str(), -1)); // 使用-1讀取原始圖像

        double data[7] = {0};
        for(int i = 0; i < 7; ++i)
            fin >> data[i];

        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 计算點雲並拼接
    // 相機内参 
    double cx = 319.5;
    double cy = 239.5;
    double fx = 481.2;
    double fy = -480.0;
    double depthScale = 5000.0;

    cout << "正在將圖像轉換為 Octomap ..." << endl;

    // octomap tree 
    octomap::OcTree tree(0.01); // 參數為分辨率

    for(int i = 0; i < 5; ++i) 
    {
        cout << "轉換圖像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];

        octomap::Pointcloud cloud;  // the point cloud in octomap 

        for(int v = 0; v < color.rows; ++v)
        {
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue; // 為0表示沒有測量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;
                // 將世界坐標系的點放入點雲
                cloud.push_back(pointWorld[0], pointWorld[1], pointWorld[2]);
            }
        }

        // 將點雲存入八叉數地圖，给定原點，這樣可以計算投射線
        tree.insertPointCloud(cloud, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
    }

    // 更新中間節點的點據信息並寫入磁盤
    tree.updateInnerOccupancy();
    cout << "saving octomap ... " << endl;
    tree.writeBinary("octomap.bt");
    return 0;
}