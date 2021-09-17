#include<gflags/gflags.h>
#include"myslam/visual_odometry.h"

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv) 
{
    #ifdef GFLAGS_NAMESPACE
        GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
    #else
        google::ParseCommandLineFlags(&argc, &argv, true);
    #endif

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
