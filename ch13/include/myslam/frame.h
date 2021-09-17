#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include"myslam/camera.h"
#include"myslam/common_include.h"

namespace myslam 
{
    // forward declare
    struct MapPoint;
    struct Feature;

    /**
     * 幀
     * 每一幀分配獨立id，關鍵幀分配關鍵幀ID
     */
    struct Frame 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;           // id of this frame
        unsigned long keyframe_id_ = 0;  // id of key frame
        bool is_keyframe_ = false;       // 是否為關鍵幀
        double time_stamp_;              // 時間戳，暫不使用
        SE3 pose_;                       // Tcw 形式Pose
        std::mutex pose_mutex_;          // Pose數據鎖
        cv::Mat left_img_, right_img_;   // stereo images

        // extracted features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // corresponding features in right image, set to nullptr if no corresponding
        std::vector<std::shared_ptr<Feature>> features_right_;

    public:  // data members
        Frame() {}

        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
            const Mat &right);

        // set and get pose, thread safe
        SE3 Pose() 
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void SetPose(const SE3 &pose) 
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        /// 設置關鍵幀並分配關鍵幀id
        void SetKeyFrame();

        /// 工廠建構模式，分配id 
        static std::shared_ptr<Frame> CreateFrame();
    };
}  // namespace myslam

#endif  // MYSLAM_FRAME_H
