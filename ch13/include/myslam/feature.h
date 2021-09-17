//
// Created by gaoxiang on 19-5-2.
//
#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include<memory>
#include<opencv2/features2d.hpp>
#include"myslam/common_include.h"

namespace myslam 
{
    struct Frame;
    struct MapPoint;

    /**
     * 2D 特徵點
     * 在三角化之後会會被關聯一個地圖點
     */
    struct Feature 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;

        std::weak_ptr<Frame> frame_;         // 持有該feature的frame
        cv::KeyPoint position_;              // 2D提取位置
        std::weak_ptr<MapPoint> map_point_;  // 關聯地圖點

        bool is_outlier_ = false;       // 是否為異常點
        bool is_on_left_image_ = true;  // 標示是否提在左圖，false為右圖

    public:
        Feature() {}

        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
            : frame_(frame), position_(kp) {}
    };
}  // namespace myslam

#endif  // MYSLAM_FEATURE_H
