#pragma once
#ifndef MAP_H
#define MAP_H

#include"myslam/common_include.h"
#include"myslam/frame.h"
#include"myslam/mappoint.h"

namespace myslam 
{
    /**
     * @brief 地圖
     * 和地圖的交互：前端調用InsertKeyframe和InsertMapPoint插入新幀和地圖點，後端維護地圖的結構，判定outlier/剔除等等
     */
    class Map 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map() {}

        /// 增加一個關鍵幀
        void InsertKeyFrame(Frame::Ptr frame);
        /// 增加一個地圖頂點
        void InsertMapPoint(MapPoint::Ptr map_point);

        /// 獲取所有地圖點
        LandmarksType GetAllMapPoints() 
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        /// 獲取所有關鍵幀
        KeyframesType GetAllKeyFrames() 
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        /// 獲取激活地圖點
        LandmarksType GetActiveMapPoints() 
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        /// 獲取激活關鍵幀
        KeyframesType GetActiveKeyFrames() 
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        /// 清理map中觀測數量為零的點
        void CleanMap();

    private:
        // 將舊的關鍵幀置為不活躍狀態
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;         // all landmarks
        LandmarksType active_landmarks_;  // active landmarks
        KeyframesType keyframes_;         // all key-frames
        KeyframesType active_keyframes_;  // all key-frames

        Frame::Ptr current_frame_ = nullptr;

        // settings
        int num_active_keyframes_ = 7;  // 激活的關鍵幀數量
    };
}  // namespace myslam

#endif  // MAP_H
