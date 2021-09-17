#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include"myslam/common_include.h"
#include"myslam/frame.h"
#include"myslam/map.h"

namespace myslam 
{
    class Map;

    /**
     * 後端
     * 有單獨優化線程，在Map更新時啟動優化
     * Map更新由前端觸發
     */ 
    class Backend 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        /// 構造函數中啟動優化線程並掛起
        Backend();

        // 設置左右目的相機，用於獲得内外參
        void SetCameras(Camera::Ptr left, Camera::Ptr right) 
        {
            cam_left_ = left;
            cam_right_ = right;
        }

        /// 設置地圖
        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        /// 觸發地圖更新，啟動優化
        void UpdateMap();

        /// 關閉後端線程
        void Stop();

    private:
        /// 後端線程
        void BackendLoop();

        /// 對给定關鍵幀和路標點進行優化
        void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
    };
}  // namespace myslam

#endif  // MYSLAM_BACKEND_H