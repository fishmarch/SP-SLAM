//
// Created by fishmarch on 19-5-24.
//

#ifndef ORB_SLAM2_MAPPLANE_H
#define ORB_SLAM2_MAPPLANE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "Converter.h"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>


namespace ORB_SLAM2 {
    class KeyFrame;
    class Frame;
    class Map;
    class MapPlane {
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud <PointT> PointCloud;
    public:
        MapPlane(const cv::Mat &Pos, KeyFrame* pRefKF, int idx, Map* pMap, bool s = true);

        void SetWorldPos(const cv::Mat &Pos);
        cv::Mat GetWorldPos();

        void AddObservation(KeyFrame* pKF, int idx);
        void AddNotSeenObservation(KeyFrame* pKF, int idx);
        void AddParObservation(KeyFrame* pKF, int idx);
        void AddVerObservation(KeyFrame* pKF, int idx);

        void EraseObservation(KeyFrame* pKF);
        void EraseParObservation(KeyFrame* pKF);
        void EraseVerObservation(KeyFrame* pKF);
        void EraseNotSeenObservation(KeyFrame* pKF);
        map<KeyFrame*, int> GetObservations();
        map<KeyFrame*, int> GetNotSeenObservations();
        map<KeyFrame*, int> GetParObservations();
        map<KeyFrame*, int> GetVerObservations();
        int GetIndexInKeyFrame(KeyFrame *pKF);
        int GetNotSeenIndexInKeyFrame(KeyFrame *pKF);
        void UpdateBoundary(const Frame& pF, int id);
        bool IsInKeyFrame(KeyFrame* pKF);
        bool isBad();
        void Replace(MapPlane* pMP);
        KeyFrame* GetReferenceKeyFrame();
    public:
        long unsigned int mnId; ///< Global ID for MapPlane;
        static long unsigned int nLastId;
        static std::mutex mGlobalMutex;
        long unsigned int mnBALocalForKF; //used in local BA
        long unsigned int mnCorrectedByKF; //used by loop closing
        long unsigned int mnCorrectedReference; //used by loop closing
        long unsigned int mnLoopPointForKF; //used by loop closing
        long unsigned int mnBAGlobalForKF;
        PointCloud::Ptr mvBoundaryPoints;
        bool mbSeen;
        cv::Mat mPosGBA;
        //used for visualization
        int mRed;
        int mGreen;
        int mBlue;
    protected:
        cv::Mat mWorldPos; ///< Position in absolute coordinates
        std::map<KeyFrame*, int> mObservations;
        std::map<KeyFrame*, int> mNotSeenObservations;
        std::map<KeyFrame*, int> mParObservations;
        std::map<KeyFrame*, int> mVerObservations;

        std::mutex mMutexPos;
        std::mutex mMutexFeatures;

        Map* mpMap;
        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad;
        MapPlane* mpReplaced;

        // Reference KeyFrame
        KeyFrame* mpRefKF;
//        void SetColor();
    };
}
#endif //ORB_SLAM2_MAPPLANE_H
