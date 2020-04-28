/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include "KeyFrame.h"
#include "Map.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <condition_variable>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/cloud_viewer.h>


namespace ORB_SLAM2 {
class KeyFrame;
class Map;
    class PointCloudMapping {
    public:
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

        PointCloudMapping(Map* map);

        // 插入一个keyframe，会更新一次地图
        void insertKeyFrame(KeyFrame *kf);

        void shutdown();

        void viewer();

    protected:
        Map* mpMap;

        void AddKFPointCloud(KeyFrame *pKF);

        PointCloud::Ptr mAllCloudPoints;

        shared_ptr<thread> viewerThread;

        bool shutDownFlag = false;
        mutex shutDownMutex;

        condition_variable keyFrameUpdated;
        mutex keyFrameUpdateMutex;

        // data to generate point clouds
        vector<KeyFrame *> mvKeyframes;

        mutex keyframeMutex;
        uint16_t lastKeyframeSize = 0;
        int mbshowsp = 0;

    };
}
#endif // POINTCLOUDMAPPING_H
