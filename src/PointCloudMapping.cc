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

#include "PointCloudMapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"
#include "Config.h"
namespace ORB_SLAM2 {
    PointCloudMapping::PointCloudMapping(Map *map): mpMap(map) {

        mAllCloudPoints = boost::make_shared<PointCloud>();
        viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
    }

    void PointCloudMapping::shutdown() {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame *kf) {
//        cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lck(keyframeMutex);
        mvKeyframes.push_back(kf);
        keyFrameUpdated.notify_one();
    }


//    void PointCloudMapping::viewer() {
//        boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Plane viewer"));
//        pcl::VoxelGrid<PointT>  voxel;
//        voxel.setLeafSize( 0.02, 0.02, 0.02);
//        viewer->setBackgroundColor(255, 255, 255);
//        mbshowsp = Config::Get<int>("Plane.ShowSPlane");
//        while(1)
//        {
//
//            {
//                unique_lock<mutex> lck_shutdown(shutDownMutex);
//                if (shutDownFlag) {
//                    break;
//                }
//            }
//
//            // keyframe is updated
//            size_t N=0;
//            {
//                unique_lock<mutex> lck( keyframeMutex );
//                N = mvKeyframes.size();
//            }
//
//            for ( size_t i=0; i<N ; i++ )
//            {
//                viewer->removeAllPointClouds();
//                KeyFrame* frame = mvKeyframes[i];
//                if(frame == nullptr || frame->isBad())
//                    continue;
//                for(int j = 0; j<frame->mvpMapPlanes.size();++j){
//                    MapPlane* pMP = frame->mvpMapPlanes[j];
//                    if(pMP== nullptr)
//                        continue;
//
//                    int ir = pMP->mRed;
//                    int ig = pMP->mGreen;
//                    int ib = pMP->mBlue;
//
//                    if(mbshowsp==0 && j >= frame->mnRealPlaneNum){
//                        break;
//                    }
//                    for(auto& p : frame->mvPlanePoints[j].points){
//                        if((p.r != 255 && p.g != 255 && p.b != 255)) {
//                            p.r = ir;
//                            p.g = ig;
//                            p.b = ib;
//                        }
//                    }
//                    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( frame->GetPose() );
//                    PointCloud::Ptr cloud(new PointCloud);
//                    pcl::transformPointCloud( frame->mvPlanePoints[j], *cloud, T.inverse().matrix());
//                    PointCloud::Ptr tmp(new PointCloud());
//                    voxel.setInputCloud( cloud );
//                    voxel.filter( *tmp );
//                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(tmp);
//                    std::stringstream cloudname;
//                    cloudname << frame->mnId << "_" << j ;
//                    viewer->addPointCloud(tmp, rgb, cloudname.str());
//                }
//            }
//            lastKeyframeSize = N;
//            viewer->spinOnce();
//        }
//    }

    void PointCloudMapping::viewer() {

        boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Plane viewer"));
        pcl::VoxelGrid<PointT>  voxel;
        voxel.setLeafSize( 0.02, 0.02, 0.02);
        viewer->setBackgroundColor(255, 255, 255);
        mbshowsp = Config::Get<int>("Plane.ShowSPlane");
        while(1) {


            {
                unique_lock<mutex> lck_shutdown(shutDownMutex);
                if (shutDownFlag) {
                    break;
                }
            }

            const vector<long unsigned int> &vnRPs = mpMap->GetRemovedPlanes();

            for(auto mn : vnRPs){
                std::stringstream cloudname;
                cloudname <<  mn;
                viewer->removePointCloud(cloudname.str());
                viewer->spinOnce();
            }

            const vector<MapPlane *> &vpMPs = mpMap->GetAllMapPlanes();
            if (vpMPs.empty())
                continue;

            for (auto pMP : vpMPs) {
                std::stringstream cloudname;
                cloudname <<  pMP->mnId;

                if(pMP->isBad()) {
                    viewer->removePointCloud(cloudname.str());
                    continue;
                }

                map<KeyFrame *, int> observations = pMP->GetObservations();
                float ir = pMP->mRed;
                float ig = pMP->mGreen;
                float ib = pMP->mBlue;

                PointCloud::Ptr planeCloudPoints(new PointCloud);

                for (auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
                    KeyFrame *frame = mit->first;
                    int id = mit->second;
                    if (!mbshowsp && id >= frame->mnRealPlaneNum) {
                        continue;
                    }
                    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(frame->GetPose());
                    PointCloud::Ptr cloud(new PointCloud);
                    pcl::transformPointCloud(frame->mvPlanePoints[id], *cloud, T.inverse().matrix());
                    *planeCloudPoints += *cloud;
                }
                PointCloud::Ptr tmp(new PointCloud());
                voxel.setInputCloud(planeCloudPoints);
                voxel.filter(*tmp);

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(tmp, ir, ig, ib);

                viewer->removePointCloud(cloudname.str());
                viewer->addPointCloud(tmp, single_color, cloudname.str());
                viewer->spinOnce();
            }
        }
    }
}



















