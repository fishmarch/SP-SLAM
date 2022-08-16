/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include <Timer.h>
namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mvPlanePoints(frame.mvPlanePoints), mvPlaneCoefficients(frame.mvPlaneCoefficients), mbNewPlane(frame.mbNewPlane),
     mvpMapPlanes(frame.mvpMapPlanes), mnPlaneNum(frame.mnPlaneNum), mvbPlaneOutlier(frame.mvbPlaneOutlier),
     mvpParallelPlanes(frame.mvpParallelPlanes), mvpVerticalPlanes(frame.mvpVerticalPlanes), mvBoundaryPoints(frame.mvBoundaryPoints),
     mvNotSeenPlaneCoefficients(frame.mvNotSeenPlaneCoefficients), mvNotSeenBoundaryPoints(frame.mvNotSeenBoundaryPoints),
     mnNotSeenPlaneNum(frame.mnNotSeenPlaneNum), mvbNotSeenPlaneOutlier(frame.mvbNotSeenPlaneOutlier),mvpNotSeenMapPlanes(frame.mvpNotSeenMapPlanes),
     mnRealPlaneNum(frame.mnRealPlaneNum)
//    mvNotSeenPlanePoints(frame.mvNotSeenPlanePoints)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


//stereo
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

//RGB-D
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), mbNewPlane(false)
{
    // Frame ID
    mnId=nNextId++;
    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);


    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();

//    ComputePlanesFromPointCloud(imDepth);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    ComputePlanesFromOrganizedPointCloud(imDepth);
    mnRealPlaneNum = mvPlanePoints.size();
    ORB_SLAM2::Timer::AddPlane(mnRealPlaneNum);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tt= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    ORB_SLAM2::Timer::SetTPlane(tt);

    GeneratePlanesFromBoundries(imDepth);
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    double tt2= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    ORB_SLAM2::Timer::SetTSPlane(tt2);

    mnPlaneNum = mvPlanePoints.size();

    ORB_SLAM2::Timer::AddSPlane(mnPlaneNum - mnRealPlaneNum);

    mvpMapPlanes = vector<MapPlane*>(mnPlaneNum,static_cast<MapPlane*>(nullptr));
    mvpParallelPlanes = vector<MapPlane*>(mnPlaneNum,static_cast<MapPlane*>(nullptr));
    mvpVerticalPlanes = vector<MapPlane*>(mnPlaneNum,static_cast<MapPlane*>(nullptr));

    mvbPlaneOutlier = vector<bool>(mnPlaneNum,false);
    mvbVerPlaneOutlier = vector<bool>(mnPlaneNum,false);
    mvbParPlaneOutlier = vector<bool>(mnPlaneNum,false);

    mnNotSeenPlaneNum = mvNotSeenPlaneCoefficients.size();
    mvpNotSeenMapPlanes = vector<MapPlane*>(mnNotSeenPlaneNum,static_cast<MapPlane*>(nullptr));
    mvbNotSeenPlaneOutlier = vector<bool>(mnNotSeenPlaneNum,false);
//    if(mvPlanePoints.size() > mnRealPlaneNum + 1) {
//        boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("viewer"));
//        viewer->setBackgroundColor(255, 255, 255);
//
//        for(int i = 0; i < mvPlanePoints.size(); ++i){
//            PointCloud::Ptr tmp = mvPlanePoints[i].makeShared();
//
//            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(tmp);
//            std::stringstream cloudname;
//            cloudname << "cloud_" << i ;
//            viewer->addPointCloud(tmp, rgb, cloudname.str());
//        }
//        while (1){
//            viewer->spinOnce();
//        }
//    }

//    if(mvPlanePoints.size() > mnRealPlaneNum + 1) {
//        pcl::visualization::CloudViewer viewer("plane viewer");
//        PointCloud::Ptr showingCloud(new PointCloud);
//
//        for(int i = 0; i < mvPlanePoints.size(); ++i){
//            *showingCloud += mvPlanePoints[i];
//        }
//        viewer.showCloud(showingCloud);
//        getchar();
//    }

//    if(mvNotSeenPlanePoints.size() > 0) {
//        pcl::visualization::CloudViewer viewer("plane viewer");
//
//        PointCloud::Ptr showingCloud(new PointCloud);
//
//        for (int x = 0; x < mvPlanePoints.size(); ++x) {
//            *showingCloud += mvPlanePoints[x];
//        }
//
//        for (int i = 0; i < mvNotSeenPlanePoints.size(); ++i) {
//            *showingCloud += mvNotSeenPlanePoints[i];
//        }
//
//        for (int j = 0; j < mvBoundaryPoints.size(); ++j) {
//            for (auto &p : mvBoundaryPoints[j].points) {
//                p.r = 0;
//                p.g = 255;
//                p.b = 0;
//            }
//            *showingCloud += mvBoundaryPoints[j];
//        }
//
//        viewer.showCloud(showingCloud);
//        getchar();
//    }
}

// monocular
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;

    mTwc = cv::Mat::eye(4,4,mTcw.type());
    mRwc.copyTo(mTwc.rowRange(0,3).colRange(0,3));
    mOw.copyTo(mTwc.rowRange(0,3).col(3));
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

//             Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

void Frame::ComputePlanesFromPointCloud(const cv::Mat &imDepth) {
    PointCloud::Ptr inputCloud( new PointCloud() );
    float max = Config::Get<float>("Plane.MaxDistance");
    for ( int m=0; m<imDepth.rows; m+=3 )
    {
        for ( int n=0; n<imDepth.cols; n+=3 )
        {
            float d = imDepth.ptr<float>(m)[n];
            if (d < 0.01 || d > max)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - cx) * p.z / fx;
            p.y = ( m - cy) * p.z / fy;

            inputCloud->points.push_back(p);
        }
    }
    if(inputCloud->points.size() < 1000)
        return;

    float disTh = Config::Get<float>("Plane.DistanceThreshold");
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(disTh);

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(inputCloud);
    float leafSize = Config::Get<float>("Plane.LeafSize");
    sor.setLeafSize(leafSize,leafSize,leafSize);
    sor.filter(*inputCloud);

    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(inputCloud);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(20);
    outrem.filter(*inputCloud);

    pcl::ExtractIndices<PointT> extract;
    int nr_points = (int) inputCloud->points.size();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    PointCloud::Ptr planeCloud(new PointCloud());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    int min_plane = Config::Get<int>("Plane.MinSize");

    while(inputCloud->points.size() > 0.3*nr_points){
        seg.setInputCloud(inputCloud);
        seg.segment(*inliers,*coefficients);
        if(inliers->indices.size() < min_plane)
            break;
        extract.setInputCloud(inputCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*planeCloud);
        mvPlanePoints.push_back(*planeCloud);

        cv::Mat coef = (cv::Mat_<float>(4,1) << coefficients->values[0],
                                                coefficients->values[1],
                                                coefficients->values[2],
                                                coefficients->values[3]);
        mvPlaneCoefficients.push_back(coef);
        extract.setNegative(true);
        extract.filter(*planeCloud);
        inputCloud.swap(planeCloud);
    }

}

    void Frame::ComputePlanesFromOrganizedPointCloud(const cv::Mat &imDepth) {
        PointCloud::Ptr inputCloud( new PointCloud() );
        int cloudDis = Config::Get<int>("Cloud.Dis");
        for ( int m=0; m<imDepth.rows; m+=cloudDis )
        {
            for ( int n=0; n<imDepth.cols; n+=cloudDis )
            {
                float d = imDepth.ptr<float>(m)[n];
                PointT p;
                p.z = d;
                p.x = ( n - cx) * p.z / fx;
                p.y = ( m - cy) * p.z / fy;
                p.r = 0;
                p.g = 0;
                p.b = 250;

                inputCloud->points.push_back(p);
            }
        }
        inputCloud->height = ceil(imDepth.rows/float(cloudDis));
        inputCloud->width = ceil(imDepth.cols/float(cloudDis));
//尝试降采样降低计算量

        //估计法线
        pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.05f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(inputCloud);
        //计算特征值
        ne.compute(*cloud_normals);


        int min_plane = Config::Get<int>("Plane.MinSize");
        float AngTh = Config::Get<float>("Plane.AngleThreshold");
        float DisTh = Config::Get<float>("Plane.DistanceThreshold");

        vector<pcl::ModelCoefficients> coefficients;
        vector<pcl::PointIndices> inliers;
        pcl::PointCloud<pcl::Label>::Ptr labels ( new pcl::PointCloud<pcl::Label> );
        vector<pcl::PointIndices> label_indices;
        vector<pcl::PointIndices> boundary;

        pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
        mps.setMinInliers (min_plane);
        mps.setAngularThreshold (0.017453 * AngTh);
        mps.setDistanceThreshold (DisTh);
        mps.setInputNormals (cloud_normals);
        mps.setInputCloud (inputCloud);
        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
        mps.segmentAndRefine (regions, coefficients, inliers, labels, label_indices, boundary);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(inputCloud);
        extract.setNegative(false);

//        srand(time(0));
        for (int i = 0; i < inliers.size(); ++i) {
            PointCloud::Ptr planeCloud(new PointCloud());
            cv::Mat coef = (cv::Mat_<float>(4,1) << coefficients[i].values[0],
                    coefficients[i].values[1],
                    coefficients[i].values[2],
                    coefficients[i].values[3]);
            if(coef.at<float>(3) < 0)
                coef = -coef;

            if(!PlaneNotSeen(coef)){
                continue;
            }
            extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers[i]));
            extract.filter(*planeCloud);

            mvPlanePoints.push_back(*planeCloud);

            PointCloud::Ptr boundaryPoints(new PointCloud());
            boundaryPoints->points = regions[i].getContour();
            mvBoundaryPoints.push_back(*boundaryPoints);
            mvPlaneCoefficients.push_back(coef);
        }

    }

    void Frame::GeneratePlanesFromBoundries(const cv::Mat &imDepth) {
        pcl::SACSegmentation<PointT> segLine;
        pcl::ExtractIndices<PointT> extract;
        double lineRatio = Config::Get<double>("Line.Ratio");
        float disTh = Config::Get<float>("Line.DistanceThreshold");
        segLine.setOptimizeCoefficients(true);
        segLine.setModelType(pcl::SACMODEL_LINE);
        segLine.setMaxIterations(1000);
        segLine.setDistanceThreshold(disTh);
        PointCloud::Ptr boundPoints(new pcl::PointCloud<PointT>);
        PointCloud::Ptr tempPoints(new pcl::PointCloud<PointT>);
        PointCloud::Ptr linePoints(new pcl::PointCloud<PointT>);
        pcl::PointIndices::Ptr lineins (new pcl::PointIndices ());
        pcl::ModelCoefficients::Ptr coeffline (new pcl::ModelCoefficients ());

        int iend = mvBoundaryPoints.size() - 1;
        for(int i=iend; i >= 0; --i){
            boundPoints->points = mvBoundaryPoints[i].points;
            int boundSize = boundPoints->points.size();
            if(boundSize < 50){
                if(boundSize == 0)
                    GenerateBoundaryPoints(i);
                continue;
            }
            for(int j=0; j < 4; j++) {
                segLine.setInputCloud(boundPoints);
                segLine.segment(*lineins, *coeffline);
                if (lineins->indices.size() < lineRatio * boundSize){
                    break;
                }

                extract.setInputCloud(boundPoints);
                extract.setIndices(lineins);
                extract.setNegative(false);
                extract.filter(*linePoints);
                cv::Mat pc = (cv::Mat_<float>(3,1) << coeffline->values[0], coeffline->values[1], coeffline->values[2]);

                if(LineInRange(pc) && IsBorderLine(linePoints,imDepth)) {
                    cv::Mat coef = (cv::Mat_<float>(6,1) << coeffline->values[0],
                            coeffline->values[1],
                            coeffline->values[2],
                            coeffline->values[3],
                            coeffline->values[4],
                            coeffline->values[5]);
                    if(CaculatePlanes(mvPlaneCoefficients[i], coef)){
                        for(auto &p : linePoints->points){
                            p.r = 255;
                            p.g = 0;
                            p.b = 0;
                        }
                        mvPlanePoints[mvPlanePoints.size()-1] += *linePoints;
                        mvBoundaryPoints.push_back(*linePoints);
                    }else {
                    }
                }
                extract.setNegative(true);
                extract.filter(*tempPoints);
                boundPoints.swap(tempPoints);
            }
        }

}

    void Frame::GenerateBoundaryPoints(int i) {
        for(int j=0;j<mvPlanePoints[i].points.size();j+=20){
            const PointT &pPlane = mvPlanePoints[i].points[j];
            PointT p;
            p.z = pPlane.z;
            p.x = pPlane.x;
            p.y = pPlane.y;
            mvBoundaryPoints[i].points.push_back(p);
        }

}

    bool Frame::IsBorderLine(const PointCloud::Ptr line, const cv::Mat &imDepth) {
        int s = line->points.size();
        int res = 0;
        for(auto p:line->points){
            cv::Mat pc = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
            if(!IsBorderPoint(pc,imDepth))
                res ++;
            if(res > s/4)
                return false;
        }
        return true;
}

    bool Frame::IsBorderPoint(const cv::Mat &Pc, const cv::Mat &imDepth) {
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);
        if(PcZ<0.0f)
            return false;

        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;
        int num = 0, nan = 0;
        float res = 0;
        int b = 10;
        for(int j = v - b; j < v + b; ++j){
            for(int i = u - b; i < u + b; ++i){
                if(imDepth.ptr<float>(j)[i] > 0.05){
                    res += imDepth.ptr<float>(j)[i];
                    num++;
                }else{
                    nan++;
                    if(nan > b*b){
                        return false;
                    }
                }
            }
        }
        if(PcZ - res/num > 0.1){
            return false;
        }
        return true;
}

    bool Frame::LineInRange(const cv::Mat &Pc) {
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);
        if(PcZ<0.0f)
            return false;

        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;
        if(u<(mnMinX+50) || u>(mnMaxX-50))
            return false;
        if(v<(mnMinY+50) || v>(mnMaxY-50))
            return false;

        return true;
}


    bool Frame::CaculatePlanes(const cv::Mat &inputplane, const cv::Mat &inputline) {
        //(l,m,n) × (o,p,q) = (mq-np,no-lq,lp-mo)

        float a,b,c,d;
        a = inputplane.at<float>(1)*inputline.at<float>(5) - inputplane.at<float>(2)*inputline.at<float>(4);
        b = inputplane.at<float>(2)*inputline.at<float>(3) - inputplane.at<float>(0)*inputline.at<float>(5);
        c = inputplane.at<float>(0)*inputline.at<float>(4) - inputplane.at<float>(1)*inputline.at<float>(3);
        d = a*inputline.at<float>(0) + b*inputline.at<float>(1) + c*inputline.at<float>(2);
        float v = sqrt(a*a + b*b + c*c);

        cv::Mat coef = (cv::Mat_<float>(4,1) << a/v, b/v, c/v, -d/v);
        if(coef.at<float>(3) < 0)
            coef = -coef;
        if(PlaneNotSeen(coef)){

            PointCloud cloud;
            PointT p;

            for(float i = -0.25; i< 0.25;){
                for(float j = -0.25; j < 0.25;){
                    p.x = inputline.at<float>(0) + i * inputline.at<float>(3) + j * inputplane.at<float>(0);
                    p.y = inputline.at<float>(1) + i * inputline.at<float>(4) + j * inputplane.at<float>(1);
                    p.z = (coef.at<float>(0)*p.x + coef.at<float>(1)*p.y + coef.at<float>(3)) / (-coef.at<float>(2));
                    p.r = 0;
                    p.g = 255;
                    p.b = 0;
                    cloud.points.push_back(p);
                    j = j + 0.01;
                }
                i = i + 0.01;
            }

            mvPlaneCoefficients.push_back(coef);
            mvPlanePoints.push_back(cloud);
            return true;
        }
        return false;
}

    bool Frame::PlaneNotSeen(const cv::Mat &coef) {
        for (int j = 0; j < mvPlaneCoefficients.size(); ++j) {
            cv::Mat pM = mvPlaneCoefficients[j];
            float d = pM.at<float>(3,0) - coef.at<float>(3,0);
            float angle = pM.at<float>(0,0) * coef.at<float>(0,0) +
                          pM.at<float>(1,0) * coef.at<float>(1,0) +
                          pM.at<float>(2,0) * coef.at<float>(2,0);

            if(d > 0.2 || d < -0.2)
                continue;

            if(angle < 0.9397 && angle > -0.9397)
                continue;
            return false;
        }
        for (int j = 0; j < mvNotSeenPlaneCoefficients.size(); ++j) {
            cv::Mat pM = mvNotSeenPlaneCoefficients[j];
            float d = pM.at<float>(3,0) - coef.at<float>(3,0);
            float angle = pM.at<float>(0,0) * coef.at<float>(0,0) +
                          pM.at<float>(1,0) * coef.at<float>(1,0) +
                          pM.at<float>(2,0) * coef.at<float>(2,0);
            if(d > 0.2 || d < -0.2)
                continue;
            if(angle < 0.9397 && angle > -0.9397)
                continue;
            return false;
        }
        return true;
}

    cv::Mat Frame::ComputePlaneWorldCoeff(const int &idx) {
        cv::Mat temp;
        cv::transpose(mTcw, temp);
        return temp*mvPlaneCoefficients[idx];
    }

    cv::Mat Frame::ComputeNotSeenPlaneWorldCoeff(const int &idx) {
        cv::Mat temp;
        cv::transpose(mTcw, temp);
        return temp*mvNotSeenPlaneCoefficients[idx];
    }
} //namespace ORB_SLAM
