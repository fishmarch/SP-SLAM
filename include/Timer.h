//
// Created by fishmarch on 19-7-27.
//

#ifndef ORB_SLAM2_TIMER_H
#define ORB_SLAM2_TIMER_H

#include <vector>
#include <memory>
using namespace std;

namespace ORB_SLAM2 {
    class Timer{
    public:
        static void StartTimer(int size = 100);
        static void SetTPlane(double t){
            mTimer->mTPlane.push_back(t);
        }
        static void SetTSPlane(double t){
            mTimer->mTSPlane.push_back(t);
        }
        static void SetTTrack(double t){
            mTimer->mTTrack.push_back(t);
        }
        static void SetTLocalBA(double t){
            mTimer->mTLocalBA.push_back(t);
        }
        static void AddPlane(int n){
            mTimer->mnPlane += n;
            mTimer->mnAllPlane += n;
        }
        static void AddSPlane(int n){
            mTimer->mnSPlane += n;
            mTimer->mnAllPlane += n;
        }

        static void SetPlaneLMNum(int n){
            mTimer->mnPlaneLM = n;
        }

        static double GetAveTPlane();
        static double GetAveTSPlane();
        static double GetAveTTrack();
        static double GetAveTLocalBA();

        static int GetPlaneNum(){return mTimer->mnPlane;}
        static int GetSPlaneNum(){return mTimer->mnSPlane;}
        static int GetAllPlaneNum(){return mTimer->mnAllPlane;}
        static int GetPlaneLMNum(){return mTimer->mnPlaneLM;}

    private:
        Timer(){}
        static std::shared_ptr<Timer> mTimer;
        vector<double> mTPlane;
        vector<double> mTSPlane;
        vector<double> mTTrack;
        vector<double> mTLocalBA;
        int mnPlane;
        int mnSPlane;
        int mnAllPlane;
        int mnPlaneLM;
    };
}






#endif //ORB_SLAM2_TIMER_H
