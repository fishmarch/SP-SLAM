//
// Created by fishmarch on 19-7-27.
//

#include "Timer.h"

namespace ORB_SLAM2 {
    shared_ptr<Timer> Timer::mTimer = nullptr;

    void Timer::StartTimer(int size) {
        if (mTimer == nullptr)
            mTimer = shared_ptr<Timer>(new Timer);

        mTimer->mTPlane.reserve(size);
        mTimer->mTSPlane.reserve(size);
        mTimer->mTTrack.reserve(size);
        mTimer->mTLocalBA.reserve(size);
        mTimer->mnPlane = 0;
        mTimer->mnSPlane = 0;
        mTimer->mnAllPlane = 0;
        mTimer->mnPlaneLM = 0;
    }

    double Timer::GetAveTPlane() {
        double res = 0.0;
        for(auto t : mTimer->mTPlane){
            res += t;
        }
        return res/mTimer->mTPlane.size();
    }

    double Timer::GetAveTSPlane() {
        double res = 0.0;
        for(auto t : mTimer->mTSPlane){
            res += t;
        }
        return res/mTimer->mTSPlane.size();
    }

    double Timer::GetAveTTrack() {
        double res = 0.0;
        for(auto t : mTimer->mTTrack){
            res += t;
        }
        return res/mTimer->mTTrack.size();
    }

    double Timer::GetAveTLocalBA() {
        double res = 0.0;
        for(auto t : mTimer->mTLocalBA){
            res += t;
        }
        return res/mTimer->mTLocalBA.size();
    }
}