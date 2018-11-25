#include "imustate.h"

namespace ORB_SLAM2
{

ImuState::ImuState()
{
    //变量初始化
    mImu_w.setZero();
    //mImu_a.setZero();
    //mPos.setZero();
    //mVel.setZero();
    mQua.setIdentity();

    g << 0, 0, 9.7919;
    mbw << 0,0,0;
    //mba << 0,0,0;
    //mImu_a = g;
    mTime = 0;
    // imuShiftX=0;
    // imuShiftY=0;
    // imuShiftZ=0;
    // imuVelocityX=0;
    // imuVelocityY=0;
    // imuVelocityZ=0;
}

}// namespace ORB_SLAM