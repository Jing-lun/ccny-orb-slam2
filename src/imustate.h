#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace ORB_SLAM2
{

class ImuState
{
public:

    ImuState();

    Eigen::Matrix<double,3,1> mImu_w;         // IMU的角速度，测量值
    Eigen::Matrix<double,3,1> mImu_a;         // IMU的线加速度，测量值
    Eigen::Quaternion<double> mImu_ori;       // IMU的四元数，测量值

    Eigen::Matrix<double,3,1> mMag;

    Eigen::Matrix<double, 3, 1> mPos;         //解算的位置
    Eigen::Matrix<double, 3, 1> mVel;         //解算的速度
    Eigen::Quaternion<double> mQua;           //解算的姿态

    Eigen::Matrix<double, 3, 1> mbw;
    Eigen::Matrix<double, 3, 1> mba;
    Eigen::Matrix<double, 3, 1> g;

    double mTime;
    double imuShiftX,imuShiftY,imuShiftZ;
    double imuVelocityX,imuVelocityY,imuVelocityZ;

};

template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
  }

template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatJPL(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (
        Eigen::Matrix<typename Derived::Scalar, 4, 4>() <<
        0, vec[2], -vec[1], vec[0],
        -vec[2], 0, vec[0], vec[1],
        vec[1], -vec[0], 0, vec[2],
        -vec[0], -vec[1], -vec[2], 0
        ).finished();
  }

}// namespace ORB_SLAM