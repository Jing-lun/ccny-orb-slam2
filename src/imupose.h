#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <vector>
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "imustate.h"
#include "Converter.h"
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace ORB_SLAM2
{

class Imuposeinf
{
private:

  ros::NodeHandle nh;
  ros::Subscriber imu_sub,orb_sub;
  ros::Publisher imupos_pub;
  nav_msgs::Odometry imu_pos;
public:

  Imuposeinf();
  void imuCallback(const double &timestamp, const Eigen::Quaterniond &q_imu, const Eigen::Matrix<double,3,1> &imu_w);
  Eigen::Quaternion<double> orbCallback(const double &timestamp, const Eigen::Quaterniond &qorb);
  Eigen::Matrix<double,3,3> Qua2Dcm(const Eigen::Quaternion<double> &q);
  Eigen::Quaternion<double> Dcm2Qua(const Eigen::Matrix<double,3,3> &dcm);

  Eigen::Matrix<double,3,1> mAdd_w;         // 对准时累加 加速度
  //Eigen::Matrix<double,3,1> mAdd_a;         // 对准时累加 线加速度

  //Eigen::Matrix<double,3,1> mAdd_mag;

  ImuState mImuInf_cur;                     // 当前状态值
  ImuState mImuInf_pre;                     // 上一状态值
  double mTimeCount;                        // 60秒计时
  bool mIMUOriCount;                        // IMU时间flag
  bool morbflag;                            // IMU时间flag
  bool mAlignOK;                            // 对准是否搞定 true搞定了

  double delta_T;
  double curorb_T;
  double preorb_T;
  double dorb_T;

  Eigen::Matrix<double,3,1> gn;               //重力g
  Eigen::Matrix<double,3,3> Cgni;             //重力到IMU系方向余弦矩阵

  //Eigen::Matrix<double,3,1> ba;                       //加表零漂
  Eigen::Matrix<double,3,1> bw;                 //陀螺零漂
  Eigen::Matrix<double,3,1> deltatheta;         //角度增量
  float deltatheta_mo;                          //角度增量的模
  Eigen::Matrix<double,3,1> deltav;             //角度增量
  float deltav_mo;                              //角度增量的模
  Eigen::Matrix<double,3,1> pesa;               //过渡量
  float pesa_mo;                                //过渡量的模

  Eigen::Matrix<double,3,1> q_bpart;             //过渡量
  Eigen::Quaternion<double> q_bpre;              //过渡量
  Eigen::Quaternion<double> q_pre;               //前一时刻姿态
  Eigen::Quaternion<double> q_now;               //当前解算姿态
  Eigen::Matrix<double,3,3> Cbn_pre;             //前一时刻方向余弦矩阵
  Eigen::Matrix<double,3,3> Cbn_now;             //当前时刻方向余弦矩阵

  Eigen::Matrix<double,3,1> d_Vgn;               //重力分量
  Eigen::Matrix<double,3,1> d_Vrot;              //过渡量
  // Eigen::Matrix<double,3,1> V_now;               //当前解算速度
  // Eigen::Matrix<double,3,1> V_pre;               //前一时刻速度

  // Eigen::Matrix<double,3,1> P_now;               //当前解算位置
  // Eigen::Matrix<double,3,1> P_pre;               //前一时刻位置

  //Eigen::Matrix<double,3,1> p_orb;               //orb输出的位置
  Eigen::Quaternion<double> q_orb;               //orb输出的姿态

  //Eigen::Matrix<double,3,3> Accxx;                //IMU加速度反对称阵
  Eigen::Matrix<double,3,3> Angxx;                //IMU角速度反对称阵
  Eigen::Matrix<double,3,3> Pcixx;                //位置反对称阵

  Eigen::Matrix<double,6,6> F_fai;
  Eigen::Matrix<double,6,6> Q_d;

  // Eigen::Matrix<double,3,3> n_a;                  //加计噪声
  // Eigen::Matrix<double,3,3> n_ba;                 //加计随机游走
  Eigen::Matrix<double,3,3> n_w;
  Eigen::Matrix<double,3,3> n_bw;

  //Eigen::Matrix<double,3,1> p_error;
  Eigen::Quaternion<double> q_e;                  //姿态误差过渡量
  Eigen::Matrix<double,3,1> q_error;
  Eigen::Matrix<double,3,1> p_ci;                 //相机与IMU之间的位置标定
  Eigen::Matrix<double,3,1> z_k;
  Eigen::Matrix<double,3,6> H_k;

  Eigen::Matrix<double,6,1> Xkpre;
  Eigen::Matrix<double,6,6> Pkpre;
  Eigen::Matrix<double,6,3> Kk;
  Eigen::Matrix<double,3,3> Rk;
  Eigen::Matrix<double,3,3> fai_ver;

};

}// namespace ORB_SLAM