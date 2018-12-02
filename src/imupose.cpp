#include "imupose.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <iostream>
using namespace std;

namespace ORB_SLAM2
{

Imuposeinf::Imuposeinf()
{
    //变量初始化
    mAdd_w << 0,0,0;
    //mAdd_a << 0,0,0;
    mTimeCount = 0.0;
    mIMUOriCount = false;
    morbflag = false;
    gn << 0, 0, -9.7919;
    //ba << 0.01, 0.01, 0.01;
    bw << 0.001, 0.001, 0.001;
    q_pre.w() = 1.0;
    q_pre.x() = 0.0;
    q_pre.y() = 0.0;
    q_pre.z() = 0.0;
    Cbn_pre << 1,0,0,0,1,0,0,0,1;

    q_now.w() = 1.0;
    q_now.x() = 0.0;
    q_now.y() = 0.0;
    q_now.z() = 0.0;
    // V_pre << 0, 0, 0;
    // V_now << 0, 0, 0;
    // P_pre << 0, 0, 0;
    // P_now << 0, 0, 0;

    // n_a << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
    // n_ba << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
    n_w << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
    n_bw << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
    p_ci << -0.105, 0, -0.01;
    Rk << 0.05,0,0, 0,0.05,0, 0,0,0.05;
    Xkpre.setZero();
    Kk.setZero();
    mAlignOK = false;
    Pkpre << 1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0,
             0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1;
    imupos_pub = nh.advertise<nav_msgs::Odometry>("/imu/Pose", 50);
}

Eigen::Matrix<double,3,3> Imuposeinf::Qua2Dcm(const Eigen::Quaternion<double> &q)
{
    Eigen::Matrix<double,3,3> dcm;
    dcm(0,0) = q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z();
    dcm(0,1) = 2*(q.x()*q.y()-q.w()*q.z());
    dcm(0,2) = 2*(q.x()*q.z()+q.w()*q.y());
    dcm(1,0) = 2*(q.x()*q.y()+q.w()*q.z());
    dcm(1,1) = q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z();
    dcm(1,2) = 2*(q.y()*q.z()-q.w()*q.x());
    dcm(2,0) = 2*(q.x()*q.z()-q.w()*q.y());
    dcm(2,1) = 2*(q.y()*q.z()+q.w()*q.x());
    dcm(2,2) = q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z();
    return dcm;
}

Eigen::Quaternion<double> Imuposeinf::Dcm2Qua(const Eigen::Matrix<double,3,3> &dcm)
{
    Eigen::Quaternion<double> q;
    q.w() = sqrt(fabs(1 + dcm(0,0) + dcm(1,1) + dcm(2,2)))/2;
    q.x() = (dcm(2,1) - dcm(1,2))/(4*q.w());
    q.y() = (dcm(0,2) - dcm(2,0))/(4*q.w());
    q.z() = (dcm(1,0) - dcm(0,1))/(4*q.w());
    return q;
}

void Imuposeinf::imuCallback(const double &timestamp, const Eigen::Quaterniond &q_imu, const Eigen::Matrix<double,3,1> &imu_w)
{
      mImuInf_cur.mTime = timestamp;

      mImuInf_cur.mImu_w = imu_w;
      //mImuInf_cur.mImu_a = imu_a;
      mImuInf_cur.mImu_ori = q_imu;

      delta_T = mImuInf_cur.mTime - mImuInf_pre.mTime;

      if(mIMUOriCount == false)//初始化为false
      {
          Cgni = mImuInf_cur.mImu_ori.toRotationMatrix();//将四元数转换为旋转矩阵
          gn = Cgni * gn;//gn为重力加速度，得到重力加速度在IMU体坐标系的分量
          //q_pre = mImu_ori;
          delta_T = 0.067;
          mIMUOriCount = true;
      }
      
      deltatheta = (mImuInf_cur.mImu_w - bw)*delta_T;
      deltatheta_mo = sqrt(deltatheta(0)*deltatheta(0)+deltatheta(1)*deltatheta(1)+deltatheta(2)*deltatheta(2));

    //   deltav = (mImuInf_cur.mImu_a - ba)*delta_T;//速度增量
    //   deltav_mo = sqrt(deltav(0)*deltav(0) + deltav(1)*deltav(1) + deltav(2)*deltav(2));

      //姿态解算
      q_bpart = (sin(deltatheta_mo/2)/deltatheta_mo)*deltatheta;
      q_bpre.w() = cos(deltatheta_mo/2);
      q_bpre.x() = q_bpart(0);
      q_bpre.y() = q_bpart(1);
      q_bpre.z() = q_bpart(2);

      //q_now = q_pre * q_bpre;
      //q_now = q_bpre * q_pre;
      //q_now = q_now.normalized();//归一化
      q_now = q_imu;                 //这里直接赋值了imu传出的四元数数据，因为用位姿解算的结果不太对。

    //   tf::Quaternion orientation;    //quaternion为四元数
    //   orientation.setW(q_now.w());
    //   orientation.setX(q_now.x());
    //   orientation.setY(q_now.y());
    //   orientation.setZ(q_now.z());
    //   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);//get the matrix represented as roll pitch and yaw about fixed axes XYZ.
    //   //相对imu体坐标系


      //位置解算
      //P_now = P_pre + (V_now + V_pre)/2*delta_T;

      //更新
      q_pre = q_now;
    //   V_pre = V_now;
    //   P_pre = P_now;
      Cbn_pre = q_pre.toRotationMatrix();

      vector<float> eulr_orb_qua = Converter::toEulr(q_pre);

            ofstream outfile2("//home//robooster//Desktop//imucallback.txt", ios_base::app);
            outfile2.setf(ios::fixed);
            outfile2<< fixed << setprecision(5) << std::left << std::setw(18)<<ros::Time::now().toSec();
            #if 1
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.x();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.y();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.z();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.w();
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[0];
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[1];
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[2];
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[0]*180/3.1415926;
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[1]*180/3.1415926;
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[2]*180/3.1415926<<endl;
            #endif
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<P_pre.x();
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<P_pre.y();
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<P_pre.z()<<endl;

      //mImuInf_cur.mPos = P_pre;
      mImuInf_cur.mQua = q_pre;
      mImuInf_pre = mImuInf_cur;
}

Eigen::Quaternion<double> Imuposeinf::orbCallback(const double &timestamp, const Eigen::Quaterniond &qorb)
{
    curorb_T = timestamp;
    //p_orb = pose;
    q_orb = qorb;
    dorb_T = curorb_T - preorb_T;
    if(morbflag == false)
    {
        dorb_T = 0.667;
        morbflag =true;
        //P_now << 0,0,0;
    }

    //Accxx = skew(deltav/delta_T);
    Angxx = skew(deltatheta/delta_T);
    Eigen::Matrix<double,3,3> I3;
    Eigen::Matrix<double,6,6> I6;
    Eigen::Matrix<double,3,3> B,F;
    I3.setIdentity();
    I6.setIdentity();

    // A = -0.5 * Cbn_pre * Accxx * dorb_T * dorb_T;
    B = I3 - Angxx * dorb_T + 0.5 * Angxx * Angxx * dorb_T * dorb_T;
    // C = -Cbn_pre * Accxx * dorb_T + 0.5 * Cbn_pre * Accxx * Angxx * dorb_T * dorb_T;
    // D = -0.5 * Cbn_pre * dorb_T * dorb_T;
    // E = -Cbn_pre * dorb_T;
    F = -I3 * dorb_T + 0.5 * Angxx * dorb_T * dorb_T;
    // G = 0.5 * Cbn_pre * Accxx * dorb_T * dorb_T;

    F_fai.setIdentity();
    // F_fai.block<3, 3> (0, 3) = A;
    // F_fai.block<3, 3> (0, 6) = I3 * dorb_T;
    // F_fai.block<3, 3> (0, 9) = D;
    F_fai.block<3, 3> (3, 3) = B;
    F_fai.block<3, 3> (3, 12) = F;
    // F_fai.block<3, 3> (6, 3) = C;
    // F_fai.block<3, 3> (6, 9) = E;
    // F_fai.block<3, 3> (6, 12) = G;

    Q_d.setZero();

    Eigen::Matrix<double,3,3> Cnb_pre = Cbn_pre.transpose();
    double tt = dorb_T * dorb_T;
    double ttt = dorb_T * dorb_T * dorb_T;
    double tttt = dorb_T * dorb_T * dorb_T * dorb_T;

    // Q_d.block<3, 3> (0, 0) = n_a * ttt/3;
    // Q_d.block<3, 3> (0, 3) = -Cbn_pre * Accxx * n_w * ttt/6;
    // Q_d.block<3, 3> (0, 6) = n_a * tt/2;
    // Q_d.block<3, 3> (0, 9) = -Cbn_pre * n_ba * ttt/6;
    // Q_d.block<3, 3> (3, 0) = Accxx * Cnb_pre * n_w * ttt/6 + Cnb_pre * n_bw * ttt/3;
    Q_d.block<3, 3> (0, 0) = n_w * dorb_T + n_bw * ttt/3;
    //Q_d.block<3, 3> (3, 6) = Accxx * Cnb_pre * n_w * tt/2 - Angxx * Accxx * Cnb_pre * n_w * ttt/6;
    Q_d.block<3, 3> (0, 3) = -n_bw * tt/2 + Angxx * n_bw * ttt/6;
    // Q_d.block<3, 3> (6, 0) = n_a * tt/2;
    // Q_d.block<3, 3> (6, 3) = -Cbn_pre * Accxx * n_bw * tttt/8;
    // Q_d.block<3, 3> (6, 6) = n_a * dorb_T + n_ba * ttt/3;
    // Q_d.block<3, 3> (6, 9) = -n_ba * Cbn_pre * tt/2;
    // Q_d.block<3, 3> (6, 12) = Cbn_pre * Accxx * n_bw * ttt/6;
    // Q_d.block<3, 3> (9, 0) = -Cnb_pre * n_ba * tt/6;
    // Q_d.block<3, 3> (9, 6) = -Cnb_pre * n_ba * tt/2;
    // Q_d.block<3, 3> (9, 9) = n_ba * dorb_T;
    // Q_d.block<3, 3> (12, 0) = -Cnb_pre * n_bw * tt/2;
    Q_d.block<3, 3> (3, 0) = -n_bw * tt/2 - Angxx * n_bw * ttt/6;
    //Q_d.block<3, 3> (12, 6) = -Accxx * Cnb_pre * n_bw * ttt/6;
    Q_d.block<3, 3> (3, 3) = n_bw * dorb_T;

    //p_error = p_orb - P_now - Cbn_pre * p_ci;
    q_e = q_now.conjugate() * q_orb;
    q_error = q_e.vec()/q_e.w()*2;
    z_k << q_error;

    Pcixx = skew(p_ci);
    H_k.setZero();
    H_k.block<3, 3> (0, 0) = I3;
    // H_k.block<3, 3> (0, 3) = -Cbn_pre * Pcixx;
    // H_k.block<3, 3> (3, 3) = I3;

    Xkpre = F_fai * Xkpre;
    Pkpre = F_fai * Pkpre * F_fai.transpose() + Q_d;
    Kk = Pkpre * H_k.transpose() * (H_k * Pkpre * H_k.transpose() + Rk).inverse();
    Xkpre = Xkpre + Kk * (z_k - H_k * Xkpre);
    Pkpre = (I6 - Kk * H_k) * Pkpre * (I6 - Kk * H_k).transpose() + Kk * Rk * Kk.transpose();

    fai_ver << 0, -Xkpre(2), Xkpre(1), Xkpre(2), 0, -Xkpre(0), -Xkpre(1), Xkpre(0), 0;

    Cbn_now = Cbn_pre * (I3 + fai_ver);
    // P_now = P_now + Xkpre.block<3, 1> (0, 0);
    // V_now = V_now + Xkpre.block<3, 1> (6, 0);
    // ba = ba + Xkpre.block<3, 1> (9, 0);
    bw = bw + Xkpre.block<3, 1> (3, 0);
    Xkpre.setZero();

    Cbn_pre = Cbn_now;
    q_now = Dcm2Qua(Cbn_pre);
    q_now = q_now.normalized();

    q_pre = q_now;
    // V_pre = V_now;
    // P_pre = P_now;

    imu_pos.pose.pose.orientation.x = q_pre.x();
    imu_pos.pose.pose.orientation.y = q_pre.y();
    imu_pos.pose.pose.orientation.z = q_pre.z();
    imu_pos.pose.pose.orientation.w = q_pre.w();
    // imu_pos.pose.pose.position.x = P_pre.x();
    // imu_pos.pose.pose.position.y = P_pre.y();
    // imu_pos.pose.pose.position.z = P_pre.z();
    imupos_pub.publish(imu_pos);

    vector<float> eulr_orb_qua = Converter::toEulr(q_pre);

            ofstream outfile2("//home//robooster//Desktop//orbcallback.txt", ios_base::app);
            outfile2.setf(ios::fixed);
            outfile2<< fixed << setprecision(5) << std::left << std::setw(18)<<ros::Time::now().toSec();
            #if 1
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.x();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.y();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.z();
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<q_pre.w();
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[0];
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[1];
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[2];
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[0]*180/3.1415926;
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[1]*180/3.1415926;
            outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<eulr_orb_qua[2]*180/3.1415926<<endl;
            #endif
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<P_pre.x();
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<P_pre.y();
            // outfile2<< fixed << setprecision(8) << std::left << std::setw(15)<<P_pre.z()<<endl;

    //mImuInf_cur.mPos = P_pre;
    mImuInf_cur.mQua = q_pre;
    mImuInf_pre = mImuInf_cur;

    preorb_T = curorb_T;

    return q_pre;
}

}// namespace ORB_SLAM



