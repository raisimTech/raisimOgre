//
// Created by jemin on 10/19/21.
//

#ifndef RAISIMOGRE_EXAMPLES_SRC_ROBOTS_ORIENTATIONFILTER_H_
#define RAISIMOGRE_EXAMPLES_SRC_ROBOTS_ORIENTATIONFILTER_H_

#include "raisim/World.hpp"
#include <Eigen/Core>


namespace raisim {

class OrientationFilter {

 public:

  OrientationFilter() {
    rot_.setIdentity();
    quat_.setZero(); quat_[0] = 1.;
  }

  OrientationFilter(const Eigen::Matrix3d& initialR) : rot_(initialR) {
    rotMatToQuat(rot_, quat_);
  }

  /// angvel, acc in the body frame
  void update (const Eigen::Vector3d& angVel, const Eigen::Vector3d& acc, double dt) {
    raisim::Vec<3> angV = rot_ * angVel;
    raisim::Vec<4> quat, quat2;
    angV *= dt;
    eulerVecToQuat(angV, quat);
    quatMul(&quat_[0], quat.ptr(), quat2.ptr());
    quat2 *= 1. / quat2.norm();
    quatToRotMat(quat2, rot_);
    quat_ = quat2;

    if(acc.norm() < 1e-7) {
      rotMatToQuat(rot_, quat_);
      return;
    }

//    std::cout<<"rot_\n"<<rot_.e()<<std::endl;
    raisim::Vec<3> accInWorld = rot_.e() * acc;
//    std::cout<<"accInWorld\n"<<accInWorld.e().transpose()<<std::endl;
    accInWorld *= 1. / accInWorld.norm();

    double accAngle = asin(std::sqrt(accInWorld[0] * accInWorld[0] + accInWorld[1] * accInWorld[1]));
//    std::cout<<"accAngle\n"<<accAngle<<std::endl;

    if(accAngle < 1e-7) {
      rotMatToQuat(rot_, quat_);
      return;
    }

    Vec<3> rotAxis = {-accInWorld[1], accInWorld[0], 0.};
    rotAxis *= 1. / rotAxis.norm();
    Mat<3,3> correctionR;
//    std::cout<<"rotAxis\n"<<rotAxis.e().transpose()<<std::endl;

    angleAxisToRotMat(rotAxis, accAngle*0.000, correctionR);
//    std::cout<<"correctionR\n"<<correctionR.e()<<std::endl;

    Mat<3,3> temp;
    temp = correctionR.transpose() * rot_;
    rot_ = temp;

    rotMatToQuat(rot_, quat_);
    quat_ /= quat.norm();
    quatToRotMat(quat_, rot_);
  }

  void getRotationMatrix(Eigen::Matrix3d& rot) {
    rot = rot_.e();
  }

  void getQuaternion(Eigen::Vector4d& quat) {
    quat = quat_.e();
  }

 private:
  raisim::Mat<3,3> rot_;
  raisim::Vec<4> quat_;
};

}

#endif //RAISIMOGRE_EXAMPLES_SRC_ROBOTS_ORIENTATIONFILTER_H_
