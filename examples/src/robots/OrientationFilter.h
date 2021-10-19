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

  void update (const Eigen::Vector3d& angVel, const Eigen::Vector3d& acc, double dt) {
    raisim::Vec<3> angV = rot_ * angVel;
    raisim::Vec<4> quat, quat2;
    angV *= dt;
    eulerVecToQuat(angV, quat);
    quatMul(&quat_[0], quat.ptr(), quat2.ptr());
    quat2 *= 1. / quat2.norm();
    quatToRotMat(quat2, rot_);

    if(acc.norm() < 1e-7) {
      rotMatToQuat(rot_, quat_);
      return;
    }

    raisim::Vec<3> accInWorld = rot_ * acc;
    raisim::Vec<3> negGaxis = accInWorld;
    negGaxis *= -1. / negGaxis.norm();

    double accAngle = asin(std::sqrt(negGaxis[0] * negGaxis[0] + negGaxis[1] * negGaxis[1]));

    if(accAngle < 1e-7) {
      rotMatToQuat(rot_, quat_);
      return;
    }

    Vec<3> rotAxis = {-negGaxis[1], negGaxis[0], 0};
    rotAxis *= 1./rotAxis.norm();
    Mat<3,3> correctionR;


    angleAxisToRotMat(rotAxis, accAngle, correctionR);

    Mat<3,3> temp;
    temp = correctionR.transpose() * rot_;
    rot_ = temp;

    rotMatToQuat(rot_, quat_);
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
