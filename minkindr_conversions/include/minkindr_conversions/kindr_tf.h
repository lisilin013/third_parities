#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kindr/minimal/quat-transformation.h>
#include <tf/transform_datatypes.h>

#include <glog/logging.h>
#include <tf_conversions/tf_eigen.h>

namespace minkindr_conversions {

// A wrapper for the relevant functions in eigen_conversions.
// tf::Quaternion amd kindr::minimal::RotationQuaternionTemplate
template <typename Scalar>
void quaternionKindrToTF(const kindr::minimal::RotationQuaternionTemplate<Scalar> &kindr,
                         tf::Quaternion *tf_type) {
  CHECK_NOTNULL(tf_type);
  tf::quaternionEigenToTF(kindr.toImplementation(), *tf_type);
}

template <typename Scalar>
void quaternionTFToKindr(const tf::Quaternion &tf_type,
                         kindr::minimal::RotationQuaternionTemplate<Scalar> *kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<Scalar> quat;
  tf::quaternionTFToEigen(tf_type, quat);
  *kindr = kindr::minimal::RotationQuaternionTemplate<Scalar>(quat);
}

// Also the Eigen implementation version of this.
// tf::Quaternion and Eigen::Quaternion
template <typename Scalar>
void quaternionKindrToTF(const Eigen::Quaternion<Scalar> &kindr, tf::Quaternion *tf_type) {
  CHECK_NOTNULL(tf_type);
  tf::quaternionEigenToTF(kindr, *tf_type);
}

template <typename Scalar>
void quaternionTFToKindr(const tf::Quaternion &tf_type, Eigen::Quaternion<Scalar> *kindr) {

  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond kindr_double;
  tf::quaternionTFToEigen(tf_type, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

// A wrapper for the relevant functions in eigen_conversions.
// tf::Vector3 and Eigen::Matrix.
template <typename Scalar>
void vectorKindrToTF(const Eigen::Matrix<Scalar, 3, 1> &kindr, tf::Vector3 *tf_type) {
  CHECK_NOTNULL(tf_type);
  tf::vectorEigenToTF(kindr, *tf_type);
}

template <typename Scalar>
void vectorTFToKindr(const tf::Vector3 &tf_type, Eigen::Matrix<Scalar, 3, 1> *kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<double, 3, 1> kindr_double;
  tf::vectorTFToEigen(tf_type, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

// Convert a kindr::minimal::QuatTransformation to a tf::Transform.
template <typename Scalar>
void transformKindrToTF(const kindr::minimal::QuatTransformationTemplate<Scalar> &kindr,
                        tf::Transform *tf_type) {
  CHECK_NOTNULL(tf_type);
  tf::Vector3 origin;
  tf::Quaternion rotation;
  vectorKindrToTF(kindr.getPosition(), &origin);
  quaternionKindrToTF(kindr.getRotation(), &rotation);
  tf_type->setOrigin(origin);
  tf_type->setRotation(rotation);
}

template <typename Scalar>
void transformTFToKindr(const tf::Transform &tf_type,
                        kindr::minimal::QuatTransformationTemplate<Scalar> *kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionTFToKindr(tf_type.getRotation(), &rotation);
  vectorTFToKindr(tf_type.getOrigin(), &position);

  // Enforce positive w.
  if (rotation.w() < 0) {
    rotation.coeffs() = -rotation.coeffs();
  }

  *kindr = kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

// Convert a kindr::minimal::QuatTransformation to a 6 DoF tf::Pose.
template <typename Scalar>
void poseKindrToTF(const kindr::minimal::QuatTransformationTemplate<Scalar> &kindr,
                   tf::Pose *tf_type) {
  transformKindrToTF(kindr, tf_type);
}

template <typename Scalar>
void poseTFToKindr(const tf::Pose &tf_type,
                   kindr::minimal::QuatTransformationTemplate<Scalar> *kindr) {
  transformTFToKindr(tf_type, kindr);
}

} // namespace minkindr_conversions
