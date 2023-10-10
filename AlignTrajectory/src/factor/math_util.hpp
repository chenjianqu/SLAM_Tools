
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>


inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }

inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

/// \brief Create a skew-symmetric matrix from a 3-element vector.
///         [  0 -w3  w2]
///   w  -> [ w3   0 -w1]
///         [-w2  w1   0]
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(
    const Eigen::MatrixBase<Derived> &w) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> w_hat;
  w_hat << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_hat;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> positify(
    const Eigen::QuaternionBase<Derived> &q) {
  // printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
  // Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
  // printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
  // return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename
  // Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
  return q;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(
    const Eigen::QuaternionBase<Derived> &q) {
  Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
  ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
  ans.template block<3, 1>(1, 0) = qq.vec(),
                              ans.template block<3, 3>(1, 1) =
                                  qq.w() *
                                      Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
                                  skewSymmetric(qq.vec());
  return ans;
}

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta) {
  typedef typename Derived::Scalar Scalar_t;
  Eigen::Quaternion<Scalar_t> dq;

  // Eigen::Matrix<typename Derived::Scalar, 3, 1> theta = theta_q;
  // typename Derived::Scalar theta_norm = theta.norm();
  // Eigen::Matrix<typename Derived::Scalar, 3, 1> a = theta.normalized();
  // Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetrica = skewSymmetric(a);
  // Eigen::Matrix<typename Derived::Scalar, 3, 3> I =
  //     Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
  // Eigen::Matrix<typename Derived::Scalar, 3, 3> expw = cos(theta_norm) * I +
  //                                                      (1 - cos(theta_norm)) * a * a.transpose()
  //                                                      + sin(theta_norm) * SkewSymmetrica;
  // dq = Eigen::Quaternion<typename Derived::Scalar>(expw);

  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

/// \brief transform a point p1 into another cordinate =>p2
// p2 = T_2_1 * p1
inline Eigen::Vector3d transform(const Eigen::Matrix4d &Tx_2_1, const Eigen::Vector3d &point,
                                 Eigen::Matrix<double, 3, 6> *J_this = nullptr,
                                 Eigen::Matrix3d *J_point = nullptr) {
  const Eigen::Matrix3d Rx_2_1 = Tx_2_1.block<3, 3>(0, 0);
  const Eigen::Vector3d tx_2_1 = Tx_2_1.block<3, 1>(0, 3);
  // J(1:3, 1:3) = dlog(R*p + t)/dw = -R * skew(p);
  // J(1:3, 4:6) = d(R*p + t)/dt = I3;
  if (J_this != nullptr) {
    J_this->block<3, 3>(0, 0) = (-1) * Rx_2_1 * skewSymmetric(point);
    J_this->block<3, 3>(0, 3).setIdentity();
  }
  // J(1:3, 1:3) = d(R*p + t)/dp = R;
  if (J_point != nullptr) {
    *J_point = Rx_2_1;
  }
  return Rx_2_1 * point + tx_2_1;
}

inline Eigen::Matrix<double, Eigen::Dynamic, 3> TransformPoints(
    const Eigen::Matrix<double, Eigen::Dynamic, 3> &points, const Eigen::Matrix4d &Tx_2_1) {
  const Eigen::Matrix3d Rx_2_1 = Tx_2_1.block<3, 3>(0, 0);
  const Eigen::Vector3d tx_2_1 = Tx_2_1.block<3, 1>(0, 3);
  return (Rx_2_1 * points.transpose()).transpose().rowwise() + tx_2_1.transpose();
  // return (points * Rx_2_1);
}

/// \brief Normalize the given quaternion to unit quaternion.
inline void quaternionNormalize(Eigen::Vector4d &q) {
  double norm = q.norm();
  q = q / norm;
  return;
}

/// \brief Perform q1 * q2
inline Eigen::Vector4d quaternionMultiplication(const Eigen::Vector4d &q1,
                                                const Eigen::Vector4d &q2) {
  Eigen::Matrix4d L;
  L << q1(3), q1(2), -q1(1), q1(0), -q1(2), q1(3), q1(0), q1(1), q1(1), -q1(0), q1(3), q1(2),
      -q1(0), -q1(1), -q1(2), q1(3);

  Eigen::Vector4d q = L * q2;
  quaternionNormalize(q);
  return q;
}

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
  Eigen::Vector3d ypr(3);
#if 0
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
#else
  double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  bool singular = sy < 1e-6;
  double r, p, y;
  if (!singular) {
    r = atan2(R(2, 1), R(2, 2));
    p = atan2(-R(2, 0), sy);
    y = atan2(R(1, 0), R(0, 0));
  } else {
    r = atan2(-R(1, 2), R(1, 1));
    p = atan2(-R(2, 0), sy);
    y = 0;
  }
#endif
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;
  return ypr / M_PI * 180.0;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr) {
  typedef typename Derived::Scalar Scalar_t;
  Scalar_t y = ypr(0) / 180.0 * M_PI;
  Scalar_t p = ypr(1) / 180.0 * M_PI;
  Scalar_t r = ypr(2) / 180.0 * M_PI;
  Eigen::AngleAxis<Scalar_t> pitch_(ypr(1) / 180.0 * M_PI, Eigen::Vector3d::UnitX());
  Eigen::AngleAxis<Scalar_t> roll_(ypr(2) / 180.0 * M_PI, Eigen::Vector3d::UnitY());
  Eigen::AngleAxis<Scalar_t> yaw_(ypr(0) / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Matrix<Scalar_t, 3, 3> rotation = (yaw_ * pitch_ * roll_).matrix();
  return rotation;
}

/*
 * @brief Convert the vector part of a quaternion to a
 *    full quaternion.
 * @note This function is useful to convert delta quaternion
 *    which is usually a 3x1 vector to a full quaternion.
 *    For more details, check Section 3.2 "Kalman Filter Update" in
 *    "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for quaternion Algebra".
 */
inline Eigen::Quaterniond getSmallAngleQuaternion(const Eigen::Vector3d &dtheta) {
  Eigen::Vector3d dq = dtheta / 2.0;
  Eigen::Quaterniond q;
  double dq_square_norm = dq.squaredNorm();

  if (dq_square_norm <= 1) {
    q.x() = dq(0);
    q.y() = dq(1);
    q.z() = dq(2);
    q.w() = std::sqrt(1 - dq_square_norm);
  } else {
    q.x() = dq(0);
    q.y() = dq(1);
    q.z() = dq(2);
    q.w() = 1;
    q.normalize();
  }

  return q;
}

