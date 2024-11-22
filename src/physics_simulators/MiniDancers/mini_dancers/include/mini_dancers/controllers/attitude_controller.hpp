#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "pid.hpp"
#include "references.hpp"
#include "../multirotor_model.hpp"

namespace mrs_multirotor_simulator
{

class AttitudeController {

public:
  class Params {
  public:
    double kp                  = 6.0;
    double kd                  = 0.05;
    double ki                  = 0.01;
    double max_rate_roll_pitch = 10.0;  // rad/s
    double max_rate_yaw        = 1.0;   // rad/s
  };

  AttitudeController();
  AttitudeController(const MultirotorModel::ModelParams& model_params);

  void setParams(const Params& params);

  reference::AttitudeRate getControlSignal(const MultirotorModel::State& state, const reference::Attitude& reference, const double& dt);
  reference::AttitudeRate getControlSignal(const MultirotorModel::State& state, const reference::TiltHdgRate& reference, const double& dt);

private:
  Params                       params_;
  MultirotorModel::ModelParams model_params_;

  void initializePIDS(void);

  double intrinsicBodyRateToHeadingRate(const Eigen::Matrix3d& R, const Eigen::Vector3d& attitude_rate);
  double getYawRateIntrinsic(const Eigen::Matrix3d& R, const double& heading_rate);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

// --------------------------------------------------------------
// |                       implementation                       |
// --------------------------------------------------------------

/* AttitudeController() //{ */

AttitudeController::AttitudeController() {
  initializePIDS();
}

//}

/* AttitudeController() //{ */

AttitudeController::AttitudeController(const MultirotorModel::ModelParams& model_params) {
  model_params_ = model_params;
  initializePIDS();
}

//}

/* setParams() //{ */

void AttitudeController::setParams(const Params& params) {

  params_ = params;

  initializePIDS();
}

//}

/* getControlSignal(const MultirotorModel::State& state, const reference::Attitude& reference, const double& dt) //{ */

reference::AttitudeRate AttitudeController::getControlSignal(const MultirotorModel::State& state, const reference::Attitude& reference, const double& dt) {

  // orientation error
  Eigen::Matrix3d R_error = 0.5 * (reference.orientation.transpose() * state.R - state.R.transpose() * reference.orientation);

  // vectorize the orientation error
  // clang-format off
  Eigen::Vector3d R_error_vec;
  R_error_vec << (R_error(1, 2) - R_error(2, 1)) / 2.0,
                 (R_error(2, 0) - R_error(0, 2)) / 2.0,
                 (R_error(0, 1) - R_error(1, 0)) / 2.0;
  // clang-format on

  reference::AttitudeRate output;

  output.rate_x   = pid_x_.update(R_error_vec(0), dt);
  output.rate_y   = pid_y_.update(R_error_vec(1), dt);
  output.rate_z   = pid_z_.update(R_error_vec(2), dt);
  output.throttle = reference.throttle;

  return output;
}

//}

/* getControlSignal(const MultirotorModel::State& state, const reference::TiltHdgRate& reference, const double& dt) //{ */

reference::AttitudeRate AttitudeController::getControlSignal(const MultirotorModel::State& state, const reference::TiltHdgRate& reference, const double& dt) {

  Eigen::Matrix3d Rd = Eigen::Matrix3d::Zero();

  // construct the desired orientation matrix such that the desired heading is close the current heading
  Rd.col(2) = reference.tilt_vector.normalized();
  Rd.col(1) = Rd.col(2).cross(state.R.col(0));
  Rd.col(1).normalize();
  Rd.col(0) = Rd.col(1).cross(Rd.col(2));
  Rd.col(0).normalize();

  // orientation error
  Eigen::Matrix3d R_error = 0.5 * (Rd.transpose() * state.R - state.R.transpose() * Rd);

  // vectorize the orientation error
  // clang-format off
  Eigen::Vector3d R_error_vec;
  R_error_vec << (R_error(1, 2) - R_error(2, 1)) / 2.0,
                 (R_error(2, 0) - R_error(0, 2)) / 2.0,
                 (R_error(0, 1) - R_error(1, 0)) / 2.0;
  // clang-format on

  reference::AttitudeRate output;

  double rate_x = pid_x_.update(R_error_vec(0), dt);
  double rate_y = pid_y_.update(R_error_vec(1), dt);
  double rate_z = pid_z_.update(R_error_vec(2), dt);

  double parasitic_hdg_rate = intrinsicBodyRateToHeadingRate(state.R, Eigen::Vector3d(rate_x, rate_y, rate_z));

  rate_z += getYawRateIntrinsic(state.R, reference.heading_rate - parasitic_hdg_rate);

  output.rate_x = rate_x;
  output.rate_y = rate_y;
  output.rate_z = rate_z;

  output.throttle = reference.throttle;

  return output;
}

//}

// | ------------------------- private ------------------------ |

/* signum() //{ */

template <typename T>
int signum(T val) {
  return (T(0) < val) - (val < T(0));
}

//}

/* initializePIDS() //{ */

void AttitudeController::initializePIDS(void) {

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params_.kp, params_.kd, params_.ki, params_.max_rate_roll_pitch, 0.1);
  pid_y_.setParams(params_.kp, params_.kd, params_.ki, params_.max_rate_roll_pitch, 0.1);
  pid_z_.setParams(params_.kp, params_.kd, params_.ki, params_.max_rate_yaw, 0.1);
}

//}

/* intrinsicBodyRateToHeadingRate() //{ */

double AttitudeController::intrinsicBodyRateToHeadingRate(const Eigen::Matrix3d& R, const Eigen::Vector3d& w) {

  // create the angular velocity tensor
  Eigen::Matrix3d W;
  W << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  // R derivative
  Eigen::Matrix3d R_d = R * W;

  // atan2 derivative
  double rx = R(0, 0);  // x-component of body X
  double ry = R(1, 0);  // y-component of body Y

  double denom = rx * rx + ry * ry;

  double atan2_d_x = 0;
  double atan2_d_y = 0;

  if (fabs(denom) <= 1e-5) {
    std::cout << "[AttitudeController]: denominator in intrinsicBodyRateToHeadingRate() close to zero" << std::endl;
  } else {
    atan2_d_x = -ry / denom;
    atan2_d_y = rx / denom;
  }

  // atan2 total differential
  double heading_rate = atan2_d_x * R_d(0, 0) + atan2_d_y * R_d(1, 0);

  return heading_rate;
}

//}

/* getYawRateIntrinsic() //{ */

double AttitudeController::getYawRateIntrinsic(const Eigen::Matrix3d& R, const double& heading_rate) {

  // when the heading rate is very small, it does not make sense to compute the
  // yaw rate (the math would break), return 0
  if (fabs(heading_rate) < 1e-3) {
    return 0;
  }

  // construct the heading orbital velocity vector
  Eigen::Vector3d heading_vector   = Eigen::Vector3d(R(0, 0), R(1, 0), 0);
  Eigen::Vector3d orbital_velocity = Eigen::Vector3d(0, 0, heading_rate).cross(heading_vector);

  // projector to the heading orbital velocity vector subspace
  Eigen::Vector3d b_orb = Eigen::Vector3d(0, 0, 1).cross(heading_vector);
  b_orb.normalize();
  Eigen::Matrix3d P = b_orb * b_orb.transpose();

  // project the body yaw orbital velocity vector base onto the heading orbital velocity vector subspace
  Eigen::Vector3d projected = P * R.col(1);

  double orbital_velocity_norm = orbital_velocity.norm();
  double projected_norm        = projected.norm();

  if (fabs(projected_norm) < 1e-5) {
    std::cout << "[AttitudeController]: getYawRateIntrinsic(): \"projected_norm\" in denominator is almost zero!!!" << std::endl;
    return 0;
  }

  double direction = signum(orbital_velocity.dot(projected));

  double output_yaw_rate = direction * (orbital_velocity_norm / projected_norm);

  if (!std::isfinite(output_yaw_rate)) {
    std::cout << "[AttitudeController]: getYawRateIntrinsic(): NaN detected in variable \"output_yaw_rate\"!!!" << std::endl;
    return 0;
  }

  // extract the yaw rate
  return output_yaw_rate;
}

//}

}  // namespace mrs_multirotor_simulator

#endif  // ATTITUDE_CONTROLLER_H
