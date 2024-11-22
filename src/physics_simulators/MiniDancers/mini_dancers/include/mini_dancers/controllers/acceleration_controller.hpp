#ifndef ACCELERATION_CONTROLLER_H
#define ACCELERATION_CONTROLLER_H

#include "references.hpp"
#include "../multirotor_model.hpp"

namespace mrs_multirotor_simulator
{

class AccelerationController {

public:
  AccelerationController();
  AccelerationController(const MultirotorModel::ModelParams& model_params);

  reference::TiltHdgRate getControlSignal(const MultirotorModel::State& state, const reference::AccelerationHdgRate& reference, const double dt);
  reference::Attitude    getControlSignal(const MultirotorModel::State& state, const reference::AccelerationHdg& reference, const double dt);

private:
  MultirotorModel::ModelParams model_params_;
};

// --------------------------------------------------------------
// |                       implementation                       |
// --------------------------------------------------------------

/* AccelerationController() //{ */

AccelerationController::AccelerationController() {
}

//}

/* AccelerationController() //{ */

AccelerationController::AccelerationController(const MultirotorModel::ModelParams& model_params) {
  model_params_ = model_params;
}

//}

/* getControlSignal(const MultirotorModel::State& state, const reference::AccelerationHdg& reference, const double dt) //{ */

reference::Attitude AccelerationController::getControlSignal(const MultirotorModel::State& state, const reference::AccelerationHdg& reference,
                                                             [[maybe_unused]] const double dt) {

  const Eigen::Vector3d fd      = (reference.acceleration + Eigen::Vector3d(0, 0, model_params_.g)) * model_params_.mass;
  const Eigen::Vector3d fd_norm = fd.normalized();

  const Eigen::Vector3d bxd(cos(reference.heading), sin(reference.heading), 0.0);

  Eigen::Matrix3d Rd;

  // | ------------------------- body z ------------------------- |
  Rd.col(2) = fd_norm;

  // | ------------------------- body x ------------------------- |

  // construct the oblique projection
  Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - fd_norm * fd_norm.transpose());

  // create a basis of the body-z complement subspace
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
  A.col(0)          = projector_body_z_compl.col(0);
  A.col(1)          = projector_body_z_compl.col(1);

  // create the basis of the projection null-space complement
  Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
  B.col(0)          = Eigen::Vector3d(1, 0, 0);
  B.col(1)          = Eigen::Vector3d(0, 1, 0);

  // oblique projector to <range_basis>
  Eigen::MatrixXd Bt_A               = B.transpose() * A;
  Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
  Eigen::MatrixXd oblique_projector  = A * Bt_A_pseudoinverse * B.transpose();

  Rd.col(0) = oblique_projector * bxd;
  Rd.col(0).normalize();

  // | ------------------------- body y ------------------------- |

  Rd.col(1) = Rd.col(2).cross(Rd.col(0));
  Rd.col(1).normalize();

  // | ------------------------- result ------------------------- |

  reference::Attitude output;

  output.orientation = Rd;

  double thrust_force = fd.dot(state.R.col(2));

  output.throttle =
      (sqrt(thrust_force / (model_params_.kf * model_params_.n_motors)) - model_params_.min_rpm) / (model_params_.max_rpm - model_params_.min_rpm);

  return output;
}

//}

/* getControlSignal(const MultirotorModel::State& state, const reference::AccelerationHdg& reference, const double dt) //{ */

reference::TiltHdgRate AccelerationController::getControlSignal(const MultirotorModel::State& state, const reference::AccelerationHdgRate& reference,
                                                                [[maybe_unused]] const double dt) {

  const Eigen::Vector3d fd      = (reference.acceleration + Eigen::Vector3d(0, 0, model_params_.g)) * model_params_.mass;
  const Eigen::Vector3d fd_norm = fd.normalized();

  // | ------------------------- result ------------------------- |

  reference::TiltHdgRate output;

  output.tilt_vector  = fd_norm;
  output.heading_rate = reference.heading_rate;

  double thrust_force = fd.dot(state.R.col(2));

  output.throttle =
      (sqrt(thrust_force / (model_params_.kf * model_params_.n_motors)) - model_params_.min_rpm) / (model_params_.max_rpm - model_params_.min_rpm);

  return output;
}

//}

}  // namespace mrs_multirotor_simulator

#endif  // ACCELERATION_CONTROLLER_H
