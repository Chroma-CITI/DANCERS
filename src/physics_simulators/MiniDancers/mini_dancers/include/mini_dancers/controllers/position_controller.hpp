#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "pid.hpp"
#include "references.hpp"
#include "../multirotor_model.hpp"

namespace mrs_multirotor_simulator
{

class PositionController {

public:
  class Params {
  public:
    double kp           = 2.0;
    double kd           = 0.15;
    double ki           = 0.2;
    double max_velocity = 6.0;  // m/s
  };

  PositionController();
  PositionController(const MultirotorModel::ModelParams& model_params);

  void setParams(const Params& params);

  reference::VelocityHdg getControlSignal(const MultirotorModel::State& state, const reference::Position& reference, const double& dt);

private:
  MultirotorModel::ModelParams model_params_;
  Params                       params_;

  void initializePIDs(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

// --------------------------------------------------------------
// |                       implementation                       |
// --------------------------------------------------------------

/* PositionController() //{ */

PositionController::PositionController() {

  initializePIDs();
}

PositionController::PositionController(const MultirotorModel::ModelParams& model_params) {

  model_params_ = model_params;

  initializePIDs();
}

//}

/* setParams() //{ */

void PositionController::setParams(const Params& params) {

  params_ = params;

  initializePIDs();
}

//}

/* getControlSignal() //{ */

reference::VelocityHdg PositionController::getControlSignal(const MultirotorModel::State& state, const reference::Position& reference, const double& dt) {

  Eigen::Vector3d pos_error = reference.position - state.x;

  reference::VelocityHdg output;

  output.velocity(0) = pid_x_.update(pos_error(0), dt);
  output.velocity(1) = pid_y_.update(pos_error(1), dt);
  output.velocity(2) = pid_z_.update(pos_error(2), dt);

  output.heading = reference.heading;

  return output;
}

//}

// | ------------------------- private ------------------------ |

/* initializedPIDs() //{ */

void PositionController::initializePIDs(void) {

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params_.kp, params_.kd, params_.ki, params_.max_velocity, 1.0);
  pid_y_.setParams(params_.kp, params_.kd, params_.ki, params_.max_velocity, 1.0);
  pid_z_.setParams(params_.kp, params_.kd, params_.ki, params_.max_velocity, 1.0);
}

//}

}  // namespace mrs_multirotor_simulator

#endif  // POSITION_CONTROLLER_H
