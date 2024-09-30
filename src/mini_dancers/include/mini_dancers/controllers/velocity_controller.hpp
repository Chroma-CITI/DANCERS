#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include "pid.hpp"
#include "references.hpp"
#include "../multirotor_model.hpp"

namespace mrs_multirotor_simulator
{

class VelocityController {

public:
  struct Params
  {
    double kp               = 2.0;
    double kd               = 0.05;
    double ki               = 0.01;
    double max_acceleration = 4.0;  // m/s^2;
  };

  VelocityController();
  VelocityController(const MultirotorModel::ModelParams& model_params);

  void setParams(const Params& params);

  reference::AccelerationHdgRate getControlSignal(const MultirotorModel::State& state, const reference::VelocityHdgRate& reference, const double& dt);
  reference::AccelerationHdg     getControlSignal(const MultirotorModel::State& state, const reference::VelocityHdg& reference, const double& dt);

private:
  MultirotorModel::ModelParams model_params_;
  Params                       params_;

  void initializePIDs(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

/* VelocityController() //{ */

VelocityController::VelocityController() {
  initializePIDs();
}

VelocityController::VelocityController(const MultirotorModel::ModelParams& model_params) {
  model_params_ = model_params;

  initializePIDs();
}

//}

/* setParams() //{ */

void VelocityController::setParams(const Params& params) {

  params_ = params;

  initializePIDs();
}

//}

/* getControlSignal(const MultirotorModel::State& state, const reference::VelocityHdg& reference, const double& dt) //{ */

reference::AccelerationHdg VelocityController::getControlSignal(const MultirotorModel::State& state, const reference::VelocityHdg& reference,
                                                                const double& dt) {

  Eigen::Vector3d vel_error = reference.velocity - state.v;

  reference::AccelerationHdg output;

  output.acceleration(0) = pid_x_.update(vel_error(0), dt);
  output.acceleration(1) = pid_y_.update(vel_error(1), dt);
  output.acceleration(2) = pid_z_.update(vel_error(2), dt);

  output.heading = reference.heading;

  return output;
}

//}

/* getControlSignal(const MultirotorModel::State& state, const reference::VelocityHdgRate& reference, const double& dt) //{ */

reference::AccelerationHdgRate VelocityController::getControlSignal(const MultirotorModel::State& state, const reference::VelocityHdgRate& reference,
                                                                    const double& dt) {

  Eigen::Vector3d vel_error = reference.velocity - state.v;

  reference::AccelerationHdgRate output;

  output.acceleration(0) = pid_x_.update(vel_error(0), dt);
  output.acceleration(1) = pid_y_.update(vel_error(1), dt);
  output.acceleration(2) = pid_z_.update(vel_error(2), dt);

  output.heading_rate = reference.heading_rate;

  return output;
}

//}

// | ------------------------- private ------------------------ |

/* initializePIDs() //{ */

void VelocityController::initializePIDs(void) {

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params_.kp, params_.kd, params_.ki, params_.max_acceleration, 1.0);
  pid_y_.setParams(params_.kp, params_.kd, params_.ki, params_.max_acceleration, 1.0);
  pid_z_.setParams(params_.kp, params_.kd, params_.ki, params_.max_acceleration, 1.0);
}

//}

}  // namespace mrs_multirotor_simulator

#endif  // VELOCITY_CONTROLLER_H
