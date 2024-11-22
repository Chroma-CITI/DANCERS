#ifndef UAV_SYSTEM_H
#define UAV_SYSTEM_H

#include "multirotor_model.hpp"

#include "controllers/mixer.hpp"
#include "controllers/rate_controller.hpp"
#include "controllers/attitude_controller.hpp"
#include "controllers/acceleration_controller.hpp"
#include "controllers/velocity_controller.hpp"
#include "controllers/position_controller.hpp"

namespace mrs_multirotor_simulator
{

class UavSystem {

public:
  enum INPUT_MODE
  {
    INPUT_UNKNOWN,
    ACTUATOR_CMD,
    CONTROL_GROUP_CMD,
    ATTITUDE_RATE_CMD,
    ATTITUDE_CMD,
    TILT_HDG_RATE_CMD,
    ACCELERATION_HDG_RATE_CMD,
    ACCELERATION_HDG_CMD,
    VELOCITY_HDG_RATE_CMD,
    VELOCITY_HDG_CMD,
    POSITION_CMD,
  };

  UavSystem(void);
  UavSystem(const MultirotorModel::ModelParams& model_params);
  UavSystem(const MultirotorModel::ModelParams& model_params, const Eigen::Vector3d spawn_pos, const double spawn_heading);

  void makeStep(const double dt);

  void crash(void);
  bool hasCrashed(void);

  void applyForce(const Eigen::Vector3d& force);

  void setInput(const reference::Actuators& actuators);
  void setInput(const reference::ControlGroup& control_group);
  void setInput(const reference::AttitudeRate& attitude_rate);
  void setInput(const reference::Attitude& attitude);
  void setInput(const reference::TiltHdgRate& tilt);
  void setInput(const reference::AccelerationHdgRate& acceleration);
  void setInput(const reference::AccelerationHdg& acceleration);
  void setInput(const reference::VelocityHdgRate& velocity);
  void setInput(const reference::VelocityHdg& velocity);
  void setInput(const reference::Position& position);
  void setInput(void);

  void setFeedforward(const reference::AccelerationHdgRate& cmd);
  void setFeedforward(const reference::AccelerationHdg& cmd);
  void setFeedforward(const reference::VelocityHdg& cmd);
  void setFeedforward(const reference::VelocityHdgRate& cmd);

  MultirotorModel::State       getState(void);
  MultirotorModel::ModelParams getParams(void);

  void setParams(const MultirotorModel::ModelParams& params);

  Eigen::Vector3d getImuAcceleration(void);

  void setMixerParams(const Mixer::Params& params);
  void setRateControllerParams(const RateController::Params& params);
  void setAttitudeControllerParams(const AttitudeController::Params& params);
  void setVelocityControllerParams(const VelocityController::Params& params);
  void setPositionControllerParams(const PositionController::Params& params);

  Eigen::MatrixXd getMixerAllocation(void);

private:
  // | ------------------------- basics ------------------------- |

  bool crashed_ = false;

  // | --------------------- dynamics model --------------------- |

  MultirotorModel multirotor_model_;

  // | ----------------------- controllers ---------------------- |

  Mixer                  mixer_;
  RateController         rate_controller_;
  AttitudeController     attitude_controller_;
  AccelerationController acceleration_controller_;
  VelocityController     velocity_controller_;
  PositionController     position_controller_;

  INPUT_MODE active_input_ = INPUT_UNKNOWN;

  // | ------------- inputs for feedback controllers ------------ |

  reference::Actuators           actuators_cmd_;
  reference::ControlGroup        control_group_cmd_;
  reference::AttitudeRate        attitude_rate_cmd_;
  reference::Attitude            attitude_cmd_;
  reference::TiltHdgRate         tilt_hdg_rate_cmd_;
  reference::AccelerationHdgRate acceleration_hdg_rate_cmd_;
  reference::AccelerationHdg     acceleration_hdg_cmd_;
  reference::VelocityHdgRate     velocity_hdg_rate_cmd_;
  reference::VelocityHdg         velocity_hdg_cmd_;
  reference::Position            position_cmd_;

  // | ------------------- feedforward inputs ------------------- |

  std::optional<reference::VelocityHdgRate>     velocity_hdg_rate_ff_;
  std::optional<reference::VelocityHdg>         velocity_hdg_ff_;
  std::optional<reference::AccelerationHdgRate> acceleration_hdg_rate_ff_;
  std::optional<reference::AccelerationHdg>     acceleration_hdg_ff_;

  void initializeControllers(void);
};

// --------------------------------------------------------------
// |                       implementation                       |
// --------------------------------------------------------------

/* UavSystem() //{ */

// constructor
UavSystem::UavSystem(void) {

  // the model params are supplied by the defaults from the header

  initializeControllers();
}

// constructor
UavSystem::UavSystem(const MultirotorModel::ModelParams& model_params) {

  multirotor_model_.setParams(model_params);

  multirotor_model_.initializeState();

  initializeControllers();
}

UavSystem::UavSystem(const MultirotorModel::ModelParams& model_params, const Eigen::Vector3d spawn_pos, const double spawn_heading) {

  multirotor_model_.setParams(model_params);

  multirotor_model_.initializeState();

  multirotor_model_.setStatePos(spawn_pos, spawn_heading);

  initializeControllers();
}

//}

/* initializeControllers() //{ */

void UavSystem::initializeControllers(void) {

  MultirotorModel::ModelParams model_params = multirotor_model_.getParams();

  mixer_                   = Mixer(model_params);
  rate_controller_         = RateController(model_params);
  attitude_controller_     = AttitudeController(model_params);
  acceleration_controller_ = AccelerationController(model_params);
  velocity_controller_     = VelocityController(model_params);
  position_controller_     = PositionController(model_params);
}

//}

/* setInput() //{ */

void UavSystem::setInput(const reference::Actuators& cmd) {

  actuators_cmd_ = cmd;

  active_input_ = ACTUATOR_CMD;
}

void UavSystem::setInput(const reference::ControlGroup& cmd) {

  control_group_cmd_ = cmd;

  active_input_ = CONTROL_GROUP_CMD;
}

void UavSystem::setInput(const reference::AttitudeRate& cmd) {

  attitude_rate_cmd_ = cmd;

  active_input_ = ATTITUDE_RATE_CMD;
}

void UavSystem::setInput(const reference::Attitude& cmd) {

  attitude_cmd_ = cmd;

  active_input_ = ATTITUDE_CMD;
}

void UavSystem::setInput(const reference::TiltHdgRate& cmd) {

  tilt_hdg_rate_cmd_ = cmd;

  active_input_ = TILT_HDG_RATE_CMD;
}

void UavSystem::setInput(const reference::AccelerationHdgRate& cmd) {

  acceleration_hdg_rate_cmd_ = cmd;

  active_input_ = ACCELERATION_HDG_RATE_CMD;
}

void UavSystem::setInput(const reference::AccelerationHdg& cmd) {

  acceleration_hdg_cmd_ = cmd;

  active_input_ = ACCELERATION_HDG_CMD;
}

void UavSystem::setInput(const reference::VelocityHdgRate& cmd) {

  velocity_hdg_rate_cmd_ = cmd;

  active_input_ = VELOCITY_HDG_RATE_CMD;
}

void UavSystem::setInput(const reference::VelocityHdg& cmd) {

  velocity_hdg_cmd_ = cmd;

  active_input_ = VELOCITY_HDG_CMD;
}

void UavSystem::setInput(const reference::Position& cmd) {

  position_cmd_ = cmd;

  active_input_ = POSITION_CMD;
}

void UavSystem::setInput(void) {

  active_input_ = INPUT_UNKNOWN;
}

//}

/* setFeedforward() //{ */

void UavSystem::setFeedforward(const reference::AccelerationHdgRate& cmd) {

  acceleration_hdg_rate_ff_ = cmd;
}

void UavSystem::setFeedforward(const reference::AccelerationHdg& cmd) {

  acceleration_hdg_ff_ = cmd;
}

void UavSystem::setFeedforward(const reference::VelocityHdgRate& cmd) {

  velocity_hdg_rate_ff_ = cmd;
}

void UavSystem::setFeedforward(const reference::VelocityHdg& cmd) {

  velocity_hdg_ff_ = cmd;
}

//}

/* crash() //{ */

void UavSystem::crash(void) {
  crashed_ = true;
}

//}

/* hasCrashed() //{ */

bool UavSystem::hasCrashed(void) {

  return crashed_;
}

//}

/* applyForce() //{ */

void UavSystem::applyForce(const Eigen::Vector3d& force) {

  multirotor_model_.applyForce(force);
}

//}

/* makeStep() //{ */

void UavSystem::makeStep(const double dt) {

  INPUT_MODE active_input = active_input_;

  if (crashed_ || active_input_ == UavSystem::INPUT_UNKNOWN) {

    actuators_cmd_.motors = Eigen::VectorXd::Zero(multirotor_model_.getParams().n_motors);

  } else {

    if (active_input == UavSystem::POSITION_CMD) {
      velocity_hdg_cmd_ = position_controller_.getControlSignal(multirotor_model_.getState(), position_cmd_, dt);
      active_input      = VELOCITY_HDG_CMD;

      if (velocity_hdg_ff_) {
        velocity_hdg_cmd_.velocity += velocity_hdg_ff_->velocity;
      } else if (velocity_hdg_rate_ff_) {
        velocity_hdg_cmd_.velocity += velocity_hdg_rate_ff_->velocity;
      }
    }

    if (active_input == UavSystem::VELOCITY_HDG_CMD) {

      acceleration_hdg_cmd_ = velocity_controller_.getControlSignal(multirotor_model_.getState(), velocity_hdg_cmd_, dt);
      active_input          = ACCELERATION_HDG_CMD;

      if (acceleration_hdg_ff_) {
        acceleration_hdg_cmd_.acceleration += acceleration_hdg_ff_->acceleration;
      } else if (acceleration_hdg_rate_ff_) {
        acceleration_hdg_cmd_.acceleration += acceleration_hdg_rate_ff_->acceleration;
      }

    } else if (active_input == UavSystem::VELOCITY_HDG_RATE_CMD) {

      acceleration_hdg_rate_cmd_ = velocity_controller_.getControlSignal(multirotor_model_.getState(), velocity_hdg_rate_cmd_, dt);
      active_input               = ACCELERATION_HDG_RATE_CMD;

      if (acceleration_hdg_rate_ff_) {
        acceleration_hdg_rate_cmd_.acceleration += acceleration_hdg_rate_ff_->acceleration;
        acceleration_hdg_rate_cmd_.heading_rate += acceleration_hdg_rate_ff_->heading_rate;
      } else if (acceleration_hdg_ff_) {
        acceleration_hdg_rate_cmd_.acceleration += acceleration_hdg_ff_->acceleration;
      }
    }

    if (active_input == UavSystem::ACCELERATION_HDG_CMD) {
      attitude_cmd_ = acceleration_controller_.getControlSignal(multirotor_model_.getState(), acceleration_hdg_cmd_, dt);
      active_input  = ATTITUDE_CMD;
    } else if (active_input == UavSystem::ACCELERATION_HDG_RATE_CMD) {
      tilt_hdg_rate_cmd_ = acceleration_controller_.getControlSignal(multirotor_model_.getState(), acceleration_hdg_rate_cmd_, dt);
      active_input       = TILT_HDG_RATE_CMD;
    }

    if (active_input == UavSystem::ATTITUDE_CMD) {
      attitude_rate_cmd_ = attitude_controller_.getControlSignal(multirotor_model_.getState(), attitude_cmd_, dt);
      active_input       = ATTITUDE_RATE_CMD;
    } else if (active_input == UavSystem::TILT_HDG_RATE_CMD) {
      attitude_rate_cmd_ = attitude_controller_.getControlSignal(multirotor_model_.getState(), tilt_hdg_rate_cmd_, dt);
      active_input       = ATTITUDE_RATE_CMD;
    }

    if (active_input == UavSystem::ATTITUDE_RATE_CMD) {
      control_group_cmd_ = rate_controller_.getControlSignal(multirotor_model_.getState(), attitude_rate_cmd_, dt);
      active_input       = CONTROL_GROUP_CMD;
    }

    if (active_input == UavSystem::CONTROL_GROUP_CMD) {
      actuators_cmd_ = mixer_.getControlSignal(control_group_cmd_);
      active_input   = ACTUATOR_CMD;
    }
  }

  // set the motor input for the model
  multirotor_model_.setInput(actuators_cmd_);

  multirotor_model_.step(dt);
}

//}

/* getState() //{ */

MultirotorModel::State UavSystem::getState(void) {

  return multirotor_model_.getState();
}

//}

/* getParams() //{ */

MultirotorModel::ModelParams UavSystem::getParams(void) {

  return multirotor_model_.getParams();
}

//}

/* setParams() //{ */

void UavSystem::setParams(const MultirotorModel::ModelParams& params) {

  multirotor_model_.setParams(params);

  initializeControllers();
}

//}

/* getMixerAllocation() //{ */

Eigen::MatrixXd UavSystem::getMixerAllocation(void) {

  return mixer_.getAllocationMatrix();
}

//}

/* getImuAcceleration() //{ */

Eigen::Vector3d UavSystem::getImuAcceleration(void) {

  return multirotor_model_.getImuAcceleration();
}

//}

/* setters for controllers' params //{ */

void UavSystem::setMixerParams(const Mixer::Params& params) {
  mixer_.setParams(params);
}

void UavSystem::setRateControllerParams(const RateController::Params& params) {
  rate_controller_.setParams(params);
}

void UavSystem::setAttitudeControllerParams(const AttitudeController::Params& params) {
  attitude_controller_.setParams(params);
}

void UavSystem::setVelocityControllerParams(const VelocityController::Params& params) {
  velocity_controller_.setParams(params);
}

void UavSystem::setPositionControllerParams(const PositionController::Params& params) {
  position_controller_.setParams(params);
}

//}

}  // namespace mrs_multirotor_simulator

#endif  // UAV_SYSTEM_H
