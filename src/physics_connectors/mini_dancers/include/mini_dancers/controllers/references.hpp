#ifndef REFERENCES_H
#define REFERENCES_H

#include <iostream>
#include <eigen3/Eigen/Core>

namespace mrs_multirotor_simulator
{

namespace reference
{

/* Actuators //{ */

class Actuators {

public:
  /**
   * @brief vector of motor throttles scaled as [0, 1]
   */
  Eigen::VectorXd motors;

  friend std::ostream& operator<<(std::ostream& os, const Actuators& data) {
    os << "Actuators = " << data.motors.transpose();
    return os;
  }
};

//}

/* ControlGroup //{ */

class ControlGroup {
public:
  /**
   * @brief the applied roll (around body-X) torque normalized to [-1, 1]
   */
  double roll = 0;

  /**
   * @brief the applied pitch (around body-Y) torque normalized to [-1, 1]
   */
  double pitch = 0;

  /**
   * @brief the applied yaw (around body-Z) torque normalized to [-1, 1]
   */
  double yaw = 0;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle = 0;

  friend std::ostream& operator<<(std::ostream& os, const ControlGroup& data) {
    os << "Control group: roll = " << data.roll << ", pitch = " << data.pitch << ", yaw = " << data.yaw << ", throttle " << data.throttle;
    return os;
  }
};

//}

/* AttitudeRate //{ */

class AttitudeRate {
public:
  /**
   * @brief angular rate around body-x in [rad]
   */
  double rate_x = 0;

  /**
   * @brief angular rate around body-y in [rad]
   */
  double rate_y = 0;

  /**
   * @brief angular rate around body-z in [rad]
   */
  double rate_z = 0;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle = 0;

  friend std::ostream& operator<<(std::ostream& os, const AttitudeRate& data) {
    os << "Attitude rate: roll = " << data.rate_x << ", pitch = " << data.rate_y << ", yaw = " << data.rate_z << ", throttle " << data.throttle;
    return os;
  }
};

//}

/* Attitude //{ */

class Attitude {
public:
  Attitude() {
    this->orientation = Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3d orientation;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle = 0;

  friend std::ostream& operator<<(std::ostream& os, const Attitude& data) {
    os << "Attitude: throttle " << data.throttle << ", R = " << std::endl << data.orientation;
    return os;
  }
};

//}

/* TiltHdgRate //{ */

class TiltHdgRate {
public:
  TiltHdgRate() {
    this->tilt_vector = Eigen::Vector3d::Identity();
  }

  Eigen::Vector3d tilt_vector;

  double heading_rate = 0;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle = 0;

  friend std::ostream& operator<<(std::ostream& os, const TiltHdgRate& data) {
    os << "Attitude: throttle " << data.throttle << ", tilt = " << std::endl << data.tilt_vector.transpose() << ", heading rate = " << data.heading_rate;
    return os;
  }
};

//}

/* AccelerationHdgRate //{ */

class AccelerationHdgRate {
public:
  AccelerationHdgRate(const Eigen::Vector3d& acceleration, const double& heading_rate) {
    this->acceleration = acceleration;
    this->heading_rate = heading_rate;
  }

  AccelerationHdgRate() {
    this->acceleration = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d acceleration;

  double heading_rate = 0;

  friend std::ostream& operator<<(std::ostream& os, const AccelerationHdgRate& data) {
    os << "Acceleration: acc = " << data.acceleration.transpose() << ", heading rate = " << data.heading_rate;
    return os;
  }
};

//}

/* AccelerationHdg //{ */

class AccelerationHdg {
public:
  AccelerationHdg(const Eigen::Vector3d& acceleration, const double& heading) {
    this->acceleration = acceleration;
    this->heading      = heading;
  }

  AccelerationHdg() {
    this->acceleration = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d acceleration;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading = 0;

  friend std::ostream& operator<<(std::ostream& os, const AccelerationHdg& data) {
    os << "Acceleration: acc = " << data.acceleration.transpose() << ", heading = " << data.heading;
    return os;
  }
};

//}

/* VelocityHdgRate //{ */

class VelocityHdgRate {
public:
  VelocityHdgRate(const Eigen::Vector3d& velocity, const double& heading_rate) {
    this->velocity     = velocity;
    this->heading_rate = heading_rate;
  }

  VelocityHdgRate() {
    this->velocity = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d velocity;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading_rate = 0;

  friend std::ostream& operator<<(std::ostream& os, const VelocityHdgRate& data) {
    os << "Velocity: vel = " << data.velocity.transpose() << ", heading rate = " << data.heading_rate;
    return os;
  }
};

//}

/* VelocityHdg //{ */

class VelocityHdg {
public:
  VelocityHdg(const Eigen::Vector3d& velocity, const double& heading) {
    this->velocity = velocity;
    this->heading  = heading;
  }

  VelocityHdg() {
    this->velocity = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d velocity;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading = 0;

  friend std::ostream& operator<<(std::ostream& os, const VelocityHdg& data) {
    os << "Velocity: vel = " << data.velocity.transpose() << ", heading rate = " << data.heading;
    return os;
  }
};

//}

/* Position //{ */

class Position {
public:
  Position() {
    this->position = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d position;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading = 0;

  friend std::ostream& operator<<(std::ostream& os, const Position& data) {
    os << "Position: pos = " << data.position.transpose() << ", heading = " << data.heading;
    return os;
  }
};

//}

}  // namespace reference

}  // namespace mrs_multirotor_simulator

#endif  // REFERENCES_H
