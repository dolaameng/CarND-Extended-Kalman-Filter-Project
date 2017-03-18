#ifndef COMMON_TYPE_H_ // common header
#define COMMON_TYPE_H_

/* Domain Specific Types Definition
**/

#include "Eigen/Dense"
#include <cmath>
#include <iostream>
#include <vector>

using std::vector;

namespace kalman {
  
  // signal precision
  using Value = double;
  // PI
  constexpr Value PI = 3.14159265;
  // position : (x, y)
  using Position = Eigen::Matrix<Value, 2, 1>;
  // polar position : (rho, phi, rho_dot)
  using PolarPosition = Eigen::Matrix<Value, 3, 1>;
  // position and velocity: (x, y, vx, vy)
  using PositionVelocity = Eigen::Matrix<Value, 4, 1>;
  // observation: (x, y) or (rho, phi, rho_dot)
  using Observation = Eigen::Matrix<Value, Eigen::Dynamic, 1>;
  // most general format
  using DynamicMatrix = Eigen::Matrix<Value, Eigen::Dynamic, Eigen::Dynamic>;
  // time stamp
  using TimeStamp = unsigned long;
  using TimeDelta = double;
  constexpr TimeDelta micro_to_sec(TimeStamp microsecond) {return microsecond / 1000000.;}
  // sensor type
  enum class Sensor {
    LASER,
    RADAR
  };
  // measurement
  struct Measurement {
    // for laser
    Measurement(TimeStamp ts, Value x, Value y):
    sensor{Sensor::LASER}, timestamp{ts}, observation{2}{
      observation << x, y;
    }
    // for radar
    Measurement(TimeStamp ts, Value rho, Value phi, Value rho_dot):
    sensor{Sensor::RADAR}, timestamp{ts}, observation{3}{
      observation << rho, phi, rho_dot;
    }

    // get observed position
    Position to_xy() const {
      if (sensor == Sensor::LASER) {
        return observation.block<2, 1>(0, 0);
      } else { // RADAR
        Value rho = observation(0);
        Value phi = observation(1);
        Value x = rho * std::cos(phi);
        Value y = rho * std::sin(phi);
        return Position{x, y};
      }
    }
    Sensor sensor;
    TimeStamp timestamp; // in microseconds
    Observation observation;
  };



  // process related types
  using PositionVelocityCovariance = Eigen::Matrix<Value, 4, 4>; // P
  using StateTransform = Eigen::Matrix<Value, 4, 4>; // F
  using StateTransformCovariance = Eigen::Matrix<Value, 4, 4>; // Q
  using LaserObservationTransform = Eigen::Matrix<Value, 2, 4>; // H laser
  using RadarObservationTransform = Eigen::Matrix<Value, 3, 4>; // H radar
  using LaserObservationCovariance = Eigen::Matrix<Value, 2, 2>; // R laser
  using RadarObservationCovariance = Eigen::Matrix<Value, 3, 3>; // R radar
}

template<typename V>
V root_mean_squared_error(const vector<V> & yhat,
                          const vector<V> & y) {
  assert(yhat.size() == y.size());
  auto n = yhat.size();
  auto result = V{};
  for (auto i = 0; i < n; ++i) {
    auto d2 = (yhat[i].array() - y[i].array()).square();
    result = result.array() + d2;
  }
  result /= n;
  result = result.array().sqrt();
  return result;
}

// output function for vectors

template<int D>
std::ostream & operator<<(std::ostream & out,
          const Eigen::Matrix<kalman::Value, D, 1> & v) {
  for (auto i = 0; i < D; ++i) {
    out << v[i];
    if (i < D-1) out << " ";
  }
  return out;
}



#endif