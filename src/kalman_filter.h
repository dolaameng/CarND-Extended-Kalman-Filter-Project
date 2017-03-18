#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
/*Kalman Filter header file
**/

#include "common_type.h"
#include <algorithm>
#include "Eigen/Dense"

namespace kalman {
  // A Kalman Filter is a functor with a state.
  // When called, it updates its state with new measurements, 
  // as well as spitting out its current estimate.
  class Filter {
  public:
    Filter(Value noise_ax=9., Value noise_ay=9.):
    is_ready{false}, noise_ax{noise_ax}, noise_ay{noise_ay} {
      // initialize transformation variables
      F << 1, 0, 1, 0,
           0, 1, 0, 1,
           0, 0, 1, 0,
           0, 0, 0, 1;
      Q << 1, 0, 1, 0,
           0, 1, 0, 1,
           1, 0, 1, 0,
           0, 1, 0, 1;
      // to observation
      H_laser << 1, 0, 0, 0,
                 0, 1, 0, 0;
      H_radar << 1, 1, 0, 0,
                 1, 1, 0, 0,
                 1, 1, 1, 1;
      // given in the code template
      // might be parameterized from data in future
      R_laser << 0.0225, 0     ,
                 0     , 0.0225;
      R_radar << 0.09  , 0     , 0     ,
                 0     , 0.0009, 0     ,
                 0     , 0     , 0.09  ;      

    }
    ~Filter() = default; // no need to take care of rule-of-six

    // state initialization
    void init(const Measurement & measurement) {
      is_ready = true;
      auto position = measurement.to_xy();
      // random shuffling to avoid radar-only update freezes 
      // because of initial all-zero positions
      estimate << position(0)+1e-5, position(1)+1e-5, 0, 0;
      P << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    }

    // functor interface
    PositionVelocity operator() (const Measurement & measurement) {
      if (!is_ready) { 
        init(measurement);
      } else {
        // predict
        TimeDelta dt_secs = micro_to_sec(measurement.timestamp - timestamp);
        predict(dt_secs);
        // update
        update(measurement);
      }
      // move along time
      timestamp = measurement.timestamp;

      // spit out estimate
      return estimate;
    }

    void predict(TimeDelta dt) {
      if (dt <= 1e-6) {
        return;
      }
      // update transform with time delta
      auto dt2 = dt * dt;
      auto dt3 = dt2 * dt;
      auto dt4 = dt2 * dt2;
      auto ax_dt4 = noise_ax * dt4 / 4;
      auto ax_dt3 = noise_ax * dt3 / 2;
      auto ax_dt2 = noise_ax * dt2;
      auto ay_dt4 = noise_ay * dt4 / 4;
      auto ay_dt3 = noise_ay * dt3 / 2;
      auto ay_dt2 = noise_ay * dt2;
      F(0, 2) = dt;
      F(1, 3) = dt;
      Q(0, 0) = ax_dt4;
      Q(0, 2) = ax_dt3;
      Q(1, 1) = ay_dt4;
      Q(1, 3) = ay_dt3;
      Q(2, 0) = ax_dt3;
      Q(2, 2) = ax_dt2;
      Q(3, 1) = ay_dt3;
      Q(3, 3) = ay_dt2;

      // update state with predicted value
      estimate = std::move(F * estimate);
      P = std::move(F * P * F.transpose() + Q);   
    }

    void update(const Measurement & m) {
      if (m.sensor == Sensor::LASER) {
        auto residual = m.observation - H_laser * estimate; 

        update_state(residual, H_laser, R_laser);

      } else {
        // calculate Jacobian(H_radar) by talyor 1st order
        auto x = estimate(0);
        auto y = estimate(1);
        auto vx = estimate(2);
        auto vy = estimate(3);
        auto n2 = x*x + y*y;
        auto n = sqrt(n2);
        auto n3 = n2 * n;
        Value eps = 1e-9;

        if (n <= eps) {
          H_radar << 0, 0, 0, 0,
                     0, 0, 0, 0,
                     0, 0, 0, 0;
        } else {
          H_radar(0, 0) = x / n;
          H_radar(0, 1) = y / n;
          H_radar(1, 0) = -y / n2;
          H_radar(1, 1) = x / n2;
          H_radar(2, 0) = y * (vx*y - vy*x) / n3;
          H_radar(2, 1) = x * (vy*x - vx*y) / n3;
          H_radar(2, 2) = x / n;
          H_radar(2, 3) = y / n;
        }
        
        // calculate residual mean for radar (rho, phi, rho_dot)
        // residual[1] must be in [-pi, pi]
        Value rho = n;
        Value phi = 0.;
        Value rho_dot = 0.;
        if (abs(x) > eps && abs(y) > eps) {
          phi = std::atan2(y, x);
          rho_dot = (x*vx + y*vy) / rho;
        }
        PolarPosition residual{m.observation - PolarPosition{rho, phi, rho_dot}};

        if (residual(1) > PI) residual(1) -= 2*PI;
        if (residual(1) < -PI) residual(1) += 2*PI;
        assert(residual(1) <= PI);
        assert(residual(1) >= -PI);
        update_state(residual, H_radar, R_radar);
      }
    }

    // general recipe to update when residual is given
    template<
      typename Residual,
      typename ObservationTransform,
      typename ObservationCovariance>
    void update_state(const Residual & residual,
      const ObservationTransform & H,
      const ObservationCovariance & R) {
      // Eigen::MatrixXd H_t = H.transpose();
      auto H_t = H.transpose();
      auto S = H * P * H_t + R;
      // somehow using auto here introduces a subtle bug
      // that results in negative covariance - not sure
      // how inverse is done in Eigen.
      DynamicMatrix K = P * H_t * S.inverse();

      estimate += K * residual;
      P = (I - K * H) * P;
      // for (auto i = 0; i < 4; ++i) {
      //   assert(P(i, i) >= 0);
      // }
    }

  private:
    // internal states
    bool is_ready;
    PositionVelocity estimate;
    PositionVelocityCovariance P;
    TimeStamp timestamp;
  private:
    // parameters that control behaviors
    const Value noise_ax; // acceleration var on x
    const Value noise_ay; // acceleration var on y
  private:
    // non-state members
    // state and observation transformation
    const PositionVelocityCovariance I = PositionVelocityCovariance::Identity();
    StateTransform F; 
    StateTransformCovariance Q; 
    LaserObservationTransform H_laser; 
    RadarObservationTransform H_radar; 
    LaserObservationCovariance R_laser; 
    RadarObservationCovariance R_radar; 
  };
}
#endif