#include <iostream>
#include <cassert>
#include "common_type.h"
#include "kalman_filter.h"
#include "Eigen/Dense"

using namespace std;
using namespace kalman;
using namespace Eigen;

void test_measurement_ctr() {
  cout << "test measurement constructor";
  Measurement m1{1111, 1, 2};
  assert(m1.sensor == Sensor::LASER);
  Measurement m2{1112, 1, 2, 3};
  assert(m2.sensor == Sensor::RADAR);
  cout << " passed..." << endl;
}

void test_measurement_position() {
  cout << "test measurement to_xy";
  Measurement m1{1111, 1, 2};
  // cout << m1.to_xy() << endl;
  // assert( (m1.to_xy() == Vector2d{1, 2}) );
  Measurement m2{1112, 1, 0, 0};
  // assert( (m2.to_xy() == Vector2d{1, 0}) );
  cout << " passed..." << endl;
}

void test_kalman_filter_initialization() {
  cout << "test kalman filter initialization";
  auto filter = kalman::Filter();
  Measurement m1{1111, 1, 2};
  auto estimate = filter(m1);
  // assert((estimate == Vector4d{1, 2, 0, 0}));
  cout << " passed..." << endl;
}

void test_kalman_filter_predict() {
  cout << "test kalman filter predict";
  auto filter = kalman::Filter();
  Measurement m1{1111, 1, 2};
  auto estimate = filter(m1); // initialize
  // cout << filter.state_mean << endl;
  // cout << filter.state_cov << endl;
  auto pre_P = filter.P;
  filter.predict(1); // next second
  // cout << filter.state_mean << endl;
  // cout << filter.state_cov << endl;
  auto cur_P = filter.P;
  for (auto i = 0; i < 4; ++i) {
    assert(pre_P(i,i) <= cur_P(i,i));
  }
  cout << " passed..." << endl;
}

//TODO
void test_kalman_filter_update_laser() {
  cout << "test kalman filter update laser";
  auto filter = kalman::Filter();
  Measurement m1{1111, 1, 2};
  auto estimate = filter(m1);
  // cout << filter.state_cov << endl;
  Measurement m2{1112, 2, 2};
  estimate = filter(m2);
  // cout << filter.state_cov << endl;
  cout << " passed..." << endl;
}

//TODO
void test_kalman_filter_update_radar() {
  cout << "test kalman filter update radar";
  auto filter = kalman::Filter();
  Measurement m1{1111, 1, 2, 3};
  auto estimate = filter(m1);
  // cout << filter.state_cov << endl;
  Measurement m2{1112, 2, 2, 3};
  estimate = filter(m2);
  // cout << filter.state_cov << endl;
  cout << " passed..." << endl;
}

int main(int argc, char * argv[]) {
  cout << "Running tests..." << endl;

  test_measurement_ctr();
  test_measurement_position();

  test_kalman_filter_initialization();
  test_kalman_filter_predict();
  test_kalman_filter_update_laser();
  test_kalman_filter_update_radar();


  return EXIT_SUCCESS;
}