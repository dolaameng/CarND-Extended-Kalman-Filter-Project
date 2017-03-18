#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <tuple>
#include <algorithm>

#include "common_type.h"
#include "kalman_filter.h"

using std::vector;
using std::string;
using std::ifstream; using std::ofstream;
using std::tuple; using std::tie; using std::forward_as_tuple;
using std::cout; using std::cerr; using std::endl;

using std::transform;
using std::begin;
using std::end;

using namespace kalman;

tuple<string, string> parse_arguments(int argc, char* argv[]) {

  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  if (argc == 3) {
    return std::make_tuple(argv[1], argv[2]);
  } else {
    throw std::invalid_argument("Invalid arguments - " + usage_instructions);
  }
}

tuple<vector<Measurement>, vector<PositionVelocity>>
parse_input(ifstream & in_stream) {
  vector<Measurement> measurements;
  vector<PositionVelocity> groud_truth;

  TimeStamp ts;
  Value x, y; // radar observation
  Value rho, phi, rho_dot; // laser observation
  Value gx, gy, gvx, gvy; // groud truth

  for(string line; getline(in_stream, line); ) {
    std::stringstream ss{line};
    string sensor; ss >> sensor;
    if (sensor == "L") {
      // continue;
      ss >> x >> y >> ts;
      measurements.push_back(Measurement{ts, x, y});
    } else if(sensor == "R") {
      // continue;
      ss >> rho >> phi >> rho_dot >> ts;
      measurements.push_back(Measurement{ts, rho, phi, rho_dot});
    } else {
      throw std::invalid_argument("Unrecognized measurement " + sensor);
    }
    ss >> gx >> gy >> gvx >> gvy;
    groud_truth.push_back(PositionVelocity{gx, gy, gvx, gvy});
  }
  return forward_as_tuple(measurements, groud_truth);
}



int main(int argc, char* argv[]) {

  // parse arguments
  string in_file, out_file;
  tie(in_file, out_file) = parse_arguments(argc, argv);

  // check input and output files
  ifstream in_stream {in_file, ifstream::in};
  if (!in_stream.is_open()) {
    throw std::invalid_argument("invalid input path");
  }
  ofstream out_stream {out_file, ofstream::out};
  if (!out_stream.is_open()) {
    throw std::invalid_argument("invalid output path");
  }

  // parse input to measurements and groud_truth
  vector<Measurement> measurements;
  vector<PositionVelocity> groud_truth;
  tie(measurements, groud_truth) = parse_input(in_stream);
  assert(measurements.size() == groud_truth.size());

  // use kalman filter to track and estimate motion
  // from different sensors
  auto filter = kalman::Filter();
  vector<PositionVelocity> estimates(measurements.size());
  transform(begin(measurements), end(measurements),
            begin(estimates), filter);
  assert(estimates.size() == measurements.size());

  // analyze the result
  // When used as a metric rather than an objective for optimization,
  // the ratio of RMSE might be more inituitive than RMSE though.
  // BUT WHY DOES vx and vy may have different RMSE?
  auto rmse = root_mean_squared_error(estimates, groud_truth);
  cout << "RMSE: " << rmse << endl;
  
  // write to output
  for(auto i = 0; i < estimates.size(); ++i) {
    out_stream << estimates[i] << " ";
    out_stream << measurements[i].to_xy() << " ";
    out_stream << groud_truth[i] << endl;
  }


  return EXIT_SUCCESS;
}