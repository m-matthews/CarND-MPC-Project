#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state, polynomial coefficients and latency in milliseconds.
  // Return the first actuation.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, bool latency);

  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  static constexpr double Lf = 2.67;

  // Reference velocity in mph.
  static constexpr const double ref_v = 90.0;  // Initial development at 50mph. 100mph works, however can 'touch' the side, so 90mph used.

  // Multiplier for MPH to m/s.
  static constexpr const double mph_to_mps = 0.44704;
};

#endif /* MPC_H */
