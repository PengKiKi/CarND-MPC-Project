#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
extern const double Lf;
extern const double max_steer;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);


  double delta_prev {0};
  double a_prev {0.1};
};

#endif /* MPC_H */
