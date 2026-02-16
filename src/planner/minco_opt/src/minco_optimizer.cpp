#include <minco_opt/minco_optimizer.h>

#include <algorithm>

namespace ego_planner {

void MincoOptimizer::setLimits(double max_vel, double max_acc) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
}

bool MincoOptimizer::optimize(const Eigen::Vector3d& start_p,
                              const Eigen::Vector3d& /*start_v*/,
                              const Eigen::Vector3d& /*start_a*/,
                              const Eigen::Vector3d& goal_p,
                              const Eigen::Vector3d& /*goal_v*/,
                              MincoResult& out) {
  const double distance = (goal_p - start_p).norm();
  const double vel = std::max(0.1, max_vel_);
  const double duration = std::max(0.2, distance / vel);

  out.coeff_x = Eigen::MatrixXd::Zero(1, 6);
  out.coeff_y = Eigen::MatrixXd::Zero(1, 6);
  out.coeff_z = Eigen::MatrixXd::Zero(1, 6);
  out.durations = Eigen::VectorXd::Constant(1, duration);

  // Constant + linear term placeholder (for compile/link unblock).
  out.coeff_x(0, 0) = start_p.x();
  out.coeff_y(0, 0) = start_p.y();
  out.coeff_z(0, 0) = start_p.z();

  out.coeff_x(0, 1) = (goal_p.x() - start_p.x()) / duration;
  out.coeff_y(0, 1) = (goal_p.y() - start_p.y()) / duration;
  out.coeff_z(0, 1) = (goal_p.z() - start_p.z()) / duration;

  (void)max_acc_;
  return true;
}

} // namespace ego_planner
