#include "rix/robot/kinematics_solver.hpp"

#include <Eigen/Geometry>
#include <random>

#include "rix/robot/eigen_util.hpp"

namespace rix {
namespace robot {

double get_random(double lower, double upper) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(lower, upper);  // [lower, upper)
    return dist(gen);
}

KinematicsSolver::KinematicsSolver(std::shared_ptr<RobotModel> robot, double step_scale, double tolerance,
                                   uint32_t max_iterations)
    : robot_(robot), step_scale_(step_scale), tolerance_(tolerance), max_iterations_(max_iterations) {}

bool KinematicsSolver::solve_ik(const std::string &link_name, const rix::msg::geometry::Transform &goal,
                                rix::msg::sensor::JS initial_guess, rix::msg::sensor::JS &solution) {
    auto chain = robot_->get_joints_in_chain(link_name);
    if (initial_guess.joint_states.empty()) {
        for (auto j : chain) {
            double p;
            if (j->limits().lower == 0 && j->limits().upper == 0) {
                p = get_random(-M_PI, M_PI);
            } else if (j->limits().lower < -1000 || j->limits().upper > 1000) {
                p = 0;
            } else {
                p = get_random(j->limits().lower, j->limits().upper);
            }

            rix::msg::sensor::JointState joint_state;
            joint_state.name = j->name();
            joint_state.position = p;
            joint_state.velocity = 0;
            joint_state.effort = 0;
            initial_guess.joint_states.push_back(joint_state);
        }
    }

    robot_->set_state(initial_guess);

    // TODO: Implement the gradient descent loop with output as a single solution
    return false;
}

bool KinematicsSolver::solve_ik(const std::string &link_name, const rix::msg::geometry::Transform &goal,
                                rix::msg::sensor::JS initial_guess, std::vector<rix::msg::sensor::JS> &solution) {
    auto chain = robot_->get_joints_in_chain(link_name);
    if (initial_guess.joint_states.empty()) {
        for (auto j : chain) {
            double p;
            if (j->limits().lower == 0 && j->limits().upper == 0) {
                p = get_random(-M_PI, M_PI);
            } else if (j->limits().lower < -1000 || j->limits().upper > 1000) {
                p = 0;
            } else {
                p = get_random(j->limits().lower, j->limits().upper);
            }

            rix::msg::sensor::JointState joint_state;
            joint_state.name = j->name();
            joint_state.position = p;
            joint_state.velocity = 0;
            joint_state.effort = 0;
            initial_guess.joint_states.push_back(joint_state);
        }
    }

    robot_->set_state(initial_guess);

    // TODO: Implement the gradient descent loop with output as a trajectory from initial_guess to solution
    return false;
}

rix::msg::geometry::Transform KinematicsSolver::solve_fk(const std::string &link) const {
    // TODO: Implement forward kinematics
    return rix::robot::transform_identity();
}

Eigen::MatrixXd KinematicsSolver::get_jacobian(const std::vector<std::shared_ptr<Joint>> &chain,
                                               Eigen::Affine3d &ee_transform) {
    // TODO: Implement Jacobian calculation (store the current end effector transform into ee_transform for later use)
    return Eigen::MatrixXd::Zero(6, chain.size());
}

bool KinematicsSolver::iterate_ik(const std::vector<std::shared_ptr<Joint>> &chain, const std::string &link_name,
                                  const Eigen::Affine3d &goal) {
    // TODO: Implement a single iteration of the IK solver using gradient descent
    return false;
}

}  // namespace robot
}  // namespace rix