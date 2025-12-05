#include "rix/robot/kinematics_solver.hpp"

#include <Eigen/Geometry>
#include <random>
#include <algorithm>

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
    Eigen::Affine3d goal_tf = msg_to_eigen(goal);
    for (uint32_t i = 0; i < max_iterations_; ++i) {
        if (iterate_ik(chain, link_name, goal_tf)) {
            solution = robot_->get_joint_states();
            return true;
        }
    }

    solution = robot_->get_joint_states();

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
    solution.clear();
    solution.push_back(robot_->get_joint_states());

    Eigen::Affine3d goal_tf = msg_to_eigen(goal);
    for (uint32_t i = 0; i < max_iterations_; ++i) {
        if (iterate_ik(chain, link_name, goal_tf)) {
            solution.push_back(robot_->get_joint_states());
            return true;
        }

        solution.push_back(robot_->get_joint_states());
    }

    return false;
}

rix::msg::geometry::Transform KinematicsSolver::solve_fk(const std::string &link) const {
    // TODO: Implement forward kinematics
    auto chain = robot_->get_joints_in_chain(link);

    Eigen::Affine3d T = msg_to_eigen(robot_->get_world_to_root());

    for (const auto &joint : chain) {

        Eigen::Affine3d origin_tf = msg_to_eigen(joint->origin());
        Eigen::Affine3d motion_tf = Eigen::Affine3d::Identity();
        Eigen::Vector3d axis = msg_to_eigen(joint->axis());
        if (axis.norm() > 0) {
            axis.normalize();
        }

        if (joint->type() == Joint::REVOLUTE || joint->type() == Joint::CONTINUOUS) {
            motion_tf = Eigen::AngleAxisd(joint->position(), axis);
        } else if (joint->type() == Joint::PRISMATIC) {
            motion_tf.translation() = axis * joint->position();
        }

        T = T * origin_tf * motion_tf;
    }

    return eigen_to_msg(T);
    //return rix::robot::transform_identity();
}

Eigen::MatrixXd KinematicsSolver::get_jacobian(const std::vector<std::shared_ptr<Joint>> &chain,
                                               Eigen::Affine3d &ee_transform) {
    // TODO: Implement Jacobian calculation (store the current end effector transform into ee_transform for later use)
    const std::vector<std::shared_ptr<Joint>> &forward_chain = chain;

    Eigen::Affine3d T = msg_to_eigen(transform_identity());

    std::vector<Eigen::Vector3d> axes_world;
    std::vector<Eigen::Vector3d> origins_world;
    std::vector<Eigen::Affine3d> T_push;
    axes_world.reserve(forward_chain.size());
    origins_world.reserve(forward_chain.size());

    /*for (const auto &joint : forward_chain) {
        Eigen::Affine3d origin_tf = msg_to_eigen(joint->origin());
        Eigen::Affine3d joint_frame = T * origin_tf;

        Eigen::Vector3d axis_local = msg_to_eigen(joint->axis());
        if (axis_local.norm() > 0) {
            axis_local.normalize();
        }

        Eigen::Vector3d axis_world = joint_frame.rotation() * axis_local;
        axes_world.push_back(axis_world);
        origins_world.push_back(joint_frame.translation());

        Eigen::Affine3d motion_tf = Eigen::Affine3d::Identity();
        if (joint->type() == Joint::REVOLUTE || joint->type() == Joint::CONTINUOUS) {
            motion_tf = Eigen::AngleAxisd(joint->position(), axis_local);
        } else if (joint->type() == Joint::PRISMATIC) {
            motion_tf.translation() = axis_local * joint->position();
        }

        T = joint_frame * motion_tf;
    }*/
    for (const auto &joint : forward_chain) {
        Eigen::Affine3d origin_tf = msg_to_eigen(joint->origin());
        Eigen::Affine3d X = msg_to_eigen(joint->transform());
        T = T * origin_tf;
        T_push.push_back(T);
        T = T * X;
    }

    ee_transform = T;
    Eigen::Vector3d ee_pos = ee_transform.translation();

    Eigen::MatrixXd J(6, forward_chain.size());
    for (size_t i = 0; i < forward_chain.size(); ++i) {
        Eigen::Vector3d linear = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular = Eigen::Vector3d::Zero();

        auto joint = forward_chain[i];
        if (joint->type() == Joint::PRISMATIC) {
            linear = axes_world[i];
        } else if (joint->type() == Joint::REVOLUTE || joint->type() == Joint::CONTINUOUS) {
            angular = axes_world[i];
            linear = axes_world[i].cross(ee_pos - origins_world[i]);
        }

        J.block<3, 1>(0, i) = linear;
        J.block<3, 1>(3, i) = angular;
    }

    return J;
    //return Eigen::MatrixXd::Zero(6, chain.size());
}

bool KinematicsSolver::iterate_ik(const std::vector<std::shared_ptr<Joint>> &chain, const std::string &link_name,
                                  const Eigen::Affine3d &goal) {
    // TODO: Implement a single iteration of the IK solver using gradient descent
    //return false;
    Eigen::Affine3d ee_transform;
    Eigen::MatrixXd J = get_jacobian(chain, ee_transform);

    Eigen::Vector3d pos_error = goal.translation() - ee_transform.translation();
    Eigen::Matrix3d rot_error_mat = goal.rotation() * ee_transform.rotation().transpose();
    Eigen::AngleAxisd rot_error(rot_error_mat);
    Eigen::Vector3d rot_vec = rot_error.angle() * rot_error.axis();

    Eigen::VectorXd error(6);
    error << pos_error, rot_vec;

    if (error.norm() < tolerance_) {
        return true;
    }

    const double damping = 1e-4;
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd damped = JJt + damping * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd delta = step_scale_ * J.transpose() * damped.ldlt().solve(error);

    for (size_t i = 0; i < chain.size(); ++i) {
        auto joint = chain[i];
        double new_pos = joint->clamp(joint->position() + delta[i]);
        robot_->set_state(joint->name(), new_pos, 0, 0);
    }

    return error.norm() < tolerance_;
}

}  // namespace robot
}  // namespace rix