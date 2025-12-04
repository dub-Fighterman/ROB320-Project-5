#pragma once

#include "rix/msg/geometry/Pose.hpp"
#include "rix/robot/eigen_util.hpp"
#include "rix/robot/robot_model.hpp"

namespace rix {
namespace robot {

/**
 * @class KinematicsSolver
 * @brief Solver for forward and inverse kinematics problems for robot manipulators.
 *
 * This class provides methods to solve forward kinematics (FK) and inverse kinematics (IK)
 * problems using gradient descent optimization.
 */
class KinematicsSolver {
   public:
    /**
     * @brief Constructs a KinematicsSolver with specified parameters.
     *
     * @param robot Shared pointer to the robot model
     * @param step_scale Step size multiplier for gradient descent (default: 0.25)
     * @param tolerance Convergence tolerance for IK solver (default: 1e-5)
     * @param max_iterations Maximum number of iterations for IK solver (default: 1000)
     */
    KinematicsSolver(std::shared_ptr<RobotModel> robot, double step_scale = 0.25, double tolerance = 1e-5,
                     uint32_t max_iterations = 1000);

    /**
     * @brief Solves inverse kinematics to reach a goal transform with a single solution.
     *
     * Uses gradient descent to find joint configurations that place the specified link
     * at the goal transform. Returns a single joint state solution.
     *
     * @param link_name Name of the end effector link
     * @param goal Target transform for the end effector
     * @param initial_guess Initial joint configuration (if empty, will be randomly generated)
     * @param solution Output parameter for the final joint state solution
     * @return true if a solution was found within tolerance, false otherwise
     * @todo Implement the gradient descent loop with output as a single solution
     */
    bool solve_ik(const std::string &link_name, const rix::msg::geometry::Transform &goal,
                  rix::msg::sensor::JS initial_guess, rix::msg::sensor::JS &solution);

    /**
     * @brief Solves inverse kinematics to reach a goal transform with trajectory output.
     *
     * Uses gradient descent to find joint configurations that place the specified link
     * at the goal transform. Returns a trajectory of joint states from initial guess to solution.
     *
     * @param link_name Name of the end effector link
     * @param goal Target transform for the end effector
     * @param initial_guess Initial joint configuration (if empty, will be randomly generated)
     * @param solution Output parameter for the trajectory of joint states
     * @return true if a solution was found within tolerance, false otherwise
     * @todo Implement the gradient descent loop with output as a trajectory from initial_guess to solution
     */
    bool solve_ik(const std::string &link_name, const rix::msg::geometry::Transform &goal,
                  rix::msg::sensor::JS initial_guess, std::vector<rix::msg::sensor::JS> &solution);

    /**
     * @brief Solves forward kinematics for the specified link.
     *
     * Computes the transform of the specified link relative to the robot's base frame
     * given the current joint state.
     *
     * @param link_name Name of the link to compute the transform for
     * @return The transform of the link relative to the base frame
     * @todo Implement forward kinematics
     */
    rix::msg::geometry::Transform solve_fk(const std::string &link_name) const;

    /**
     * @brief Computes the Jacobian matrix for the kinematic chain.
     *
     * Calculates the 6xN Jacobian matrix relating joint velocities to end effector
     * twist (linear and angular velocities).
     *
     * @param chain Vector of joints in the kinematic chain
     * @param ee_transform Output parameter for the current end effector transform
     * @return 6xN Jacobian matrix where N is the number of joints in the chain
     * @todo Implement Jacobian calculation (store the current end effector transform into ee_transform for later use)
     */
    Eigen::MatrixXd get_jacobian(const std::vector<std::shared_ptr<Joint>> &chain, Eigen::Affine3d &ee_transform);

    /**
     * @brief Gets the current step scale parameter.
     * @return Step scale value
     */
    inline double get_step_scale() const { return step_scale_; }

    /**
     * @brief Gets the current tolerance parameter.
     * @return Tolerance value
     */
    inline double get_tolerance() const { return tolerance_; }

    /**
     * @brief Gets the maximum number of iterations.
     * @return Maximum iteration count
     */
    inline uint32_t get_max_iterations() const { return max_iterations_; }

    /**
     * @brief Sets the step scale parameter for gradient descent.
     * @param step_scale New step scale value
     */
    inline void set_step_scale(double step_scale) { step_scale_ = step_scale; }

    /**
     * @brief Sets the convergence tolerance for IK solver.
     * @param tolerance New tolerance value
     */
    inline void set_tolerance(double tolerance) { tolerance_ = tolerance; }

    /**
     * @brief Sets the maximum number of iterations for IK solver.
     * @param max_iterations New maximum iteration count
     */
    inline void set_max_iterations(uint32_t max_iterations) { max_iterations_ = max_iterations; }

   private:
    std::shared_ptr<RobotModel> robot_;  ///< Shared pointer to the robot model
    double step_scale_;                  ///< Step size multiplier for gradient descent
    double tolerance_;                   ///< Convergence tolerance for IK solver
    uint32_t max_iterations_;            ///< Maximum number of iterations for IK solver

    /**
     * @brief Performs a single iteration of the IK solver.
     *
     * Executes one step of gradient descent to move the end effector closer to the goal.
     *
     * @param chain Vector of joints in the kinematic chain
     * @param link_name Name of the end effector link
     * @param goal Target transform for the end effector
     * @return true if converged within tolerance, false otherwise
     * @todo Implement a single iteration of the IK solver using gradient descent
     */
    bool iterate_ik(const std::vector<std::shared_ptr<Joint>> &chain, const std::string &link_name,
                    const Eigen::Affine3d &goal);
};

}  // namespace robot
}  // namespace rix