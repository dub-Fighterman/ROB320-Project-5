#include "trajectory_service/trajectory_service.hpp"

TrajectoryService::TrajectoryService(const rix::ipc::Endpoint &rixhub_endpoint,
                                     std::shared_ptr<rix::robot::RobotModel> robot)
    : Node("trajectory_service", rixhub_endpoint), robot_(robot) {
    if (!ok()) {
        rix::util::Log::error << "Failed to initialize node" << std::endl;
        shutdown();
        return;
    }

    if (!robot_->is_valid()) {
        rix::util::Log::error << "Failed to load JRDF." << std::endl;
        shutdown();
        return;
    }

    auto srv = create_service<rix::msg::trajectory_service_msgs::TrajectoryRequest, rix::msg::sensor::JointTrajectory>(
        "/trajectory/setpoint",
        std::bind(&TrajectoryService::setpoint_callback, this, std::placeholders::_1, std::placeholders::_2));
    if (!srv->ok()) {
        rix::util::Log::error << "Failed to create Service." << std::endl;
        shutdown();
        return;
    }
}

void TrajectoryService::setpoint_callback(const rix::msg::trajectory_service_msgs::TrajectoryRequest &request,
                                          rix::msg::sensor::JointTrajectory &response) {
    // Get a set of poses from the current end effector pose to the goal pose
    auto &ee_name = request.end_effector;
    auto &start_q = request.start_q;
    auto &end_pose = request.end_pose;
    auto &step_size = request.step_size;
    auto &error = request.error;
    auto &max_iterations = request.max_iterations;

    auto solver = std::make_shared<rix::robot::KinematicsSolver>(robot_, step_size, error, max_iterations);

    // Transform those end effector poses into joint states
    auto chain = robot_->get_joints_in_chain(ee_name);
    if (!solver->solve_ik(ee_name, end_pose, start_q, response.points)) {
        response.points.clear();
    }

    std::cout << "Found trajectory with " << response.points.size() << " configurations." << std::endl;
}