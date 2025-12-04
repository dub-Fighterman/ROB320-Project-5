#include "setpoint_controller/setpoint_controller.hpp"

SetpointController::SetpointController(const rix::ipc::Endpoint &rixhub_endpoint,
                                       std::shared_ptr<rix::robot::RobotModel> robot, double rate,
                                       const std::string &end_effector)
    : Node("setpoint_controller", rixhub_endpoint), robot_(robot), end_effector_(end_effector) {
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

    client_ =
        create_service_client<rix::msg::trajectory_service_msgs::TrajectoryRequest, rix::msg::sensor::JointTrajectory>(
            "/trajectory/setpoint");
    if (!client_->ok()) {
        rix::util::Log::error << "Failed to create ServiceClient." << std::endl;
        shutdown();
        return;
    }

    joint_state_pub_ = create_publisher<rix::msg::sensor::JS>("/trajectory/joint_states");
    if (!joint_state_pub_->ok()) {
        rix::util::Log::error << "Failed to create Publisher." << std::endl;
        shutdown();
        return;
    }
    
    goal_pub_ = create_publisher<rix::msg::geometry::Transform>("/trajectory/goal");
    if (!goal_pub_->ok()) {
        rix::util::Log::error << "Failed to create Publisher." << std::endl;
        shutdown();
        return;
    }

    auto chain = robot_->get_joints_in_chain(end_effector_);
    for (auto j : chain) {
        double p;
        if (j->limits().lower == 0 && j->limits().upper == 0) {
            p = get_random(-M_PI, M_PI);
        } else if (j->limits().lower < -1000 || j->limits().upper > 1000) {
            p = 0;
        } else {
            p = get_random(j->limits().lower, j->limits().upper);
        }
        j->set_state(p, 0, 0);
    }

    create_timer(rix::util::Duration(1 / rate),
                 std::bind(&SetpointController::timer_callback, this, std::placeholders::_1));
}

double SetpointController::get_random(double lower, double upper) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(lower, upper);  // [lower, upper)
    return dist(gen);
}

void SetpointController::publish_interpolated_joint_states(const rix::msg::sensor::JS &start,
                                                           const rix::msg::sensor::JS &end, size_t step_count) {
    rix::msg::sensor::JS js = start;
    for (size_t i = 0; i < step_count; ++i) {
        double ratio = (double)i / step_count;
        for (size_t j = 0; j < js.joint_states.size(); ++j) {
            js.joint_states[j].position = start.joint_states[j].position +
                                          ratio * (end.joint_states[j].position - start.joint_states[j].position);
        }
        joint_state_pub_->publish(js);
        rix::util::sleep_for(rix::util::Duration(0.01));
    }
}

void SetpointController::timer_callback(const rix::core::Timer::Event &) {
    rix::msg::trajectory_service_msgs::TrajectoryRequest request;
    request.end_effector = end_effector_;

    // Get random, valid configuration
    auto chain = robot_->get_joints_in_chain(end_effector_);
    for (auto j : chain) {
        request.start_q.joint_states.push_back(j->get_state());

        double p;
        if (j->limits().lower == 0 && j->limits().upper == 0) {
            p = get_random(-M_PI, M_PI);
        } else if (j->limits().lower < -1000 || j->limits().upper > 1000) {
            p = 0;
        } else {
            p = get_random(j->limits().lower, j->limits().upper);
        }
        j->set_state(p, 0, 0);
    }

    rix::robot::KinematicsSolver solver(robot_);
    request.end_effector = end_effector_;
    request.end_pose = solver.solve_fk(end_effector_);
    // request.end_pose.rotation = rix::robot::transform_identity().rotation;
    request.step_size = 0.5;
    request.max_iterations = 100;
    request.error = 1e-2;

    robot_->set_state(request.start_q);
    
    rix::msg::sensor::JointTrajectory response;
    if (client_->call(request, response)) {
        if (!response.points.empty()) {
            goal_pub_->publish(request.end_pose);
            for (size_t i = 0; i < response.points.size(); ++i) {
                joint_state_pub_->publish(response.points[i]);
                rix::util::sleep_for(rix::util::Duration(0.05));
            }
            robot_->set_state(response.points.back());
        }
    }
}