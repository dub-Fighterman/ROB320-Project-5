#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rix/core/common.hpp"
#include "rix/ipc/signal.hpp"
#include "rix/msg/sensor/JS.hpp"
#include "rix/msg/sensor/JointState.hpp"
#include "rix/msg/sensor/JointTrajectory.hpp"
#include "rix/msg/trajectory_service_msgs/TrajectoryRequest.hpp"
#include "rix/rix.hpp"
#include "rix/robot/eigen_util.hpp"
#include "rix/robot/kinematics_solver.hpp"
#include "rix/robot/robot_model.hpp"
#include "rix/tf/transform_listener.hpp"

class TrajectoryService : public rix::core::Node {
   public:
    TrajectoryService(const rix::ipc::Endpoint &rixhub_endpoint, std::shared_ptr<rix::robot::RobotModel> robot);

   private:
    void setpoint_callback(const rix::msg::trajectory_service_msgs::TrajectoryRequest &request,
                           rix::msg::sensor::JointTrajectory &response);

    std::shared_ptr<rix::robot::RobotModel> robot_;
};