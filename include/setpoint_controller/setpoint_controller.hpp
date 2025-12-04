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

class SetpointController : public rix::core::Node {
   public:
    SetpointController(const rix::ipc::Endpoint &rixhub_endpoint, std::shared_ptr<rix::robot::RobotModel> robot,
                       double rate, const std::string &end_effector);

   private:
    void timer_callback(const rix::core::Timer::Event &);

    double get_random(double lower, double upper);

    void publish_interpolated_joint_states(const rix::msg::sensor::JS &start, const rix::msg::sensor::JS &end,
                                           size_t step_count);

    std::shared_ptr<rix::robot::RobotModel> robot_;
    std::shared_ptr<rix::core::ServiceClient> client_;
    std::shared_ptr<rix::core::Publisher> joint_state_pub_;
    std::shared_ptr<rix::core::Publisher> goal_pub_;
    std::string end_effector_;
};