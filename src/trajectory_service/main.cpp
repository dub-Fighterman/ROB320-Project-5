#include "trajectory_service/trajectory_service.hpp"

int main(int argc, char **argv) {
    rix::util::ArgumentParser parser("trajectory_service", "Publishes JS messages");
    parser.add<std::string>("file", "Robot description file");
    parser.add<std::string>("pub_ep", "Endpoint for publisher to host on", {});
    parser.add<std::string>("ip", "RIX Hub IP Address", 'i', "127.0.0.1");
    parser.add<double>("rate", "Publish rate in rate", 'r', 20.0);

    if (!parser.parse(argc, argv)) {
        rix::util::Log::error << parser.help() << std::endl;
        return 1;
    }

    std::string robot_description_file;
    if (!parser.get<std::string>("file", robot_description_file)) {
        rix::util::Log::error << "Failed to get file argument" << std::endl;
        return 1;
    }

    rix::ipc::Endpoint rixhub_endpoint;
    if (!parser.get<std::string>("ip", rixhub_endpoint.address)) {
        rix::util::Log::error << "Failed to get ip argument" << std::endl;
        return 1;
    }
    rixhub_endpoint.port = rix::core::RIXHUB_PORT;

    std::string endpoint_string;
    if (!parser.get<std::string>("pub_ep", endpoint_string)) {
        rix::util::Log::error << "Failed to get pub_ep argument" << std::endl;
        return 1;
    }
    rix::ipc::Endpoint publisher_endpoint("127.0.0.1", 0);
    if (!endpoint_string.empty()) {
        size_t colon_pos = endpoint_string.find_first_of(':');
        rix::ipc::Endpoint publisher_endpoint(endpoint_string.substr(0, colon_pos),
                                              std::stoi(endpoint_string.substr(colon_pos)));
    }

    double rate;
    if (!parser.get<double>("rate", rate)) {
        rix::util::Log::error << "Failed to get rate argument" << std::endl;
        return 1;
    }

    auto robot = std::make_shared<rix::robot::RobotModel>(robot_description_file);
    auto traj_srv = std::make_shared<TrajectoryService>(rixhub_endpoint, robot);

    auto signal = std::make_shared<rix::ipc::Signal>(SIGINT);
    traj_srv->spin(signal);

    return 0;
}