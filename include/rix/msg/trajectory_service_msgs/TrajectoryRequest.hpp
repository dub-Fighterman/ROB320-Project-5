#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/sensor/JS.hpp"

namespace rix {
namespace msg {
namespace trajectory_service_msgs {

class TrajectoryRequest : public Message {
  public:
    std::string end_effector;
    sensor::JS start_q;
    geometry::Transform end_pose;
    double step_size;
    double error;
    uint32_t max_iterations;

    TrajectoryRequest() = default;
    TrajectoryRequest(const TrajectoryRequest &other) = default;
    ~TrajectoryRequest() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string(end_effector);
        size += size_message(start_q);
        size += size_message(end_pose);
        size += size_number(step_size);
        size += size_number(error);
        size += size_number(max_iterations);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xd316078466a525eaULL, 0x601047a9754468ccULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string(dst, offset, end_effector);
        serialize_message(dst, offset, start_q);
        serialize_message(dst, offset, end_pose);
        serialize_number(dst, offset, step_size);
        serialize_number(dst, offset, error);
        serialize_number(dst, offset, max_iterations);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string(end_effector, src, size, offset)) { return false; };
        if (!deserialize_message(start_q, src, size, offset)) { return false; };
        if (!deserialize_message(end_pose, src, size, offset)) { return false; };
        if (!deserialize_number(step_size, src, size, offset)) { return false; };
        if (!deserialize_number(error, src, size, offset)) { return false; };
        if (!deserialize_number(max_iterations, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace trajectory_service_msgs
} // namespace msg
} // namespace rix