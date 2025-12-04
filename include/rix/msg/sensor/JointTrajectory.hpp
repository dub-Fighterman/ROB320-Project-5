#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/sensor/JS.hpp"

namespace rix {
namespace msg {
namespace sensor {

class JointTrajectory : public Message {
  public:
    std::vector<sensor::JS> points;

    JointTrajectory() = default;
    JointTrajectory(const JointTrajectory &other) = default;
    ~JointTrajectory() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_message_vector(points);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x7e711e3a1be132afULL, 0x8e3cbe6572d07254ULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_message_vector(dst, offset, points);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_message_vector(points, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix