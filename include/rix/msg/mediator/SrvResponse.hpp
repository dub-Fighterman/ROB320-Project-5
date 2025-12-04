#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/mediator/SrvInfo.hpp"

namespace rix {
namespace msg {
namespace mediator {

class SrvResponse : public Message {
  public:
    uint8_t error;
    mediator::SrvInfo srv_info;

    SrvResponse() = default;
    SrvResponse(const SrvResponse &other) = default;
    ~SrvResponse() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_number(error);
        size += size_message(srv_info);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0xea5c0f5bb06c7b42ULL, 0xad127f3568ed78efULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_number(dst, offset, error);
        serialize_message(dst, offset, srv_info);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_number(error, src, size, offset)) { return false; };
        if (!deserialize_message(srv_info, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix