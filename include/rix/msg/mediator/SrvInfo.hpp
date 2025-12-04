#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <map>
#include <string>
#include <cstring>

#include "rix/msg/serialization.hpp"
#include "rix/msg/message.hpp"
#include "rix/msg/mediator/Endpoint.hpp"

namespace rix {
namespace msg {
namespace mediator {

class SrvInfo : public Message {
  public:
    std::string name;
    uint64_t id;
    uint64_t node_id;
    uint8_t protocol;
    mediator::Endpoint endpoint;
    std::array<uint64_t, 2> request_hash;
    std::array<uint64_t, 2> response_hash;

    SrvInfo() = default;
    SrvInfo(const SrvInfo &other) = default;
    ~SrvInfo() = default;

    size_t size() const override {
        using namespace detail;
        size_t size = 0;
        size += size_string(name);
        size += size_number(id);
        size += size_number(node_id);
        size += size_number(protocol);
        size += size_message(endpoint);
        size += size_number_array(request_hash);
        size += size_number_array(response_hash);
        return size;
    }

    std::array<uint64_t, 2> hash() const override {
        return {0x5677a7cf4eb908a7ULL, 0xdbe0eebccfef43baULL};
    }

    void serialize(uint8_t *dst, size_t &offset) const override {
        using namespace detail;
        serialize_string(dst, offset, name);
        serialize_number(dst, offset, id);
        serialize_number(dst, offset, node_id);
        serialize_number(dst, offset, protocol);
        serialize_message(dst, offset, endpoint);
        serialize_number_array(dst, offset, request_hash);
        serialize_number_array(dst, offset, response_hash);
    }

    bool deserialize(const uint8_t *src, size_t size, size_t &offset) override {
        using namespace detail;
        if (!deserialize_string(name, src, size, offset)) { return false; };
        if (!deserialize_number(id, src, size, offset)) { return false; };
        if (!deserialize_number(node_id, src, size, offset)) { return false; };
        if (!deserialize_number(protocol, src, size, offset)) { return false; };
        if (!deserialize_message(endpoint, src, size, offset)) { return false; };
        if (!deserialize_number_array(request_hash, src, size, offset)) { return false; };
        if (!deserialize_number_array(response_hash, src, size, offset)) { return false; };
        return true;
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix