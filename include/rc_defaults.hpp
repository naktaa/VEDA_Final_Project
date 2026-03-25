#pragma once

#include <string_view>

namespace rc_defaults {

inline constexpr std::string_view kBrokerHost = "192.168.100.7";
inline constexpr int kBrokerPort = 1883;

inline constexpr std::string_view kGoalTopic = "wiserisk/rc/goal";
inline constexpr std::string_view kPoseTopic = "wiserisk/p1/pose";
inline constexpr std::string_view kSafetyTopic = "wiserisk/rc/safety";
inline constexpr std::string_view kStatusTopic = "wiserisk/rc/status";

inline constexpr int kStatusPublishIntervalMs = 50;

} // namespace rc_defaults
