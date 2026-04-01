#pragma once

#include <string>

#include "rc_control_types.hpp"

bool ParseGoalJson(const std::string& payload, RcGoal& out_goal);
bool ParsePoseJson(const std::string& payload, RcPose& out_pose);
bool ParseSafetyJson(const std::string& payload, RcSafety& out_safety);
