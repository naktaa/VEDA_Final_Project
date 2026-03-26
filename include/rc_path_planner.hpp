#pragma once

#include <vector>

#include "rc_control_types.hpp"

std::vector<RcWaypoint> BuildSmoothPath(const RcPose& pose,
                                        const RcGoal& goal,
                                        double final_tolerance_cm);
