#include "rc_path_planner.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

double Clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

double NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double Distance(const Vec2& a, const Vec2& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

Vec2 EvalBezier(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double t) {
    const double u = 1.0 - t;
    const double b0 = u * u * u;
    const double b1 = 3.0 * u * u * t;
    const double b2 = 3.0 * u * t * t;
    const double b3 = t * t * t;

    return {
        (b0 * p0.x) + (b1 * p1.x) + (b2 * p2.x) + (b3 * p3.x),
        (b0 * p0.y) + (b1 * p1.y) + (b2 * p2.y) + (b3 * p3.y),
    };
}

} // namespace

std::vector<RcWaypoint> BuildSmoothPath(const RcPose& pose,
                                        const RcGoal& goal,
                                        double final_tolerance_m) {
    std::vector<RcWaypoint> out;
    if (!pose.valid || !goal.valid) {
        return out;
    }

    const Vec2 start{pose.x, pose.y};
    const Vec2 target{goal.x, goal.y};
    const double dist = Distance(start, target);
    const double deadzone = std::max(final_tolerance_m, 0.12);
    if (dist <= deadzone * 2.0) {
        out.push_back({goal.x, goal.y, true});
        return out;
    }

    const Vec2 heading{std::cos(pose.yaw), std::sin(pose.yaw)};
    const Vec2 goal_dir{(goal.x - pose.x) / std::max(0.001, dist),
                        (goal.y - pose.y) / std::max(0.001, dist)};
    const double heading_err =
        NormalizeAngle(std::atan2(goal.y - pose.y, goal.x - pose.x) - pose.yaw);

    double handle_start = Clamp(dist * 0.35, 0.20, 0.80);
    if (std::cos(heading_err) < 0.0) {
        handle_start = 0.12;
    }
    const double handle_end = Clamp(dist * 0.25, 0.15, 0.60);

    const Vec2 p0 = start;
    const Vec2 p1{start.x + (heading.x * handle_start), start.y + (heading.y * handle_start)};
    const Vec2 p2{target.x - (goal_dir.x * handle_end), target.y - (goal_dir.y * handle_end)};
    const Vec2 p3 = target;

    const int sample_count = static_cast<int>(Clamp(std::ceil(dist / 0.20), 4.0, 12.0));
    const double min_spacing = std::max(deadzone, 0.18);

    Vec2 last_added = start;
    for (int i = 1; i <= sample_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(sample_count);
        const Vec2 point = EvalBezier(p0, p1, p2, p3, t);
        const bool is_final = (i == sample_count);
        if (!is_final && Distance(last_added, point) < min_spacing) {
            continue;
        }

        out.push_back({point.x, point.y, is_final});
        last_added = point;
    }

    if (out.empty() || !out.back().is_final) {
        out.push_back({goal.x, goal.y, true});
    }
    return out;
}
