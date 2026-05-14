#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace sim {
constexpr int ROW = 8;
constexpr int COL = 8;
constexpr int MAX_TARGET_NUM = 6;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;

constexpr double CHANNEL_WIDTH = 2.0;
constexpr double CHANNEL_LENGTH = 10.0;
constexpr double CHANNEL_HEIGHT = 1.8;
constexpr double OBSTACLE_WIDTH = 0.8;
constexpr double OBSTACLE_DEPTH = 0.3;
constexpr double OBSTACLE_HEIGHT = 1.5;
constexpr double ROBOT_DIAMETER = 0.35;
constexpr double ROBOT_RADIUS = ROBOT_DIAMETER / 2.0;
constexpr double SENSOR_Z0 = 0.75;

constexpr double TOF_MAX_RANGE_M = 2.5;
constexpr uint16_t TOF_NO_TARGET_MM = 8190;
constexpr uint16_t MAX_DISTANCE_TO_PROCESS_MM = 2500;

constexpr double FOV_H = 45.0 * DEG2RAD;
constexpr double FOV_V = 45.0 * DEG2RAD;

constexpr uint16_t DIS_REACT = 1400;
constexpr uint16_t DIS_SLOW = 700;
constexpr uint16_t DIS_STOP = 500;
constexpr uint16_t DIS_FEAR = 150;
constexpr uint16_t DIS_GROUND_MIN = 400;
constexpr uint16_t DIS_CEILING_MIN = 600;
constexpr int GROUND_BORDER = 6;
constexpr int CELLING_BORDER = 2;
constexpr int MIN_PIXEL_NUMBER = 1;

constexpr double TURN_MAX = 1.0;
constexpr double TURN_SLOW = 0.7;
constexpr double TURN_FAST = 3.0;
constexpr double TURN_NOT = 0.0;
constexpr double VEL_STOP = 0.0;
constexpr double VEL_FEAR = -0.2;
constexpr double VEL_SCALE_MEDIUM = 1.0;
constexpr double VEL_SCALE_SLOW = 0.7;
constexpr double VEL_UP = 0.2;
constexpr double VEL_DOWN = -0.5;
constexpr double MAX_TURN_RATIO = 0.8;
constexpr double EPSILON = 0.0001;

constexpr double DT = 0.02;
constexpr double SIM_TIME_LIMIT = 80.0;
constexpr double MAX_FORWARD_SPEED = 0.35;
constexpr double MAX_REVERSE_SPEED = -0.12;
constexpr double MAX_YAW_RATE = 210.0 * DEG2RAD;
constexpr double COURSE_ALPHA = 3.2;
constexpr double ALTITUDE_ALPHA = 2.0;
constexpr int CONSOLE_PRINT_EVERY = 5;
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct AABB {
    string name;
    double xmin = 0.0;
    double xmax = 0.0;
    double ymin = 0.0;
    double ymax = 0.0;
    double zmin = 0.0;
    double zmax = 0.0;
    bool physical = true;
};

struct Borders {
    int top = sim::ROW;
    int left = sim::COL;
    int right = -1;
    int bottom = -1;
};

struct Target {
    double row = 0.0;
    double col = 0.0;
    Borders borders;
    uint16_t min_distance = numeric_limits<uint16_t>::max();
    int pixels_number = 0;
};

struct FlyCommand {
    double command_velocity_x = 0.0;
    double command_velocity_z = 0.0;
    double command_turn = 0.0;
};

struct DecisionMetrics {
    uint16_t min_global = numeric_limits<uint16_t>::max();
    uint16_t min_front = numeric_limits<uint16_t>::max();
    uint16_t min_left = numeric_limits<uint16_t>::max();
    uint16_t min_right = numeric_limits<uint16_t>::max();
    uint16_t min_up = numeric_limits<uint16_t>::max();
    uint16_t min_down = numeric_limits<uint16_t>::max();
    int object_count = 0;
};

struct PID {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double min_out = 0.0;
    double max_out = 0.0;
    double integral = 0.0;
    double prev_error = 0.0;

    PID(double p, double i, double d, double min_value, double max_value)
        : kp(p), ki(i), kd(d), min_out(min_value), max_out(max_value) {}

    double update(double target, double current, double dt) {
        const double error = target - current;
        integral += error * dt;
        const double derivative = (error - prev_error) / dt;
        prev_error = error;
        const double output = kp * error + ki * integral + kd * derivative;
        return clamp(output, min_out, max_out);
    }
};

struct RobotState {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = sim::SENSOR_Z0;
    double psi = 0.0;
    double psi_cmd = 0.0;
    double speed = 0.0;
    double target_speed = 0.0;
    double yaw_rate = 0.0;
    double target_yaw_rate = 0.0;
    double z_cmd = sim::SENSOR_Z0;
};

struct FrameLog {
    RobotState state;
    FlyCommand command;
    DecisionMetrics metrics;
    array<array<uint16_t, sim::COL>, sim::ROW> tof{};
    double clearance = numeric_limits<double>::infinity();
    bool collision = false;
};

struct DecisionState {
    array<double, 5> previous_command{};
    size_t index = 0;

    DecisionState() {
        previous_command.fill(0.0);
        previous_command[0] = -1.0;
    }

    double handleExceptionCommands(double current_turn_command) {
        int left_commands = 0;
        int right_commands = 0;
        for (double cmd : previous_command) {
            if (cmd < -sim::TURN_MAX + sim::EPSILON) {
                right_commands++;
            } else if (cmd > sim::TURN_MAX - sim::EPSILON) {
                left_commands++;
            }
        }

        double refined = current_turn_command;
        if ((current_turn_command < -sim::TURN_MAX + sim::EPSILON ||
             current_turn_command > sim::TURN_MAX - sim::EPSILON) &&
            (left_commands + right_commands) >= static_cast<int>(sim::MAX_TURN_RATIO * previous_command.size())) {
            refined = (right_commands > left_commands) ? -sim::TURN_FAST : sim::TURN_FAST;
        }

        previous_command[index] = current_turn_command;
        index = (index + 1) % previous_command.size();
        return refined;
    }
};

static vector<AABB> createEnvironment() {
    vector<AABB> boxes;
    boxes.push_back({"left_wall", -1.02, -1.0, 0.0, sim::CHANNEL_LENGTH, 0.0, sim::CHANNEL_HEIGHT, true});
    boxes.push_back({"right_wall", 1.0, 1.02, 0.0, sim::CHANNEL_LENGTH, 0.0, sim::CHANNEL_HEIGHT, true});

    for (int i = 0; i < 4; ++i) {
        const double y0 = 2.0 + 2.0 * i;
        const double y1 = y0 + sim::OBSTACLE_DEPTH;
        const bool left_side = (i % 2 == 0);
        const double x0 = left_side ? -sim::CHANNEL_WIDTH / 2.0 : sim::CHANNEL_WIDTH / 2.0 - sim::OBSTACLE_WIDTH;
        const double x1 = x0 + sim::OBSTACLE_WIDTH;
        boxes.push_back({"block_" + to_string(i + 1), x0, x1, y0, y1, 0.0, sim::OBSTACLE_HEIGHT, true});
    }
    return boxes;
}

static optional<double> intersectRayAABB(const Vec3& origin, const Vec3& dir, const AABB& box) {
    double tmin = 0.0;
    double tmax = numeric_limits<double>::infinity();

    const array<double, 3> o = {origin.x, origin.y, origin.z};
    const array<double, 3> d = {dir.x, dir.y, dir.z};
    const array<double, 3> mn = {box.xmin, box.ymin, box.zmin};
    const array<double, 3> mx = {box.xmax, box.ymax, box.zmax};

    for (int axis = 0; axis < 3; ++axis) {
        if (fabs(d[axis]) < 1e-9) {
            if (o[axis] < mn[axis] || o[axis] > mx[axis]) {
                return nullopt;
            }
            continue;
        }

        double t1 = (mn[axis] - o[axis]) / d[axis];
        double t2 = (mx[axis] - o[axis]) / d[axis];
        if (t1 > t2) {
            swap(t1, t2);
        }
        tmin = max(tmin, t1);
        tmax = min(tmax, t2);
        if (tmax < tmin) {
            return nullopt;
        }
    }

    if (tmax < 0.0) {
        return nullopt;
    }
    return tmin >= 0.0 ? tmin : tmax;
}

static Vec3 tofRayDirection(double psi, int row, int col) {
    const double horiz = (((static_cast<double>(col) + 0.5) / sim::COL) - 0.5) * sim::FOV_H;
    const double vert = (0.5 - ((static_cast<double>(row) + 0.5) / sim::ROW)) * sim::FOV_V;
    const double yaw = psi + horiz;
    return {sin(yaw) * cos(vert), cos(yaw) * cos(vert), sin(vert)};
}

static array<array<uint16_t, sim::COL>, sim::ROW> simulateToF(const RobotState& robot, const vector<AABB>& boxes) {
    array<array<uint16_t, sim::COL>, sim::ROW> tof{};
    const Vec3 origin{robot.x, robot.y, robot.z};

    for (int r = 0; r < sim::ROW; ++r) {
        for (int c = 0; c < sim::COL; ++c) {
            const Vec3 dir = tofRayDirection(robot.psi, r, c);
            double best = numeric_limits<double>::infinity();
            for (const AABB& box : boxes) {
                const auto hit = intersectRayAABB(origin, dir, box);
                if (hit && *hit < best) {
                    best = *hit;
                }
            }
            tof[r][c] = (best <= sim::TOF_MAX_RANGE_M) ?
                static_cast<uint16_t>(lround(best * 1000.0)) : sim::TOF_NO_TARGET_MM;
        }
    }

    return tof;
}

static vector<Target> extractTargets(const array<array<uint16_t, sim::COL>, sim::ROW>& tof) {
    bool binary[sim::ROW][sim::COL] = {};
    bool visited[sim::ROW][sim::COL] = {};
    for (int r = 0; r < sim::ROW; ++r) {
        for (int c = 0; c < sim::COL; ++c) {
            binary[r][c] = (tof[r][c] <= sim::MAX_DISTANCE_TO_PROCESS_MM);
        }
    }

    vector<Target> targets;
    const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    for (int sr = 0; sr < sim::ROW; ++sr) {
        for (int sc = 0; sc < sim::COL; ++sc) {
            if (!binary[sr][sc] || visited[sr][sc]) {
                continue;
            }

            Target target;
            int row_sum = 0;
            int col_sum = 0;
            queue<pair<int, int>> q;
            q.push({sr, sc});
            visited[sr][sc] = true;

            while (!q.empty()) {
                const auto [r, c] = q.front();
                q.pop();

                target.pixels_number++;
                row_sum += r;
                col_sum += c;
                target.min_distance = min(target.min_distance, tof[r][c]);
                target.borders.top = min(target.borders.top, r);
                target.borders.bottom = max(target.borders.bottom, r);
                target.borders.left = min(target.borders.left, c);
                target.borders.right = max(target.borders.right, c);

                for (int k = 0; k < 8; ++k) {
                    const int nr = r + dr[k];
                    const int nc = c + dc[k];
                    if (nr >= 0 && nr < sim::ROW && nc >= 0 && nc < sim::COL && binary[nr][nc] && !visited[nr][nc]) {
                        visited[nr][nc] = true;
                        q.push({nr, nc});
                    }
                }
            }

            target.row = static_cast<double>(row_sum) / target.pixels_number;
            target.col = static_cast<double>(col_sum) / target.pixels_number;
            if (static_cast<int>(targets.size()) < sim::MAX_TARGET_NUM) {
                targets.push_back(target);
            }
        }
    }
    return targets;
}

static FlyCommand decisionMaking(const vector<Target>& targets, DecisionState& state, DecisionMetrics& metrics) {
    double command_velocity_x = sim::VEL_SCALE_MEDIUM;
    double command_velocity_z = sim::VEL_STOP;
    double command_turn = sim::TURN_NOT;

    metrics = {};
    metrics.object_count = static_cast<int>(targets.size());

    if (targets.empty()) {
        return {command_velocity_x, command_velocity_z, command_turn};
    }

    for (const Target& target : targets) {
        const bool critical_close = target.min_distance <= sim::DIS_STOP;
        const bool boundary_threat = target.borders.top >= sim::GROUND_BORDER || target.borders.bottom <= sim::CELLING_BORDER;
        if (!critical_close && !boundary_threat && target.pixels_number < sim::MIN_PIXEL_NUMBER) {
            continue;
        }

        const uint16_t dist = target.min_distance;
        metrics.min_global = min(metrics.min_global, dist);

        const bool in_drone_zone =
            target.borders.right >= 3 && target.borders.left <= 4 &&
            target.borders.bottom >= 3 && target.borders.top <= 5;
        if (in_drone_zone) {
            metrics.min_front = min(metrics.min_front, dist);
        }

        if (target.col < 3.5) {
            metrics.min_left = min(metrics.min_left, dist);
        } else {
            metrics.min_right = min(metrics.min_right, dist);
        }

        if (target.borders.bottom <= sim::CELLING_BORDER) {
            metrics.min_up = min(metrics.min_up, dist);
        }
        if (target.borders.top >= sim::GROUND_BORDER) {
            metrics.min_down = min(metrics.min_down, dist);
        }
    }

    if (metrics.min_global <= sim::DIS_FEAR) {
        return {sim::VEL_FEAR, sim::VEL_STOP, sim::TURN_NOT};
    }

    bool vz_active = false;
    if (metrics.min_down < sim::DIS_GROUND_MIN) {
        command_velocity_z = sim::VEL_UP;
        vz_active = true;
    }
    if (metrics.min_up < sim::DIS_CEILING_MIN) {
        command_velocity_z = sim::VEL_DOWN;
        vz_active = true;
    }

    if (metrics.min_front < sim::DIS_REACT) {
        if (metrics.min_front <= sim::DIS_STOP) {
            command_velocity_x = sim::VEL_STOP;
        } else if (metrics.min_front <= sim::DIS_SLOW) {
            command_velocity_x = (static_cast<double>(metrics.min_front - sim::DIS_STOP) /
                                  static_cast<double>(sim::DIS_SLOW - sim::DIS_STOP)) * sim::VEL_SCALE_SLOW;
        } else {
            command_velocity_x = (static_cast<double>(metrics.min_front - sim::DIS_SLOW) /
                                  static_cast<double>(sim::DIS_REACT - sim::DIS_SLOW)) *
                                      (sim::VEL_SCALE_MEDIUM - sim::VEL_SCALE_SLOW) +
                                  sim::VEL_SCALE_SLOW;
        }
    }

    if (metrics.min_left < sim::DIS_REACT || metrics.min_right < sim::DIS_REACT) {
        if (metrics.min_left < metrics.min_right) {
            if (metrics.min_left <= sim::DIS_STOP) {
                command_turn = sim::TURN_MAX;
            } else if (metrics.min_left <= sim::DIS_SLOW) {
                command_turn = sim::TURN_MAX * 0.8;
            } else {
                command_turn = sim::TURN_SLOW;
            }
        } else {
            if (metrics.min_right <= sim::DIS_STOP) {
                command_turn = -sim::TURN_MAX;
            } else if (metrics.min_right <= sim::DIS_SLOW) {
                command_turn = -sim::TURN_MAX * 0.8;
            } else {
                command_turn = -sim::TURN_SLOW;
            }
        }
    }

    if (vz_active) {
        command_velocity_x = min(command_velocity_x, sim::VEL_SCALE_SLOW);
        command_turn = clamp(command_turn, -sim::TURN_SLOW, sim::TURN_SLOW);
    }

    command_turn = state.handleExceptionCommands(command_turn);
    return {command_velocity_x, command_velocity_z, command_turn};
}

static double clearanceToBox(const Vec3& p, const AABB& box) {
    const double cx = clamp(p.x, box.xmin, box.xmax);
    const double cy = clamp(p.y, box.ymin, box.ymax);
    const double cz = clamp(p.z, box.zmin, box.zmax);
    const double dx = p.x - cx;
    const double dy = p.y - cy;
    const double dz = p.z - cz;
    return sqrt(dx * dx + dy * dy + dz * dz) - sim::ROBOT_RADIUS;
}

static bool checkCollisionAndClearance(const RobotState& robot, const vector<AABB>& boxes, double& clearance) {
    const Vec3 p{robot.x, robot.y, robot.z};
    bool collision = false;
    clearance = numeric_limits<double>::infinity();
    for (const AABB& box : boxes) {
        if (!box.physical) {
            continue;
        }
        const double c = clearanceToBox(p, box);
        clearance = min(clearance, c);
        if (c <= 0.0) {
            collision = true;
        }
    }
    return collision;
}

static filesystem::path outputDirectory() {
    filesystem::path source_path = filesystem::absolute(filesystem::path(__FILE__));
    if (filesystem::exists(source_path.parent_path())) {
        return source_path.parent_path();
    }
    return filesystem::current_path();
}

static void writeObstacleCsv(const filesystem::path& path, const vector<AABB>& boxes) {
    ofstream out(path);
    out << "name,xmin,xmax,ymin,ymax,zmin,zmax\n";
    for (const AABB& box : boxes) {
        out << box.name << ',' << box.xmin << ',' << box.xmax << ',' << box.ymin << ',' << box.ymax << ','
            << box.zmin << ',' << box.zmax << '\n';
    }
}

static void writeLogCsv(const filesystem::path& path, const vector<FrameLog>& frames) {
    ofstream out(path);
    out << fixed << setprecision(6);
    out << "time,x,y,z,psi,psi_cmd,speed,target_speed,cmd_vx,cmd_vz,cmd_turn,target_yaw_rate,yaw_rate,"
        << "object_count,min_global_mm,min_front_mm,min_left_mm,min_right_mm,min_up_mm,min_down_mm,clearance,collision";
    for (int r = 0; r < sim::ROW; ++r) {
        for (int c = 0; c < sim::COL; ++c) {
            out << ",tof" << r << c;
        }
    }
    out << '\n';

    for (const FrameLog& frame : frames) {
        const RobotState& s = frame.state;
        const FlyCommand& cmd = frame.command;
        out << s.t << ',' << s.x << ',' << s.y << ',' << s.z << ',' << s.psi << ',' << s.psi_cmd << ','
            << s.speed << ',' << s.target_speed << ',' << cmd.command_velocity_x << ',' << cmd.command_velocity_z << ','
            << cmd.command_turn << ',' << s.target_yaw_rate << ',' << s.yaw_rate << ','
            << frame.metrics.object_count << ','
            << frame.metrics.min_global << ',' << frame.metrics.min_front << ',' << frame.metrics.min_left << ','
            << frame.metrics.min_right << ',' << frame.metrics.min_up << ',' << frame.metrics.min_down << ','
            << frame.clearance << ',' << (frame.collision ? 1 : 0);
        for (int r = 0; r < sim::ROW; ++r) {
            for (int c = 0; c < sim::COL; ++c) {
                out << ',' << frame.tof[r][c];
            }
        }
        out << '\n';
    }
}

static string formatDistance(uint16_t distance_mm) {
    if (distance_mm == numeric_limits<uint16_t>::max()) {
        return "INF";
    }
    return to_string(distance_mm);
}

static double wrapAngle(double angle) {
    while (angle > sim::PI) {
        angle -= 2.0 * sim::PI;
    }
    while (angle < -sim::PI) {
        angle += 2.0 * sim::PI;
    }
    return angle;
}

static double shortestAngleError(double target, double current) {
    return wrapAngle(target - current);
}

static void printFrameSummary(const FrameLog& frame) {
    cout << fixed << setprecision(2)
         << "[t=" << frame.state.t
         << "s] pos=(" << setprecision(3) << frame.state.x << ", " << frame.state.y << ", " << frame.state.z << ") "
         << "psi=" << setprecision(3) << frame.state.psi
         << " obj=" << frame.metrics.object_count
         << " min(front/left/right)=(" << formatDistance(frame.metrics.min_front) << ", "
         << formatDistance(frame.metrics.min_left) << ", "
         << formatDistance(frame.metrics.min_right) << ")mm"
         << " cmd(vx,vz,turn)=(" << setprecision(3) << frame.command.command_velocity_x << ", "
         << frame.command.command_velocity_z << ", "
         << frame.command.command_turn << ")"
         << " clr=" << frame.clearance;
    if (frame.collision) {
        cout << " COLLISION";
    }
    cout << '\n';
}

static void writeDecisionTrace(const filesystem::path& path, const vector<FrameLog>& frames) {
    ofstream out(path);
    out << fixed << setprecision(3);
    for (const FrameLog& frame : frames) {
        out << "time=" << frame.state.t
            << " pos=(" << frame.state.x << ", " << frame.state.y << ", " << frame.state.z << ")"
            << " psi=" << frame.state.psi
            << " psi_cmd=" << frame.state.psi_cmd
            << " speed=" << frame.state.speed
            << " target_speed=" << frame.state.target_speed
            << " object_count=" << frame.metrics.object_count
            << " min_global=" << formatDistance(frame.metrics.min_global)
            << " min_front=" << formatDistance(frame.metrics.min_front)
            << " min_left=" << formatDistance(frame.metrics.min_left)
            << " min_right=" << formatDistance(frame.metrics.min_right)
            << " min_up=" << formatDistance(frame.metrics.min_up)
            << " min_down=" << formatDistance(frame.metrics.min_down)
            << " cmd=(" << frame.command.command_velocity_x << ", "
            << frame.command.command_velocity_z << ", "
            << frame.command.command_turn << ")"
            << " clearance=" << frame.clearance
            << " collision=" << (frame.collision ? 1 : 0) << '\n';
        for (int r = 0; r < sim::ROW; ++r) {
            out << "  ";
            for (int c = 0; c < sim::COL; ++c) {
                out << setw(5) << frame.tof[r][c];
            }
            out << '\n';
        }
        out << '\n';
    }
}

int main() {
    const vector<AABB> boxes = createEnvironment();
    DecisionState decision_state;
    PID speed_pid(2.8, 0.15, 0.05, -1.2, 1.2);
    PID yaw_rate_pid(1.15, 0.05, 0.03, -0.75, 0.75);
    PID altitude_pid(1.8, 0.0, 0.05, -0.5, 0.5);

    RobotState robot;
    vector<FrameLog> frames;
    frames.reserve(static_cast<size_t>(sim::SIM_TIME_LIMIT / sim::DT));

    bool collision = false;
    size_t frame_index = 0;
    while (robot.t <= sim::SIM_TIME_LIMIT && robot.y < sim::CHANNEL_LENGTH && !collision) {
        FrameLog frame;
        frame.state = robot;
        frame.tof = simulateToF(robot, boxes);
        const vector<Target> targets = extractTargets(frame.tof);
        frame.command = decisionMaking(targets, decision_state, frame.metrics);

        frame.state.target_speed = clamp(frame.command.command_velocity_x * sim::MAX_FORWARD_SPEED,
                                         sim::MAX_REVERSE_SPEED, sim::MAX_FORWARD_SPEED);
        frame.state.target_yaw_rate = clamp(frame.command.command_turn * sim::MAX_YAW_RATE,
                                            -sim::TURN_FAST * sim::MAX_YAW_RATE,
                                            sim::TURN_FAST * sim::MAX_YAW_RATE);
        frame.state.yaw_rate = robot.yaw_rate;
        frame.collision = checkCollisionAndClearance(robot, boxes, frame.clearance);
        frames.push_back(frame);
        if ((frame_index % sim::CONSOLE_PRINT_EVERY) == 0 || frames.back().collision) {
            printFrameSummary(frames.back());
        }
        frame_index++;

        const double acceleration_cmd = speed_pid.update(frame.state.target_speed, robot.speed, sim::DT);
        robot.speed = clamp(robot.speed + acceleration_cmd * sim::DT, sim::MAX_REVERSE_SPEED, sim::MAX_FORWARD_SPEED);

        const double yaw_rate_correction = yaw_rate_pid.update(frame.state.target_yaw_rate, robot.yaw_rate, sim::DT);
        robot.psi_cmd = wrapAngle(robot.psi_cmd + (frame.state.target_yaw_rate + yaw_rate_correction) * sim::DT);
        robot.yaw_rate = sim::COURSE_ALPHA * shortestAngleError(robot.psi_cmd, robot.psi);
        robot.psi = wrapAngle(robot.psi + robot.yaw_rate * sim::DT);

        robot.z_cmd = clamp(robot.z_cmd + frame.command.command_velocity_z * sim::DT, sim::ROBOT_RADIUS, sim::CHANNEL_HEIGHT - sim::ROBOT_RADIUS);
        const double z_rate = sim::ALTITUDE_ALPHA * altitude_pid.update(robot.z_cmd, robot.z, sim::DT);
        robot.z = clamp(robot.z + z_rate * sim::DT, sim::ROBOT_RADIUS, sim::CHANNEL_HEIGHT - sim::ROBOT_RADIUS);

        robot.x += robot.speed * sin(robot.psi) * sim::DT;
        robot.y += robot.speed * cos(robot.psi) * sim::DT;
        robot.t += sim::DT;
        collision = frame.collision;
    }

    double final_clearance = numeric_limits<double>::infinity();
    collision = checkCollisionAndClearance(robot, boxes, final_clearance);
    FrameLog final_frame;
    final_frame.state = robot;
    final_frame.tof = simulateToF(robot, boxes);
    const vector<Target> final_targets = extractTargets(final_frame.tof);
    final_frame.command = decisionMaking(final_targets, decision_state, final_frame.metrics);
    final_frame.clearance = final_clearance;
    final_frame.collision = collision;
    frames.push_back(final_frame);
    printFrameSummary(frames.back());

    const filesystem::path out_dir = outputDirectory();
    const filesystem::path log_path = out_dir / "tof_sim_log.csv";
    const filesystem::path obstacle_path = out_dir / "tof_obstacles.csv";
    const filesystem::path trace_path = out_dir / "tof_decision_trace.txt";
    writeLogCsv(log_path, frames);
    writeObstacleCsv(obstacle_path, boxes);
    writeDecisionTrace(trace_path, frames);

    cout << "ToF corridor simulation finished.\n";
    cout << "Frames: " << frames.size() << "\n";
    cout << "Result: " << (collision ? "avoidance failed (collision)" : (robot.y >= sim::CHANNEL_LENGTH ? "success" : "time limit")) << "\n";
    cout << "Log: " << log_path.string() << "\n";
    cout << "Obstacles: " << obstacle_path.string() << "\n";
    cout << "Trace: " << trace_path.string() << "\n";
    return collision ? 2 : 0;
}




