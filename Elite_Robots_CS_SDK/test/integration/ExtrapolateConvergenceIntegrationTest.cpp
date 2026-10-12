#include <Elite/DashboardClient.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtUtils.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "common/SimpleArgParser.hpp"

#if defined(__linux) || defined(linux) || defined(__linux__)
#include <pthread.h>
#include <sys/mman.h>
#endif

using namespace ELITE;
using namespace std::chrono;

namespace {

struct CliArgs {
    std::string robot_ip;
    std::string local_ip;
    bool use_headless_mode = true;
    std::string output_csv = "extrapolate_convergence_trace.csv";
    std::string summary_csv = "extrapolate_convergence_summary.csv";

    int rtsi_frequency_hz = 250;
    int command_duration_ms = 1200;
    int wait_after_stop_ms = 5000;
    int command_timeout_ms = 0;  // <=0 means infinite timeout
    int joint_index = 5;

    double joint_speed_rad_s = 0.2;
    double zero_speed_threshold = 0.02;
    double const_speed_keep_ratio = 0.9;
    double hold_position_epsilon = 0.0005;

    double servoj_time = 0.004;
    double servoj_extrapolate_max_time = 0.08;
    double servoj_decelerate_time = 0.01;
    double servoj_hold_velocity_threshold = 0.05;
    double servoj_hold_stable_time = 0.04;
};

struct Sample {
    double t_s = 0.0;
    int cmd_active = 0;
    std::array<double, 6> q_cmd{};
    std::array<double, 6> q{};
    std::array<double, 6> qd{};
    std::array<double, 6> tcp{};
    std::array<double, 6> tcpd{};
};

struct Metrics {
    double baseline_speed = 0.0;
    double const_velocity_duration_s = std::numeric_limits<double>::quiet_NaN();
    double deceleration_duration_s = std::numeric_limits<double>::quiet_NaN();
    double stable_to_lock_duration_s = std::numeric_limits<double>::quiet_NaN();
    double stop_distance_rad = std::numeric_limits<double>::quiet_NaN();
    double stop_error_to_last_cmd_rad = std::numeric_limits<double>::quiet_NaN();

    bool found_const_end = false;
    bool found_decel_end = false;
    bool found_lock = false;
};

void registerParserOptions(SimpleArgParser& parser) {
    parser.addOption("robot-ip", "Robot IP address.", true);
    parser.addOptionWithDefault("local-ip", "Local NIC IP for reverse/trajectory sockets.", "");
    parser.addOptionWithDefault("use-headless-mode", "Use headless mode: 1/0, true/false.", "1");
    parser.addOptionWithDefault("output-csv", "Trace CSV output path.", "extrapolate_convergence_trace.csv");
    parser.addOptionWithDefault("summary-csv", "Summary CSV output path (append mode).",
                                "extrapolate_convergence_summary.csv");

    parser.addOptionWithDefault("rtsi-frequency-hz", "RTSI polling frequency.", "250");
    parser.addOptionWithDefault("command-duration-ms", "Duration for sending servoj commands before stop.", "1200");
    parser.addOptionWithDefault("wait-after-stop-ms", "Duration to keep tracing after command stop.", "5000");
    parser.addOptionWithDefault("command-timeout-ms",
                                "writeServoj timeout. This test is designed for <=0 (infinite timeout).", "0");
    parser.addOptionWithDefault("joint-index", "Joint index [0..5] used for command and metrics.", "5");

    parser.addOptionWithDefault("joint-speed-rad-s", "Commanded joint speed for motion phase.", "0.2");
    parser.addOptionWithDefault("zero-speed-threshold", "Speed threshold to consider converged to zero.", "0.02");
    parser.addOptionWithDefault("const-speed-keep-ratio", "Ratio against baseline speed for constant-speed phase.", "0.9");
    parser.addOptionWithDefault("hold-position-epsilon", "Position window to detect hold lock.", "0.0005");

    parser.addOptionWithDefault("servoj-time", "EliteDriverConfig.servoj_time", "0.004");
    parser.addOptionWithDefault("servoj-extrapolate-max-time", "EliteDriverConfig.servoj_extrapolate_max_time", "0.08");
    parser.addOptionWithDefault("servoj-decelerate-time", "EliteDriverConfig.servoj_decelerate_time", "0.01");
    parser.addOptionWithDefault("servoj-hold-velocity-threshold", "EliteDriverConfig.servoj_hold_velocity_threshold", "0.05");
    parser.addOptionWithDefault("servoj-hold-stable-time", "EliteDriverConfig.servoj_hold_stable_time", "0.04");
}

bool parseArgs(int argc, char** argv, CliArgs& args, bool* help_requested) {
    SimpleArgParser parser("ExtrapolateConvergenceIntegrationTest",
                           "./ExtrapolateConvergenceIntegrationTest --robot-ip=<ip> [options]");
    registerParserOptions(parser);

    std::string error;
    if (!parser.parse(argc, argv, error)) {
        std::cerr << "Argument error: " << error << "\n\n";
        parser.printHelp(std::cerr);
        return false;
    }

    if (parser.isHelpRequested()) {
        if (help_requested) {
            *help_requested = true;
        }
        parser.printHelp(std::cout);
        return false;
    }

    bool ok = true;
    args.robot_ip = parser.getString("robot-ip");
    args.local_ip = parser.getStringOr("local-ip", "");
    args.use_headless_mode = parser.getBoolOr("use-headless-mode", true, &ok);
    if (!ok) {
        std::cerr << "Invalid bool for --use-headless-mode\n";
        return false;
    }

    args.output_csv = parser.getStringOr("output-csv", args.output_csv);
    args.summary_csv = parser.getStringOr("summary-csv", args.summary_csv);

    args.rtsi_frequency_hz = parser.getIntOr("rtsi-frequency-hz", args.rtsi_frequency_hz, &ok);
    if (!ok) {
        std::cerr << "Invalid int for --rtsi-frequency-hz\n";
        return false;
    }
    args.command_duration_ms = parser.getIntOr("command-duration-ms", args.command_duration_ms, &ok);
    if (!ok) {
        std::cerr << "Invalid int for --command-duration-ms\n";
        return false;
    }
    args.wait_after_stop_ms = parser.getIntOr("wait-after-stop-ms", args.wait_after_stop_ms, &ok);
    if (!ok) {
        std::cerr << "Invalid int for --wait-after-stop-ms\n";
        return false;
    }
    args.command_timeout_ms = parser.getIntOr("command-timeout-ms", args.command_timeout_ms, &ok);
    if (!ok) {
        std::cerr << "Invalid int for --command-timeout-ms\n";
        return false;
    }
    args.joint_index = parser.getIntOr("joint-index", args.joint_index, &ok);
    if (!ok) {
        std::cerr << "Invalid int for --joint-index\n";
        return false;
    }

    args.joint_speed_rad_s = parser.getDoubleOr("joint-speed-rad-s", args.joint_speed_rad_s, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --joint-speed-rad-s\n";
        return false;
    }
    args.zero_speed_threshold = parser.getDoubleOr("zero-speed-threshold", args.zero_speed_threshold, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --zero-speed-threshold\n";
        return false;
    }
    args.const_speed_keep_ratio = parser.getDoubleOr("const-speed-keep-ratio", args.const_speed_keep_ratio, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --const-speed-keep-ratio\n";
        return false;
    }
    args.hold_position_epsilon = parser.getDoubleOr("hold-position-epsilon", args.hold_position_epsilon, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --hold-position-epsilon\n";
        return false;
    }

    args.servoj_time = parser.getDoubleOr("servoj-time", args.servoj_time, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --servoj-time\n";
        return false;
    }
    args.servoj_extrapolate_max_time = parser.getDoubleOr("servoj-extrapolate-max-time", args.servoj_extrapolate_max_time, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --servoj-extrapolate-max-time\n";
        return false;
    }
    args.servoj_decelerate_time = parser.getDoubleOr("servoj-decelerate-time", args.servoj_decelerate_time, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --servoj-decelerate-time\n";
        return false;
    }
    args.servoj_hold_velocity_threshold =
        parser.getDoubleOr("servoj-hold-velocity-threshold", args.servoj_hold_velocity_threshold, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --servoj-hold-velocity-threshold\n";
        return false;
    }
    args.servoj_hold_stable_time = parser.getDoubleOr("servoj-hold-stable-time", args.servoj_hold_stable_time, &ok);
    if (!ok) {
        std::cerr << "Invalid double for --servoj-hold-stable-time\n";
        return false;
    }

    if (args.robot_ip.empty()) {
        std::cerr << "--robot-ip is required\n";
        return false;
    }
    if (args.joint_index < 0 || args.joint_index > 5) {
        std::cerr << "--joint-index must be in [0, 5]\n";
        return false;
    }
    if (args.rtsi_frequency_hz <= 0 || args.command_duration_ms <= 0 || args.wait_after_stop_ms <= 0) {
        std::cerr << "rtsi-frequency-hz/command-duration-ms/wait-after-stop-ms must be > 0\n";
        return false;
    }
    if (args.command_timeout_ms > 0) {
        std::cerr << "This test is designed for infinite timeout. Please set --command-timeout-ms<=0\n";
        return false;
    }

    return true;
}

void ensureRobotReady(const CliArgs& args, EliteDriver& driver, DashboardClient& dashboard) {
    ELITE_LOG_INFO("Connecting dashboard...");
    if (!dashboard.connect(args.robot_ip)) {
        throw std::runtime_error("Dashboard connect failed");
    }

    ELITE_LOG_INFO("Power on...");
    if (!dashboard.powerOn()) {
        throw std::runtime_error("Power on failed");
    }

    ELITE_LOG_INFO("Brake release...");
    if (!dashboard.brakeRelease()) {
        throw std::runtime_error("Brake release failed");
    }

    if (args.use_headless_mode) {
        if (!driver.isRobotConnected()) {
            ELITE_LOG_INFO("Sending external control script...");
            if (!driver.sendExternalControlScript()) {
                throw std::runtime_error("sendExternalControlScript failed");
            }
        }
    } else {
        ELITE_LOG_INFO("Starting External Control program from dashboard...");
        if (!dashboard.playProgram()) {
            throw std::runtime_error("playProgram failed");
        }
    }

    ELITE_LOG_INFO("Waiting robot connect to reverse/trajectory/script sockets...");
    auto start = steady_clock::now();
    while (!driver.isRobotConnected()) {
        std::this_thread::sleep_for(10ms);
        if (steady_clock::now() - start > 20s) {
            throw std::runtime_error("Timed out waiting driver connection");
        }
    }
}

Sample readSample(double t_s, int cmd_active, const vector6d_t& commanded_q, RtsiIOInterface& rtsi) {
    Sample s;
    s.t_s = t_s;
    s.cmd_active = cmd_active;
    s.q_cmd = commanded_q;
    s.q = rtsi.getActualJointPositions();
    s.qd = rtsi.getActualJointVelocity();
    s.tcp = rtsi.getActualTCPPose();
    s.tcpd = rtsi.getActualTCPVelocity();
    return s;
}

void writeCsv(const std::string& path, const std::vector<Sample>& samples) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("Failed to open CSV: " + path);
    }

    ofs << "t_s,cmd_active";
    for (int i = 0; i < 6; ++i) {
        ofs << ",q_cmd" << i;
    }
    for (int i = 0; i < 6; ++i) {
        ofs << ",q" << i;
    }
    for (int i = 0; i < 6; ++i) {
        ofs << ",qd" << i;
    }
    for (int i = 0; i < 6; ++i) {
        ofs << ",tcp" << i;
    }
    for (int i = 0; i < 6; ++i) {
        ofs << ",tcpd" << i;
    }
    ofs << "\n";

    ofs << std::fixed << std::setprecision(8);
    for (const auto& s : samples) {
        ofs << s.t_s << ',' << s.cmd_active;
        for (double v : s.q_cmd) {
            ofs << ',' << v;
        }
        for (double v : s.q) {
            ofs << ',' << v;
        }
        for (double v : s.qd) {
            ofs << ',' << v;
        }
        for (double v : s.tcp) {
            ofs << ',' << v;
        }
        for (double v : s.tcpd) {
            ofs << ',' << v;
        }
        ofs << '\n';
    }
}

void writeSummaryCsv(const std::string& path, const CliArgs& args, const Metrics& m, int trace_rows) {
    bool write_header = true;
    {
        std::ifstream ifs(path);
        if (ifs.good()) {
            ifs.seekg(0, std::ios::end);
            write_header = (ifs.tellg() == 0);
        }
    }

    std::ofstream ofs(path, std::ios::app);
    if (!ofs.is_open()) {
        throw std::runtime_error("Failed to open summary CSV: " + path);
    }

    if (write_header) {
        ofs << "robot_ip,joint_index,rtsi_frequency_hz,command_duration_ms,wait_after_stop_ms,command_timeout_ms,"
               "joint_speed_rad_s,servoj_time,servoj_extrapolate_max_time,servoj_decelerate_time,"
               "servoj_hold_velocity_threshold,servoj_hold_stable_time,baseline_speed,const_velocity_duration_s,"
               "deceleration_duration_s,stable_to_lock_duration_s,stop_distance_rad,stop_error_to_last_cmd_rad,"
               "found_const_end,found_decel_end,found_lock,trace_rows\n";
    }

    ofs << std::fixed << std::setprecision(8);
    ofs << args.robot_ip << ',' << args.joint_index << ',' << args.rtsi_frequency_hz << ',' << args.command_duration_ms
        << ',' << args.wait_after_stop_ms << ',' << args.command_timeout_ms << ',' << args.joint_speed_rad_s << ','
        << args.servoj_time << ',' << args.servoj_extrapolate_max_time << ',' << args.servoj_decelerate_time << ','
        << args.servoj_hold_velocity_threshold << ',' << args.servoj_hold_stable_time << ',' << m.baseline_speed << ','
        << m.const_velocity_duration_s << ',' << m.deceleration_duration_s << ',' << m.stable_to_lock_duration_s << ','
        << m.stop_distance_rad << ',' << m.stop_error_to_last_cmd_rad << ',' << (m.found_const_end ? 1 : 0) << ','
        << (m.found_decel_end ? 1 : 0) << ',' << (m.found_lock ? 1 : 0) << ',' << trace_rows << '\n';
}

double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const size_t mid = values.size() / 2;
    if (values.size() % 2 == 0) {
        return 0.5 * (values[mid - 1] + values[mid]);
    }
    return values[mid];
}

Metrics computeMetrics(const std::vector<Sample>& trace, int joint_index, int observation_start_idx,
                      double const_speed_keep_ratio, double zero_speed_threshold, double hold_window_s,
                      double hold_position_epsilon) {
    Metrics m;
    if (trace.size() <= static_cast<size_t>(observation_start_idx + 2)) {
        return m;
    }

    const double t0 = trace[observation_start_idx].t_s;
    const double q0 = trace[observation_start_idx].q[joint_index];
    const double q_cmd0 = trace[observation_start_idx].q_cmd[joint_index];

    std::vector<double> pre_stop_speeds;
    const int pre_samples = std::min(40, observation_start_idx);
    for (int i = observation_start_idx - pre_samples; i < observation_start_idx; ++i) {
        pre_stop_speeds.push_back(std::abs(trace[i].qd[joint_index]));
    }
    m.baseline_speed = median(pre_stop_speeds);

    const double const_threshold = std::max(1e-6, m.baseline_speed * const_speed_keep_ratio);

    int idx_const_end = -1;
    for (size_t i = observation_start_idx; i < trace.size(); ++i) {
        if (std::abs(trace[i].qd[joint_index]) < const_threshold) {
            idx_const_end = static_cast<int>(i);
            break;
        }
    }
    if (idx_const_end >= 0) {
        m.found_const_end = true;
        m.const_velocity_duration_s = trace[idx_const_end].t_s - t0;
    }

    int idx_decel_end = -1;
    const int decel_search_begin = (idx_const_end >= 0) ? idx_const_end : observation_start_idx;
    for (size_t i = decel_search_begin; i < trace.size(); ++i) {
        if (std::abs(trace[i].qd[joint_index]) <= zero_speed_threshold) {
            idx_decel_end = static_cast<int>(i);
            break;
        }
    }
    if (idx_decel_end >= 0) {
        m.found_decel_end = true;
        const double t_decel_begin = (idx_const_end >= 0) ? trace[idx_const_end].t_s : t0;
        m.deceleration_duration_s = trace[idx_decel_end].t_s - t_decel_begin;
    }

    int idx_lock = -1;
    if (idx_decel_end >= 0) {
        for (size_t i = idx_decel_end; i < trace.size(); ++i) {
            const double now_t = trace[i].t_s;
            const double win_begin = now_t - hold_window_s;
            if (win_begin < trace[idx_decel_end].t_s) {
                continue;
            }

            double q_min = std::numeric_limits<double>::max();
            double q_max = std::numeric_limits<double>::lowest();
            for (int j = idx_decel_end; j <= static_cast<int>(i); ++j) {
                if (trace[j].t_s < win_begin) {
                    continue;
                }
                q_min = std::min(q_min, trace[j].q[joint_index]);
                q_max = std::max(q_max, trace[j].q[joint_index]);
            }

            if ((q_max - q_min) <= hold_position_epsilon) {
                idx_lock = static_cast<int>(i);
                break;
            }
        }
    }

    if (idx_lock >= 0) {
        m.found_lock = true;
        m.stable_to_lock_duration_s = trace[idx_lock].t_s - trace[idx_decel_end].t_s;
        m.stop_distance_rad = trace[idx_lock].q[joint_index] - q0;
        m.stop_error_to_last_cmd_rad = trace[idx_lock].q[joint_index] - q_cmd0;
    }

    return m;
}

void printMetrics(const Metrics& m, const CliArgs& args) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n===== Extrapolate Convergence Metrics =====\n";
    std::cout << "timeout_mode: " << ((args.command_timeout_ms <= 0) ? "infinite" : "finite") << "\n";
    std::cout << "baseline_speed(rad/s): " << m.baseline_speed << "\n";
    std::cout << "constant_velocity_duration_s: " << m.const_velocity_duration_s << "\n";
    std::cout << "deceleration_duration_s: " << m.deceleration_duration_s << "\n";
    std::cout << "stable_to_lock_duration_s: " << m.stable_to_lock_duration_s << "\n";
    std::cout << "stop_distance_rad: " << m.stop_distance_rad << "\n";
    std::cout << "stop_error_to_last_cmd_rad: " << m.stop_error_to_last_cmd_rad << "\n";

    if (!m.found_const_end) {
        std::cout << "WARN: constant-speed phase end not detected.\n";
    }
    if (!m.found_decel_end) {
        std::cout << "WARN: deceleration-to-zero not detected.\n";
    }
    if (!m.found_lock) {
        std::cout << "WARN: hold lock point not detected in observation window.\n";
    }
    std::cout << "==========================================\n\n";
}

}  // namespace

int main(int argc, char** argv) {
#if defined(__linux) || defined(linux) || defined(__linux__)
    mlockall(MCL_CURRENT | MCL_FUTURE);
    pthread_t handle = pthread_self();
    RT_UTILS::setThreadFiFoScheduling(handle, RT_UTILS::getThreadFiFoMaxPriority());
    RT_UTILS::bindThreadToCpus(handle, 2);
#endif

    CliArgs args;
    bool help_requested = false;
    if (!parseArgs(argc, argv, args, &help_requested)) {
        return help_requested ? 0 : 1;
    }

    EliteDriverConfig config;
    config.robot_ip = args.robot_ip;
    config.local_ip = args.local_ip;
    config.headless_mode = args.use_headless_mode;
    config.script_file_path = "external_control.script";
    config.servoj_time = static_cast<float>(args.servoj_time);
    config.servoj_extrapolate_max_time = static_cast<float>(args.servoj_extrapolate_max_time);
    config.servoj_decelerate_time = static_cast<float>(args.servoj_decelerate_time);
    config.servoj_hold_velocity_threshold = static_cast<float>(args.servoj_hold_velocity_threshold);
    config.servoj_hold_stable_time = static_cast<float>(args.servoj_hold_stable_time);

    std::unique_ptr<EliteDriver> driver;
    std::unique_ptr<DashboardClient> dashboard;
    std::unique_ptr<RtsiIOInterface> rtsi;
    std::vector<Sample> trace;

    try {
        driver = std::make_unique<EliteDriver>(config);
        dashboard = std::make_unique<DashboardClient>();
        rtsi = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", args.rtsi_frequency_hz);

        ELITE_LOG_INFO("Connecting RTSI...");
        if (!rtsi->connect(args.robot_ip)) {
            throw std::runtime_error("RTSI connect failed");
        }

        ensureRobotReady(args, *driver, *dashboard);

        const auto dt = duration<double>(1.0 / static_cast<double>(args.rtsi_frequency_hz));
        const auto dt_chrono = duration_cast<steady_clock::duration>(dt);

        vector6d_t target_q = rtsi->getActualJointPositions();
        const double delta_per_step = args.joint_speed_rad_s / static_cast<double>(args.rtsi_frequency_hz);
        const int cmd_samples = std::max(1, args.command_duration_ms * args.rtsi_frequency_hz / 1000);
        const int wait_samples = std::max(1, args.wait_after_stop_ms * args.rtsi_frequency_hz / 1000);

        auto next_tick = steady_clock::now();

        ELITE_LOG_INFO("Phase 1: command joint %d at speed %.6f rad/s for %d ms with timeout=%d (<=0 means infinite)",
                       args.joint_index, args.joint_speed_rad_s, args.command_duration_ms, args.command_timeout_ms);
        for (int i = 0; i < cmd_samples; ++i) {
            target_q[args.joint_index] += delta_per_step;
            if (!driver->writeServoj(target_q, args.command_timeout_ms, false)) {
                throw std::runtime_error("writeServoj failed during command phase");
            }

            const double t_s = static_cast<double>(i) / static_cast<double>(args.rtsi_frequency_hz);
            trace.push_back(readSample(t_s, 1, target_q, *rtsi));

            next_tick += dt_chrono;
            std::this_thread::sleep_until(next_tick);
        }

        const int observation_start_idx = static_cast<int>(trace.size() - 1);

        ELITE_LOG_INFO("Phase 2: stop sending new points, keep tracing for %d ms", args.wait_after_stop_ms);
        for (int i = 0; i < wait_samples; ++i) {
            const double t_s = static_cast<double>(cmd_samples + i) / static_cast<double>(args.rtsi_frequency_hz);
            trace.push_back(readSample(t_s, 0, target_q, *rtsi));

            next_tick += dt_chrono;
            std::this_thread::sleep_until(next_tick);
        }

        const Metrics metrics = computeMetrics(trace, args.joint_index, observation_start_idx, args.const_speed_keep_ratio,
                                               args.zero_speed_threshold, args.servoj_hold_stable_time,
                                               args.hold_position_epsilon);

        writeCsv(args.output_csv, trace);
        writeSummaryCsv(args.summary_csv, args, metrics, static_cast<int>(trace.size()));
        printMetrics(metrics, args);

        std::cout << "CSV trace written to: " << args.output_csv << "\n";
        std::cout << "CSV summary written to: " << args.summary_csv << "\n";

        driver->stopControl();
        rtsi->disconnect();

        if (!(metrics.found_const_end && metrics.found_decel_end)) {
            return 2;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << "\n";
        if (driver) {
            driver->stopControl();
        }
        if (rtsi) {
            rtsi->disconnect();
        }
        return 1;
    }
}
