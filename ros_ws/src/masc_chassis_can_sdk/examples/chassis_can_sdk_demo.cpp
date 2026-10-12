#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "masc_chassis_can_sdk/chassis_sdk.hpp"

namespace {

std::atomic_bool g_should_stop{false};

void HandleSignal(int signal_number) {
    (void)signal_number;
    g_should_stop = true;
}

void PrintUsage(const char *program_name) {
    std::cout << "Usage: " << program_name << " [can_interface] [run_seconds]\n"
              << "Example: " << program_name << " can0 10\n\n"
              << "This demo connects to one MASC chassis, keeps sending a zero\n"
              << "motion_control_cmd (0x100), and prints decoded feedback/health snapshots\n"
              << "once per second.\n";
}

int ParseRunSeconds(const char *text, int fallback) {
    if (text == nullptr) {
        return fallback;
    }

    const int value = std::atoi(text);
    if (value <= 0) {
        return fallback;
    }
    return value;
}

void PrintVersionInfo(const masc_chassis_can_sdk::VersionInfo &version_info) {
    std::cout << "Connected. UUID=";
    for (std::size_t index = 0; index < version_info.m_uuid.size(); ++index) {
        if (index != 0) {
            std::cout << ":";
        }
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(version_info.m_uuid[index]);
    }
    std::cout << std::dec << std::setfill(' ') << " sys_sw_hw=0x" << std::hex
              << static_cast<int>(version_info.m_system_software_hardware_version)
              << " chassis_type=0x" << static_cast<int>(version_info.m_chassis_type_version)
              << " proto=0x"
              << static_cast<int>(version_info.m_protocol_version) << std::dec << "\n";
}

void PrintFeedbackSnapshot(masc_chassis_can_sdk::ChassisSdk *sdk) {
    const auto motion = sdk->GetMotionFeedback();
    if (motion.has_value()) {
        std::cout << "motion x=" << motion->m_current_linear_x_mps
                  << " m/s y=" << motion->m_current_linear_y_mps
                  << " m/s wz=" << motion->m_current_angular_z_radps
                  << " rad/s steer=" << motion->m_current_steering_angle_rad << " rad\n";
    } else {
        std::cout << "motion feedback: not received yet\n";
    }

    const auto system = sdk->GetSystemFeedback();
    if (system.has_value()) {
        std::cout << "system_state=0x" << std::hex
                  << static_cast<int>(system->m_system_state_raw)
                  << " status_flags=0x" << static_cast<int>(system->m_system_status_flags)
                  << " obstacle_flags=0x" << static_cast<int>(system->m_obstacle_status_flags)
                  << " raw_status=0x" << system->m_raw_system_status
                  << " system_fault=0x" << system->m_system_fault << std::dec << "\n";
    }

    const auto bms = sdk->GetBmsBasicFeedback();
    if (bms.has_value()) {
        std::cout << "battery soc=" << static_cast<int>(bms->m_battery_soc_percent)
                  << "% voltage=" << bms->m_battery_voltage_v
                  << " V current=" << bms->m_battery_current_a << " A\n";
    }

    const auto imu = sdk->GetImuFeedback();
    if (imu.has_value()) {
        std::cout << "imu pitch=" << imu->m_pitch_rad << " rad yaw=" << imu->m_yaw_rad
                  << " rad roll=" << imu->m_roll_rad
                  << " rad temp=" << imu->m_temperature_deg_c << " degC\n";
    }

    const auto rc = sdk->GetRcFeedback();
    if (rc.has_value()) {
        std::cout << "rc online=" << rc->m_online << " channels=";
        for (std::size_t index = 0; index < rc->m_channels.size(); ++index) {
            if (index != 0) {
                std::cout << ",";
            }
            std::cout << static_cast<int>(rc->m_channels[index]);
        }
        std::cout << "\n";
    }

    const auto health = sdk->GetHealthSnapshot();
    std::cout << "health connected=" << health.m_connected
              << " motion_timeout=" << health.m_motion_command_timeout_active
              << " transport_errors=" << health.m_transport_error_count
              << " tx_errors=" << health.m_tx_error_count
              << " rx_errors=" << health.m_rx_error_count << "\n";
}

}  // namespace

int main(int argc, char **argv) {
    if (argc > 1 && std::string(argv[1]) == "--help") {
        PrintUsage(argv[0]);
        return 0;
    }

    const std::string can_interface = argc > 1 ? argv[1] : "can0";
    const int run_seconds = argc > 2 ? ParseRunSeconds(argv[2], 10) : 10;

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    masc_chassis_can_sdk::CanConfig config;
    config.m_interface_name = can_interface;
    config.m_bitrate = 500000;
    config.m_connect_timeout_ms = 1000;
    config.m_motion_command_period_ms = 20;
    config.m_motion_command_timeout_ms = 100;

    masc_chassis_can_sdk::ChassisSdk sdk;
    if (!sdk.Configure(config)) {
        std::cerr << "Failed to configure SDK. Is it already connected?\n";
        return 1;
    }

    // Callbacks run in the SDK receive dispatch path. Keep them lightweight.
    std::atomic_uint64_t motion_callback_count{0};
    std::atomic_uint64_t system_callback_count{0};
    std::atomic_uint64_t imu_callback_count{0};
    std::atomic_uint64_t rc_callback_count{0};
    sdk.SetMotionFeedbackCallback([&motion_callback_count](const auto &feedback) {
        (void)feedback;
        motion_callback_count++;
    });
    sdk.SetSystemFeedbackCallback([&system_callback_count](const auto &feedback) {
        (void)feedback;
        system_callback_count++;
    });
    sdk.SetImuFeedbackCallback([&imu_callback_count](const auto &feedback) {
        (void)feedback;
        imu_callback_count++;
    });
    sdk.SetRcFeedbackCallback([&rc_callback_count](const auto &feedback) {
        (void)feedback;
        rc_callback_count++;
    });

    std::cout << "Opening " << can_interface
              << " and waiting for system_version_feedback (0x205)...\n";
    if (!sdk.Connect()) {
        const auto health = sdk.GetHealthSnapshot();
        std::cerr << "Connect failed. connection_attempts=" << health.m_connection_attempt_count
                  << " transport_errors=" << health.m_transport_error_count
                  << " expected_protocol=0x" << std::hex
                  << static_cast<int>(config.m_expected_protocol_version) << std::dec << "\n";
        return 1;
    }

    const auto version_info = sdk.GetVersionInfo();
    if (version_info.has_value()) {
        PrintVersionInfo(*version_info);
    }

    // This customer demo intentionally sends zero motion_control_cmd (0x100) only.
    // Replace these fields in your application after you have verified chassis mode, safety state,
    // and operating area.
    masc_chassis_can_sdk::MotionCommand zero_motion;

    const auto start_time = std::chrono::steady_clock::now();
    auto last_print_time = start_time;

    while (!g_should_stop) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        if (elapsed_seconds >= run_seconds) {
            break;
        }

        // Refresh the command faster than m_motion_command_timeout_ms so the SDK watchdog stays
        // inactive during normal operation.
        sdk.SetMotionCommand(zero_motion);

        if (now - last_print_time >= std::chrono::seconds(1)) {
            std::cout << "\n--- SDK snapshot ---\n";
            PrintFeedbackSnapshot(&sdk);
            std::cout << "callbacks motion=" << motion_callback_count.load()
                      << " system=" << system_callback_count.load()
                      << " imu=" << imu_callback_count.load()
                      << " rc=" << rc_callback_count.load() << "\n";
            last_print_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    sdk.Disconnect();
    std::cout << "Demo finished safely.\n";
    return 0;
}
