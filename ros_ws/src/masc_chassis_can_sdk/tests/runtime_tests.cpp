#include <atomic>
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "masc_chassis_can_sdk/chassis_protocol.hpp"
#include "masc_chassis_can_sdk/chassis_sdk.hpp"

namespace {

using namespace masc_chassis_can_sdk;

class MockTransport : public ICanTransport {
public:
    bool Open(const CanConfig &config) override {
        (void)config;
        std::lock_guard<std::mutex> lock(m_mutex);
        m_open = true;
        return true;
    }

    void Close() override {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_open = false;
        }
        m_cv.notify_all();
    }

    bool IsOpen() const override {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_open;
    }

    CanReadStatus ReadFrame(CanFrame *frame, int timeout_ms) override {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (!m_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
                return !m_rx_queue.empty() || !m_open;
            })) {
            return CanReadStatus::Timeout;
        }
        if (!m_open) {
            return CanReadStatus::Error;
        }
        *frame = m_rx_queue.front();
        m_rx_queue.pop();
        return CanReadStatus::Frame;
    }

    bool WriteFrame(const CanFrame &frame) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_open) {
            return false;
        }
        m_tx_queue.push(frame);
        m_cv.notify_all();
        return true;
    }

    std::string GetLastError() const override {
        return m_last_error;
    }

    void PushRx(const CanFrame &frame) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_rx_queue.push(frame);
        }
        m_cv.notify_all();
    }

    bool WaitForTx(CanFrame *frame, int timeout_ms) {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (!m_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
                return !m_tx_queue.empty();
            })) {
            return false;
        }
        *frame = m_tx_queue.front();
        m_tx_queue.pop();
        return true;
    }

private:
    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_open = false;
    std::queue<CanFrame> m_rx_queue;
    std::queue<CanFrame> m_tx_queue;
    std::string m_last_error;
};

CanFrame MakeVersionFrame(std::uint8_t protocol_version,
                          std::uint8_t chassis_type_version = 0x61) {
    CanFrame frame;
    frame.m_id = chassis_protocol::kVersionFeedbackId;
    frame.m_dlc = kCanDlc;
    frame.m_data = {0x01, 0x02, 0x03, 0x04,
                    0x05, 0x10, chassis_type_version, protocol_version};
    return frame;
}

CanFrame MakeSystemFrame() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kSystemFeedbackId;
    frame.m_dlc = chassis_protocol::kSystemFeedbackDlc;
    frame.m_data = {0x06, 0x1E, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12};
    return frame;
}

CanFrame MakeMotionFrame() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kMotionFeedbackId;
    frame.m_dlc = kCanDlc;
    frame.m_data = {0xF4, 0x01, 0x00, 0x00, 0x7A, 0x04, 0x00, 0x00};
    return frame;
}

CanFrame MakeRawFrame(std::uint16_t can_id) {
    CanFrame frame;
    frame.m_id = can_id;
    frame.m_dlc = kCanDlc;
    frame.m_data = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
    return frame;
}

CanFrame MakeImuFrame() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kImuFeedbackId;
    frame.m_dlc = chassis_protocol::kImuFeedbackDlc;
    frame.m_data = {0xE8, 0x03, 0x18, 0xFC, 0xFA, 0x00, 0xFB, 0x00};
    return frame;
}

CanFrame MakeRcFrame() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kRcFeedbackId;
    frame.m_dlc = chassis_protocol::kRcFeedbackDlc;
    frame.m_data = {0x01, 0x9C, 0xCE, 0x00, 0x32, 0x64, 0xFF, 0x01};
    return frame;
}

void TestConnectSuccessAndCallback() {
    auto transport = std::make_unique<MockTransport>();
    auto *transport_ptr = transport.get();
    ChassisSdk sdk(std::move(transport));

    int callback_count = 0;
    int system_callback_count = 0;
    int imu_callback_count = 0;
    int rc_callback_count = 0;
    sdk.SetMotionFeedbackCallback([&callback_count](const MotionFeedback &feedback) {
        assert(feedback.m_current_linear_x_mps == 0.5);
        callback_count++;
    });
    sdk.SetSystemFeedbackCallback([&system_callback_count](const SystemFeedback &feedback) {
        assert(feedback.m_system_state == SystemState::CommControl);
        assert(feedback.m_system_status_flags == 0x1E);
        assert(feedback.m_system_fault == 0x12345678);
        system_callback_count++;
    });
    sdk.SetImuFeedbackCallback([&imu_callback_count](const ImuFeedback &feedback) {
        assert(feedback.m_temperature_deg_c > 25.09);
        assert(feedback.m_temperature_deg_c < 25.11);
        imu_callback_count++;
    });
    sdk.SetRcFeedbackCallback([&rc_callback_count](const RcFeedback &feedback) {
        assert(feedback.m_online);
        assert(feedback.m_channels[0] == -100);
        rc_callback_count++;
    });

    std::thread producer([transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeVersionFrame(kProtocolVersionV10));
        transport_ptr->PushRx(MakeSystemFrame());
        transport_ptr->PushRx(MakeMotionFrame());
        transport_ptr->PushRx(MakeImuFrame());
        transport_ptr->PushRx(MakeRcFrame());
    });

    CanConfig config;
    config.m_connect_timeout_ms = 500;
    assert(sdk.Configure(config));
    assert(sdk.Connect());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto version = sdk.GetVersionInfo();
    assert(version.has_value());
    assert(version->m_protocol_version == kProtocolVersionV10);
    assert(sdk.GetSystemFeedback().has_value());
    assert(sdk.GetMotionFeedback().has_value());
    assert(sdk.GetImuFeedback().has_value());
    assert(sdk.GetRcFeedback().has_value());
    assert(callback_count == 1);
    assert(system_callback_count == 1);
    assert(imu_callback_count == 1);
    assert(rc_callback_count == 1);

    sdk.Disconnect();
    producer.join();
}

void TestRawFrameCallbackForUnsupportedFrame() {
    auto transport = std::make_unique<MockTransport>();
    auto *transport_ptr = transport.get();
    ChassisSdk sdk(std::move(transport));

    std::atomic_int raw_callback_count{0};
    sdk.SetRawFrameCallback([&raw_callback_count](const CanFrame &frame) {
        if (frame.m_id == 0x321) {
            raw_callback_count++;
        }
    });

    std::thread producer([transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeVersionFrame(kProtocolVersionV10));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeRawFrame(0x321));
    });

    CanConfig config;
    config.m_connect_timeout_ms = 500;
    assert(sdk.Configure(config));
    assert(sdk.Connect());

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    assert(raw_callback_count == 1);
    const auto stats = sdk.GetFrameStats(0x321);
    assert(stats.has_value());
    assert(stats->m_drop_count == 1);

    sdk.Disconnect();
    producer.join();
}

void TestConnectVersionMismatch() {
    auto transport = std::make_unique<MockTransport>();
    auto *transport_ptr = transport.get();
    ChassisSdk sdk(std::move(transport));

    std::thread producer([transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeVersionFrame(0x01));
    });

    CanConfig config;
    config.m_connect_timeout_ms = 500;
    assert(sdk.Configure(config));
    assert(!sdk.Connect());
    assert(!sdk.IsConnected());
    producer.join();
}

void TestSendRawFrame() {
    auto transport = std::make_unique<MockTransport>();
    auto *transport_ptr = transport.get();
    ChassisSdk sdk(std::move(transport));

    std::thread producer([transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeVersionFrame(kProtocolVersionV10));
    });

    CanConfig config;
    config.m_connect_timeout_ms = 500;
    config.m_motion_command_period_ms = 1000;
    assert(sdk.Configure(config));
    assert(sdk.Connect());

    const auto raw_frame = MakeRawFrame(0x123);
    assert(sdk.SendRawFrame(raw_frame));

    CanFrame tx_frame;
    bool saw_raw_frame = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (transport_ptr->WaitForTx(&tx_frame, 50) && tx_frame.m_id == raw_frame.m_id) {
            saw_raw_frame = true;
            break;
        }
    }
    assert(saw_raw_frame);

    auto invalid_frame = raw_frame;
    invalid_frame.m_dlc = 9;
    assert(!sdk.SendRawFrame(invalid_frame));
    assert(sdk.GetHealthSnapshot().m_tx_error_count > 0);

    sdk.Disconnect();
    producer.join();
}

void TestSendTowMode() {
    auto transport = std::make_unique<MockTransport>();
    auto *transport_ptr = transport.get();
    ChassisSdk sdk(std::move(transport));

    std::thread producer([transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeVersionFrame(kProtocolVersionV10));
    });

    CanConfig config;
    config.m_connect_timeout_ms = 500;
    config.m_motion_command_period_ms = 1000;
    assert(sdk.Configure(config));
    assert(sdk.Connect());

    assert(sdk.SendTowMode(true));

    CanFrame tx_frame;
    bool saw_tow_frame = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (transport_ptr->WaitForTx(&tx_frame, 50) &&
            tx_frame.m_id == chassis_protocol::kTowModeCmdId) {
            saw_tow_frame = true;
            assert(tx_frame.m_dlc == chassis_protocol::kTowModeCmdDlc);
            assert(tx_frame.m_data[0] == 0x01);
            break;
        }
    }
    assert(saw_tow_frame);

    assert(sdk.SendTowMode(false));
    saw_tow_frame = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (transport_ptr->WaitForTx(&tx_frame, 50) &&
            tx_frame.m_id == chassis_protocol::kTowModeCmdId) {
            saw_tow_frame = true;
            assert(tx_frame.m_data[0] == 0x00);
            break;
        }
    }
    assert(saw_tow_frame);

    sdk.Disconnect();
    producer.join();
}

void TestMotionModeSupportByChassisType() {
    auto transport = std::make_unique<MockTransport>();
    auto *transport_ptr = transport.get();
    ChassisSdk sdk(std::move(transport));

    std::thread producer([transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeVersionFrame(kProtocolVersionV10, 0x61));
    });

    CanConfig config;
    config.m_connect_timeout_ms = 500;
    config.m_motion_command_period_ms = 1000;
    assert(sdk.Configure(config));
    assert(sdk.Connect());
    assert(sdk.IsMotionModeSwitchSupported());
    assert(sdk.SendMotionMode(MotionMode::Park));

    CanFrame tx_frame;
    bool saw_mode_frame = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (transport_ptr->WaitForTx(&tx_frame, 50) &&
            tx_frame.m_id == chassis_protocol::kModeSwitchCmdId) {
            saw_mode_frame = true;
            assert(tx_frame.m_data[0] == 0x01);
            break;
        }
    }
    assert(saw_mode_frame);

    sdk.Disconnect();
    producer.join();

    auto unsupported_transport = std::make_unique<MockTransport>();
    auto *unsupported_transport_ptr = unsupported_transport.get();
    ChassisSdk unsupported_sdk(std::move(unsupported_transport));

    std::thread unsupported_producer([unsupported_transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        unsupported_transport_ptr->PushRx(MakeVersionFrame(kProtocolVersionV10, 0x60));
    });

    assert(unsupported_sdk.Configure(config));
    assert(unsupported_sdk.Connect());
    assert(!unsupported_sdk.IsMotionModeSwitchSupported());
    assert(!unsupported_sdk.SendMotionMode(MotionMode::Park));

    unsupported_sdk.Disconnect();
    unsupported_producer.join();
}

void TestMotionCommandTimeoutZeroing() {
    auto transport = std::make_unique<MockTransport>();
    auto *transport_ptr = transport.get();
    ChassisSdk sdk(std::move(transport));

    std::thread producer([transport_ptr]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        transport_ptr->PushRx(MakeVersionFrame(kProtocolVersionV10));
    });

    CanConfig config;
    config.m_connect_timeout_ms = 500;
    config.m_motion_command_period_ms = 10;
    config.m_motion_command_timeout_ms = 30;
    assert(sdk.Configure(config));
    assert(sdk.Connect());

    MotionCommand command;
    command.m_target_linear_x_mps = 0.5;
    sdk.SetMotionCommand(command);

    CanFrame frame;
    assert(transport_ptr->WaitForTx(&frame, 100));

    bool saw_zero_command = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < deadline) {
        if (transport_ptr->WaitForTx(&frame, 50)) {
            if (frame.m_id == chassis_protocol::kMotionControlCmdId && frame.m_data[0] == 0x00 &&
                frame.m_data[1] == 0x00) {
                saw_zero_command = true;
                break;
            }
        }
    }
    assert(saw_zero_command);
    assert(sdk.GetHealthSnapshot().m_motion_command_timeout_active);

    sdk.Disconnect();
    producer.join();
}

}  // namespace

int main() {
    TestConnectSuccessAndCallback();
    TestRawFrameCallbackForUnsupportedFrame();
    TestConnectVersionMismatch();
    TestSendRawFrame();
    TestSendTowMode();
    TestMotionModeSupportByChassisType();
    TestMotionCommandTimeoutZeroing();
    return 0;
}
