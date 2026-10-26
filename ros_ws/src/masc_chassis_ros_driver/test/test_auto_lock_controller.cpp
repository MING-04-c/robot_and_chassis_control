#include <gtest/gtest.h>

#include <cstdint>

#include "masc_chassis_ros_driver/auto_lock_controller.hpp"

namespace masc_chassis_ros_driver {
namespace {
using masc_chassis_can_sdk::MotionFeedback;
using masc_chassis_can_sdk::MotionMode;
using masc_chassis_can_sdk::MotionModeFeedback;

AutoLockUpdateInput Input(MotionMode mode, int tick) {
    AutoLockUpdateInput input;
    input.m_motion_mode_switch_supported = true;
    input.m_auto_lock_allowed = true;
    input.m_motion_feedback = MotionFeedback{};
    input.m_motion_mode_feedback = MotionModeFeedback{mode, false};
    input.m_now = RosTime(static_cast<std::int64_t>(tick) * 20000000);
    return input;
}

TEST(AutoLockController, LowSpeedAccumulatesAndUnlocksAfterIdle) {
    for (const double speed : {0.025, -0.025}) {
        AutoLockController controller;
        controller.Configure(AutoLockConfig{});
        controller.Update(Input(MotionMode::Park, 0));
        bool requested = false;
        for (int tick = 1; tick <= 30; ++tick) {
            auto input = Input(MotionMode::Park, tick);
            input.m_command.m_target_linear_x_mps = speed;
            const auto result = controller.Update(input);
            // No movement is sent before mode feedback confirms unlocking.
            EXPECT_DOUBLE_EQ(result.m_command.m_target_linear_x_mps, 0.0);
            if (result.m_action == AutoLockAction::Unlock) {
                ASSERT_TRUE(result.m_motion_mode_request.has_value());
                EXPECT_EQ(*result.m_motion_mode_request, MotionMode::DefaultChassis);
                requested = true;
                break;
            }
        }
        EXPECT_TRUE(requested) << "low-speed command was repeatedly erased";
        auto confirmed = Input(MotionMode::DefaultChassis, 31);
        confirmed.m_command.m_target_linear_x_mps = speed;
        EXPECT_DOUBLE_EQ(controller.Update(confirmed).m_command.m_target_linear_x_mps, speed);
    }
}

TEST(AutoLockController, LowAngularCommandAccumulatesAndUnlocks) {
    AutoLockController controller;
    controller.Configure(AutoLockConfig{});
    controller.Update(Input(MotionMode::Park, 0));
    bool requested = false;
    for (int tick = 1; tick <= 30; ++tick) {
        auto input = Input(MotionMode::Park, tick);
        input.m_command.m_target_angular_z_radps = 0.025;
        requested |= controller.Update(input).m_action == AutoLockAction::Unlock;
    }
    EXPECT_TRUE(requested);
}

TEST(AutoLockController, BelowThresholdNoiseDoesNotUnlock) {
    AutoLockController controller;
    controller.Configure(AutoLockConfig{});
    controller.Update(Input(MotionMode::Park, 0));
    for (int tick = 1; tick <= 200; ++tick) {
        auto input = Input(MotionMode::Park, tick);
        input.m_command.m_target_linear_x_mps = 0.005;
        EXPECT_EQ(controller.Update(input).m_action, AutoLockAction::None);
    }
}

TEST(AutoLockController, LocksAfterFilteredCommandAndFeedbackStop) {
    AutoLockController controller;
    controller.Configure(AutoLockConfig{});
    auto moving = Input(MotionMode::DefaultChassis, 0);
    moving.m_command.m_target_linear_x_mps = 0.025;
    controller.Update(moving);
    for (int tick = 1; tick < 100; ++tick) {
        auto input = Input(MotionMode::DefaultChassis, tick);
        input.m_motion_feedback->m_current_linear_x_mps = 0.1;
        EXPECT_EQ(controller.Update(input).m_action, AutoLockAction::None);
    }
    bool locked = false;
    for (int tick = 100; tick <= 260; ++tick) {
        const auto result = controller.Update(Input(MotionMode::DefaultChassis, tick));
        if (tick < 250) {
            EXPECT_EQ(result.m_action, AutoLockAction::None);
        }
        locked |= result.m_action == AutoLockAction::Lock;
    }
    EXPECT_TRUE(locked);
}
}  // namespace
}  // namespace masc_chassis_ros_driver
