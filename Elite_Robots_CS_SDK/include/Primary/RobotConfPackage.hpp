// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// RobotConfPackage.hpp
// Provides the RobotConfPackage class for handling robot configuration data packages.
#ifndef __ELITE__ROBOT_CONFIGURE_PACKAGE_HPP__
#define __ELITE__ROBOT_CONFIGURE_PACKAGE_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>
#include <Elite/PrimaryPackage.hpp>
#include <cstdint>

namespace ELITE {

/**
 * @brief The RobotConfig message in the robot's primary port
 *
 */
class RobotConfPackage : public PrimaryPackage {
   private:
    /// The Robot configure sub-package type
    static constexpr int ROBOT_CONFIG_PKG_TYPE = 6;

   public:
    /**
     * @brief Construct a new Robot Conf Package object
     *
     */
    ELITE_EXPORT RobotConfPackage() : PrimaryPackage(ROBOT_CONFIG_PKG_TYPE) {}
    ELITE_EXPORT virtual ~RobotConfPackage() = default;
};

/**
 * @brief The robot kinematics infomation in RobotConfig message
 *
 */
class KinematicsInfo : public RobotConfPackage {
   private:
    // In version 2.11.0 of robot controller, the DH parameters are located at a specific offset relative to the sub-header of the
    // message.
    static constexpr int DH_PARAM_OFFSET =
        sizeof(uint32_t) + sizeof(uint8_t) + sizeof(double) * 2 * 6 + sizeof(double) * 2 * 6 + sizeof(double) * 5;

   public:
    ELITE_EXPORT KinematicsInfo() = default;
    ELITE_EXPORT ~KinematicsInfo() = default;

    vector6d_t dh_a_;
    vector6d_t dh_d_;
    vector6d_t dh_alpha_;

    /**
     * @brief Parser message from robot. Internal use.
     *
     * @param len The len of sub-package
     * @param iter Position of the sub-package in the entire package
     */
    ELITE_EXPORT void parser(int len, const std::vector<uint8_t>::const_iterator& iter);
};

}  // namespace ELITE

#endif
