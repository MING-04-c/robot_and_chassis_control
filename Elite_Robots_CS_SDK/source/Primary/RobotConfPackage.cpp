// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "RobotConfPackage.hpp"
#include "Utils.hpp"

/** In controller version 2.11.0, the sub package is:
    uint32_t	configuration_sub_len
    uint8_t	configuration_sub_type
    foreach joint:		
    double	limit_min_joint_x
    double	limit_max_joint_x
    end

    foreach joint:		
    double	max_velocity_joint_x
    double	max_acc_joint_x
    end

    double	default_velocity_joint
    double	default_acc_joint
    double	default_tool_velocity
    double	default_tool_acc
    double	eq_radius

    foreach joint:		
    double	dh_a_joint_x
    end

    foreach joint:		
    double	dh_d_joint_d
    end

    foreach joint:	
    double	dh_alpha_joint_x
    end

    foreach joint:
    double	reserver
    end

    uint32_t	board_version
    uint32_t	control_box_type
    uint32_t	robot_type
    uint32_t	robot_struct
 */

namespace ELITE
{


void KinematicsInfo::parser(int len, const std::vector<uint8_t>::const_iterator& iter) {
    int offset = DH_PARAM_OFFSET;
    for (size_t i = 0; i < 6; i++) {
        EndianUtils::unpack(iter + offset, dh_a_[i]);
        offset += sizeof(double);
    }
    for (size_t i = 0; i < 6; i++) {
        EndianUtils::unpack(iter + offset, dh_d_[i]);
        offset += sizeof(double);
    }
    for (size_t i = 0; i < 6; i++) {
        EndianUtils::unpack(iter + offset, dh_alpha_[i]);
        offset += sizeof(double);
    }
}



} // namespace ELITE

