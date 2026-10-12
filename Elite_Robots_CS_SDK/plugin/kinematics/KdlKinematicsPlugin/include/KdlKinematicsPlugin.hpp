// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// KdlKinematicsPlugin.hpp
// KDL kinematics plugin
#ifndef __ELITE__KDL_KINEMATICS_PLUGIN_HPP__
#define __ELITE__KDL_KINEMATICS_PLUGIN_HPP__

#include <mutex>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

// Elite
#include <Elite/KinematicsBase.hpp>
#include <Elite/ClassLoader.hpp>
#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>

namespace ELITE {

class KdlKinematicsPlugin : public KinematicsBase
{
private:
    vector6d_t dh_alpha_;
    vector6d_t dh_a_;
    vector6d_t dh_d_;

    mutable std::mutex mutex_;

    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::Chain> robot_chain_;

    KDL::JntArray convertToKDLJoints(const ELITE::vector6d_t &joints) const;
public:
    /**
     * @brief Set robot MDH parameter
     * 
     * @param alpha MDH alpha parameter
     * @param a MDH a parameter
     * @param d MDH d parameter
     */
    ELITE_EXPORT virtual void setMDH(const vector6d_t& alpha, const vector6d_t& a, const vector6d_t& d);

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose
     * 
     * @param joint_angles The state for which FK is being computed
     * @param poses The resultant set of poses.
     * @return True if a valid solution was found, false otherwise
     */
    ELITE_EXPORT virtual bool getPositionFK(const vector6d_t& joint_angles, vector6d_t& poses) const;

    /**
     * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
     * 
     * In contrast to the searchPositionIK methods, this one is expected to return the solution
     * closest to the seed state. Randomly re-seeding is explicitly not allowed.
     * @param pose the desired pose of the link
     * @param near an initial guess solution for the inverse kinematics
     * @param solution the solution vector
     * @param result A struct that reports the results of the query
     * @return True if a valid set of solutions was found, false otherwise.
     */
    ELITE_EXPORT virtual bool getPositionIK(const vector6d_t& pose, const vector6d_t& near, vector6d_t& solution, KinematicsResult& result) const;

    /**
     * @brief Get the Position I K object
     * 
     * @param pose The desired pose of each tip link
     * @param near an initial guess solution for the inverse kinematics
     * @param solutions A vector of valid joint vectors. This return has two variant behaviors:
     *                  1) Return a joint solution for every input |pose|, e.g. multi-arm support
     *                  2) Return multiple joint solutions for a single |pose| input, e.g. underconstrained IK
     *                  TODO(dave): This dual behavior is confusing and should be changed in a future refactor of this API
     * @param result A struct that reports the results of the query
     * @return True if a valid set of solutions was found, false otherwise.
     */
    ELITE_EXPORT virtual bool getPositionIK(const vector6d_t& pose, const vector6d_t& near, std::vector<vector6d_t>& solutions,
                               KinematicsResult& result) const;
    KdlKinematicsPlugin();
    ~KdlKinematicsPlugin();
};

} // namespace ELITE

#endif  // __ELITE__KDL_KINEMATICS_PLUGIN_HPP__