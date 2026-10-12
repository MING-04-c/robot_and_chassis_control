// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// KinematicsBase.hpp
// Provides an interface for kinematics solvers.
#ifndef __ELITE__KINEMATICS_BASE_HPP__
#define __ELITE__KINEMATICS_BASE_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>
#include <vector>
#include <memory>

namespace ELITE {


/*
 * @brief Kinematic error codes that occur in a ik query
 */
enum KinematicError {
    OK = 1,                               // No errors
    SOLVER_NOT_ACTIVE,                    // Solver isn't active
    NO_SOLUTION                           // A valid joint solution that can reach this pose(s) could not be found
};

/*
 * @brief Reports result details of an ik query
 *
 * This struct is used as an output argument of the getPositionIK(...) method that returns multiple joint solutions.
 * It contains the type of error that led to a failure or KinematicErrors::OK when a set of joint solutions is found.
 * The solution percentage shall provide a ratio of solutions found over solutions searched.
 *
 */
struct KinematicsResult {
    KinematicError kinematic_error;  // Error code that indicates the type of failure
};


/**
 * @brief Provides an interface for kinematics solvers.
 */
class KinematicsBase {
   public:
    static constexpr double DEFAULT_TIMEOUT = 1.0; // seconds

    KinematicsBase() : default_timeout_(DEFAULT_TIMEOUT) {}

    virtual ~KinematicsBase() = default;

    /**
     * @brief Set robot MDH parameter
     * 
     * @param alpha MDH alpha parameter
     * @param a MDH a parameter
     * @param d MDH d parameter
     */
    ELITE_EXPORT virtual void setMDH(const vector6d_t& alpha, const vector6d_t& a, const vector6d_t& d) = 0;

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose
     * 
     * @param joint_angles The state for which FK is being computed
     * @param poses The resultant set of poses.
     * @return True if a valid solution was found, false otherwise
     */
    ELITE_EXPORT virtual bool getPositionFK(const vector6d_t& joint_angles, vector6d_t& poses) const = 0;

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
    ELITE_EXPORT virtual bool getPositionIK(const vector6d_t& pose, const vector6d_t& near, vector6d_t& solution, KinematicsResult& result) const = 0;

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
                               KinematicsResult& result) const = 0;

    /**
     * @brief For functions that require a timeout specified but one is not specified using arguments,
     * a default timeout is used, as set by this function (and initialized to KinematicsBase::DEFAULT_TIMEOUT)
     * 
     * @param timeout seconds 
     */
    ELITE_EXPORT void setDefaultTimeout(double timeout) { default_timeout_ = timeout; }

    /**
     * @brief For functions that require a timeout specified but one is not specified using arguments,
     * this default timeout is used
     * 
     * @return double seconds
     */
    ELITE_EXPORT double getDefaultTimeout() const { return default_timeout_; }


   protected:
    double default_timeout_;
};

using KinematicsBaseSharedPtr = std::shared_ptr<KinematicsBase>;

}  // namespace ELITE

#endif  // __ELITE__KINEMATICS_BASE_HPP__