// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// RtsiRecipeInternal.hpp
// Provides the RtsiRecipeInternal class for internal RTSI recipe handling.
#ifndef __RTSI_RECIPE_INTERNAL_HPP__
#define __RTSI_RECIPE_INTERNAL_HPP__

#include <string>
#include <vector>
#include "RtsiRecipe.hpp"

namespace ELITE {

/**
 * @brief
 *      Rtsi recipe, used internal
 *
 */
class RtsiRecipeInternal : public RtsiRecipe {
   private:
   public:
    /**
     * @brief Create new object
     *
     */
    RtsiRecipeInternal() = delete;
    explicit RtsiRecipeInternal(const std::vector<std::string>& list);
    virtual ~RtsiRecipeInternal() = default;

    /**
     * @brief
     *      Parser package RTSI ack of type list and recipe ID
     *      When setup input or output recipe, after send the variable name list, RTSI server will ack the type of variables list
     * and recipe ID
     *
     * @param package_len The package len
     * @param package The bytes buffer of package
     */
    void parserTypePackage(int package_len, const std::vector<std::uint8_t>& package);

    /**
     * @brief
     *      Parser package of Data.
     *      When RTSI client send start signal, RTSI server will periodically send data package.
     *      This function can parser the data package
     *
     * @param package_len The package len
     * @param package The bytes buffer of package
     * @return true success
     * @return false fail
     */
    bool parserDataPackage(int package_len, const std::vector<std::uint8_t>& package);

    /**
     * @brief Pack the data in recipe to bytes
     *
     * @return std::vector<uint8_t> The RTSI data package
     */
    std::vector<uint8_t> packToBytes();
};

}  // namespace ELITE

#endif