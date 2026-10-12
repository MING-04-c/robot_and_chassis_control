// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// ClassRegistry.hpp
// Implementations for class plugin registration. These interfaces should not be explicitly called by users.
#ifndef __ELITE_CLASS_REGISTRY_HPP__
#define __ELITE_CLASS_REGISTRY_HPP__

#include <Elite/EliteOptions.hpp>
#include <Elite/Log.hpp>
#include <memory>
#include <string>
#include <unordered_map>

namespace ELITE {

namespace INTERNAL {

class ClassRegistry {
   public:
    using Factory = void*(*)();

    ELITE_EXPORT static ClassRegistry& instance();

    ClassRegistry();
    ~ClassRegistry() = default;

    ELITE_EXPORT bool registerClass(const std::string& derived, const std::string& base, Factory factory);

    ELITE_EXPORT Factory getFactory(const std::string& derived, const std::string& base) const;

   private:
    std::unordered_map<std::string, Factory> factories_;

   private:
    // Compose a unique key from derived and base class names.
    // Uses a control character as a separator to minimize collision risk.
    std::string makeClassKey(const std::string& derived, const std::string& base) const;
};

}  // namespace INTERNAL

}  // namespace ELITE

#endif  // __ELITE_CLASS_REGISTRY_HPP__