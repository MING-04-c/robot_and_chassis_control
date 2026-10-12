// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// ClassLoader.hpp
// Provides interfaces for loading plugins.
#ifndef __ELITE__CLASS_LOADER_HPP__
#define __ELITE__CLASS_LOADER_HPP__

#include <memory>
#include <string>
#include <Elite/ClassRegistry.hpp>

namespace ELITE {
class ELITE_EXPORT ClassLoader {
   private:
    class Impl;
    std::unique_ptr<Impl> impl_;

   public:
   /**
    * @brief Constructor.
    *   In this constructor, only the library path is set, the library is not loaded yet.
    * @param lib_path library path
    */
    explicit ClassLoader(const std::string& lib_path);

    ~ClassLoader();

    /**
     * @brief Load library
     * 
     * @return true Library loaded successfully
     */
    bool loadLib();

    /**
     * @brief Check if library is loaded
     * 
     * @return true Library is loaded
     */
    bool hasLoadedLib() const;

    /**
     * @brief Create a object
     * 
     * @tparam Base The base class type
     * @param derived_class_name Derived class name
     * @return std::unique_ptr<Base> The created object pointer, nullptr if failed
     */
    template <typename Base>
    std::unique_ptr<Base> createUniqueInstance(const std::string& derived_class_name) {
        auto factory = INTERNAL::ClassRegistry::instance().getFactory(derived_class_name, typeid(Base).name());
        if (!hasLoadedLib() || !factory) {
            return nullptr;
        }
        return std::unique_ptr<Base>(static_cast<Base*>(factory()));
    }
};

}  // namespace ELITE

#endif  // __ELITE__CLASS_LOADER_HPP__