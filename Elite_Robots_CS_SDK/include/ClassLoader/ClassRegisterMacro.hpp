// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// ClassRegisterMacro.hpp
// Provides macro definitions for registering plugins.
#ifndef __ELITE__CLASS_REGISTER_MACRO_HPP__
#define __ELITE__CLASS_REGISTER_MACRO_HPP__

#include <Elite/ClassRegistry.hpp>
#include <typeinfo>

#define ELITE_CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, RegisterID)                    \
    namespace {                                                                                  \
    struct ProxyExec##RegisterID {                                                               \
        typedef Derived _Derived;                                                                \
        typedef Base _Base;                                                                      \
        ProxyExec##RegisterID() {                                                                \
            ELITE::INTERNAL::ClassRegistry::instance().registerClass(                            \
                #Derived, typeid(_Base).name(), []() -> void* { return static_cast<_Base*>(new _Derived()); }); \
        }                                                                                        \
    };                                                                                           \
    ProxyExec##RegisterID g_register_plugin_##RegisterID;                                 \
    }

#define ELITE_CLASS_LOADER_REGISTER_CLASS(Derived, Base) ELITE_CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, __COUNTER__)

#endif