// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "VersionInfo.hpp"
#include "Utils.hpp"
#include "EliteException.hpp"

using namespace ELITE;

VersionInfo::VersionInfo(const std::string& version) {
    fromString(version);
}

std::string VersionInfo::toString() const {
    return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(bugfix) + "." + std::to_string(build);
}

VersionInfo VersionInfo::fromString(const std::string& str) {
    auto components = StringUtils::splitString(str, ".");
    VersionInfo version;
    if (components.size() >= 2) {
        version.major = std::stoi(components[0]);
        version.minor = std::stoi(components[1]);
        if (components.size() ==3 ) {
            version.bugfix = std::stoi(components[2]);
        } else if (components.size() == 4) {
            version.bugfix = std::stoi(components[2]);
            version.build = std::stoi(components[3]);
        }
    } else {
        throw EliteException(EliteException::Code::ILLEGAL_PARAM, 
                             "Given string '" + str + "' does not conform a version string format.");
    }
    return version;
}

VersionInfo& VersionInfo::operator=(const VersionInfo& input) {
    major = input.major;
    minor = input.minor;
    bugfix = input.bugfix;
    build = input.build;
    return *this;
}

bool VersionInfo::operator==(const VersionInfo& v) const {
    if (major == v.major && minor == v.minor && bugfix == v.bugfix && build == v.build) {
        return true;
    }
    return false;
}

bool VersionInfo::operator!=(const VersionInfo& v) const {
    return !(*this == v);
}

bool VersionInfo::operator>(const VersionInfo& v) const {
    if (major > v.major && minor > v.minor)
    {
        return true;
    }
    return false;
}


bool VersionInfo::operator>=(const VersionInfo& v) const {
    return (*this == v) || (*this > v);
}


bool VersionInfo::operator<(const VersionInfo& v) const {
    return !(*this >= v);
}


bool VersionInfo::operator<=(const VersionInfo& v) const {
    return (*this == v) || (*this < v);
}

constexpr bool VersionInfo::operator==(VersionInfo& v) const {
    if (major == v.major && minor == v.minor && bugfix == v.bugfix && build == v.build) {
        return true;
    }
    return false;
}

constexpr bool VersionInfo::operator!=(VersionInfo& v) const {
    return !(*this == v);
}

constexpr bool VersionInfo::operator>(VersionInfo& v) const {
    if (major > v.major && minor > v.minor)
    {
        return true;
    }
    return false;
}

constexpr bool VersionInfo::operator>=(VersionInfo& v) const {
    return (*this == v) || (*this > v);
}

constexpr bool VersionInfo::operator<(VersionInfo& v) const {
    return !(*this >= v);
}

constexpr bool VersionInfo::operator<=(VersionInfo& v) const {
    return (*this == v) || (*this < v);
}