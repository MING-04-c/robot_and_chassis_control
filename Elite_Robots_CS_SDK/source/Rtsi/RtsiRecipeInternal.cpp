// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "RtsiRecipeInternal.hpp"
#include "EliteException.hpp"
#include "Utils.hpp"

#include <iterator>

using namespace ELITE;

RtsiRecipeInternal::RtsiRecipeInternal(const std::vector<std::string>& list) : RtsiRecipe() { recipe_list_ = list; }

void RtsiRecipeInternal::parserTypePackage(int package_len, const std::vector<std::uint8_t>& package) {
    std::lock_guard<std::mutex> lock(update_mutex_);
    // Referring to the RTSI document, the fourth byte of the message is the recipe ID.
    recipe_id_ = package[3];

    std::string types_string;
    std::copy(package.begin() + 4, package.begin() + package_len, std::back_inserter(types_string));
    std::vector<std::string> types_list = StringUtils::splitString(types_string, ",");
    if (types_list.size() != recipe_list_.size()) {
        throw EliteException(EliteException::Code::RTSI_RECIPE_PARSER_FAIL, "not match recipe");
    }

    RtsiTypeVariant init_value;
    for (int i = 0; i < recipe_list_.size(); i++) {
        if (types_list[i] == "VECTOR6D") {
            init_value = vector6d_t();
        } else if (types_list[i] == "VECTOR3D") {
            init_value = vector3d_t();
        } else if (types_list[i] == "DOUBLE") {
            init_value = double();
        } else if (types_list[i] == "UINT32") {
            init_value = uint32_t();
        } else if (types_list[i] == "UINT64") {
            init_value = uint64_t();
        } else if (types_list[i] == "INT32") {
            init_value = int32_t();
        } else if (types_list[i] == "UINT8") {
            init_value = uint8_t();
        } else if (types_list[i] == "BOOL") {
            init_value = bool();
        } else if (types_list[i] == "UINT16") {
            init_value = uint16_t();
        } else if (types_list[i] == "VECTOR6INT32") {
            init_value = vector6int32_t();
        } else if (types_list[i] == "VECTOR6UINT32") {
            init_value = vector6uint32_t();
        } else {
            throw EliteException(EliteException::Code::RTSI_UNKNOW_VARIABLE_TYPE,
                                 "variable \"" + recipe_list_[i] + "\" error type: " + types_list[i]);
        }
        value_table_.insert({recipe_list_[i], init_value});
    }
}

bool RtsiRecipeInternal::parserDataPackage(int package_len, const std::vector<std::uint8_t>& package) {
    std::lock_guard<std::mutex> lock(update_mutex_);
    // Referring to the RTSI document, the fourth byte of the message is the recipe ID.
    int offset = 3;
    if (package[offset] != recipe_id_) {
        return false;
    }
    offset++;

#if (ELITE_SDK_COMPILE_STANDARD >= 17)
    for (auto item : recipe_list_) {
        RtsiTypeVariant& value = value_table_[item];
        // bool, uint8_t, uint16_t, uint32_t, uint64_t, int32_t, double, vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t
        if (std::holds_alternative<bool>(value)) {
            value = (bool)package[offset];
            offset++;

        } else if (std::holds_alternative<uint8_t>(value)) {
            value = (uint8_t)package[offset];
            offset++;

        } else if (std::holds_alternative<uint16_t>(value)) {
            EndianUtils::unpack(package, offset, std::get<uint16_t>(value));

        } else if (std::holds_alternative<uint32_t>(value)) {
            EndianUtils::unpack(package, offset, std::get<uint32_t>(value));

        } else if (std::holds_alternative<uint64_t>(value)) {
            EndianUtils::unpack(package, offset, std::get<uint64_t>(value));

        } else if (std::holds_alternative<int32_t>(value)) {
            EndianUtils::unpack(package, offset, std::get<int32_t>(value));

        } else if (std::holds_alternative<double>(value)) {
            EndianUtils::unpack(package, offset, std::get<double>(value));

        } else if (std::holds_alternative<vector3d_t>(value)) {
            EndianUtils::unpack<double, 3>(package, offset, std::get<vector3d_t>(value));

        } else if (std::holds_alternative<vector6d_t>(value)) {
            EndianUtils::unpack<double, 6>(package, offset, std::get<vector6d_t>(value));

        } else if (std::holds_alternative<vector6int32_t>(value)) {
            EndianUtils::unpack<int32_t, 6>(package, offset, std::get<vector6int32_t>(value));

        } else if (std::holds_alternative<vector6uint32_t>(value)) {
            EndianUtils::unpack<uint32_t, 6>(package, offset, std::get<vector6uint32_t>(value));

        } else {
            return false;
        }
    }
#elif (ELITE_SDK_COMPILE_STANDARD == 14)
    for (auto item : recipe_list_) {
        RtsiTypeVariant& value = value_table_[item];
        // bool, uint8_t, uint16_t, uint32_t, uint64_t, int32_t, double, vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t
        if (boost::get<bool>(&value)) {
            value = (bool)package[offset];
            offset++;

        } else if (boost::get<uint8_t>(&value)) {
            value = (uint8_t)package[offset];
            offset++;

        } else if (boost::get<uint16_t>(&value)) {
            EndianUtils::unpack(package, offset, boost::get<uint16_t>(value));

        } else if (boost::get<uint32_t>(&value)) {
            EndianUtils::unpack(package, offset, boost::get<uint32_t>(value));

        } else if (boost::get<uint64_t>(&value)) {
            EndianUtils::unpack(package, offset, boost::get<uint64_t>(value));

        } else if (boost::get<int32_t>(&value)) {
            EndianUtils::unpack(package, offset, boost::get<int32_t>(value));

        } else if (boost::get<double>(&value)) {
            EndianUtils::unpack(package, offset, boost::get<double>(value));

        } else if (boost::get<vector3d_t>(&value)) {
            EndianUtils::unpack<double, 3>(package, offset, boost::get<vector3d_t>(value));

        } else if (boost::get<vector6d_t>(&value)) {
            EndianUtils::unpack<double, 6>(package, offset, boost::get<vector6d_t>(value));

        } else if (boost::get<vector6int32_t>(&value)) {
            EndianUtils::unpack<int32_t, 6>(package, offset, boost::get<vector6int32_t>(value));

        } else if (boost::get<vector6uint32_t>(&value)) {
            EndianUtils::unpack<uint32_t, 6>(package, offset, boost::get<vector6uint32_t>(value));

        } else {
            return false;
        }
    }
#endif
    return true;
}

std::vector<uint8_t> RtsiRecipeInternal::packToBytes() {
    std::lock_guard<std::mutex> lock(update_mutex_);
    std::unordered_map<std::string, ELITE::RtsiTypeVariant>::const_iterator iter;
    std::vector<uint8_t> result;
    std::vector<uint8_t> bytes;
    result.push_back(recipe_id_);

#if (ELITE_SDK_COMPILE_STANDARD >= 17)
    for (auto item : recipe_list_) {
        iter = value_table_.find(item);
        if (iter == value_table_.end()) {
            throw EliteException(EliteException::Code::RTSI_RECIPE_PARSER_FAIL, "bad recipe");
        }
        // bool, uint8_t, uint16_t, uint32_t, uint64_t, int32_t, double, vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t
        if (auto va = std::get_if<bool>(&iter->second)) {
            result.push_back(*va);

        } else if (auto va = std::get_if<uint8_t>(&iter->second)) {
            result.push_back(*va);

        } else if (auto va = std::get_if<uint16_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<uint32_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<uint64_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<int32_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<double>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<vector3d_t>(&iter->second)) {
            bytes = EndianUtils::pack<double, 3>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<vector6d_t>(&iter->second)) {
            bytes = EndianUtils::pack<double, 6>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<vector6int32_t>(&iter->second)) {
            bytes = EndianUtils::pack<int32_t, 6>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = std::get_if<vector6uint32_t>(&iter->second)) {
            bytes = EndianUtils::pack<uint32_t, 6>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));
        }
    }
#elif (ELITE_SDK_COMPILE_STANDARD == 14)
    for (auto item : recipe_list_) {
        iter = value_table_.find(item);
        if (iter == value_table_.end()) {
            throw EliteException(EliteException::Code::RTSI_RECIPE_PARSER_FAIL, "bad recipe");
        }
        // bool, uint8_t, uint16_t, uint32_t, uint64_t, int32_t, double, vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t
        if (auto va = boost::get<bool>(&iter->second)) {
            result.push_back(*va);

        } else if (auto va = boost::get<uint8_t>(&iter->second)) {
            result.push_back(*va);

        } else if (auto va = boost::get<uint16_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<uint32_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<uint64_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<int32_t>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<double>(&iter->second)) {
            bytes = EndianUtils::pack(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<vector3d_t>(&iter->second)) {
            bytes = EndianUtils::pack<double, 3>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<vector6d_t>(&iter->second)) {
            bytes = EndianUtils::pack<double, 6>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<vector6int32_t>(&iter->second)) {
            bytes = EndianUtils::pack<int32_t, 6>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));

        } else if (auto va = boost::get<vector6uint32_t>(&iter->second)) {
            bytes = EndianUtils::pack<uint32_t, 6>(*va);
            result.insert(result.end(), std::make_move_iterator(bytes.begin()), std::make_move_iterator(bytes.end()));
        }
    }

#endif
    return result;
}