cmake_minimum_required(VERSION 3.16)

project(elite_kdl_kinematics_plugin LANGUAGES CXX)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

set(PLUGIN_TARGET elite_kdl_kinematics)

add_library(
    ${PLUGIN_TARGET} 
    SHARED
    source/KdlKinematicsPlugin.cpp
)

target_include_directories(
    ${PLUGIN_TARGET}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    ${PLUGIN_TARGET}
    PRIVATE
    elite_cs_series_sdk::shared
    Eigen3::Eigen
    orocos-kdl
)

set_target_properties(
    ${PLUGIN_TARGET} 
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${ELITE_KINEMATICS_PLUGIN_BUILD_PATH}
)

if(ELITE_INSTALL)
    install(
        TARGETS ${PLUGIN_TARGET}
        LIBRARY DESTINATION ${ELITE_KINEMATICS_PLUGIN_INSTALL_PATH}
    )
endif()
