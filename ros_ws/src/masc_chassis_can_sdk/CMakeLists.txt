cmake_minimum_required(VERSION 3.16)
project(masc_chassis_can_sdk VERSION 1.0.0 LANGUAGES CXX)

find_package(Threads REQUIRED)
include(GNUInstallDirs)

set(MASC_CHASSIS_CAN_SDK_DEFAULT_USE_AMENT ON)
set(MASC_CHASSIS_CAN_SDK_DEFAULT_USE_CATKIN OFF)
if(DEFINED ENV{ROS_VERSION})
    if("$ENV{ROS_VERSION}" STREQUAL "1")
        set(MASC_CHASSIS_CAN_SDK_DEFAULT_USE_AMENT OFF)
        set(MASC_CHASSIS_CAN_SDK_DEFAULT_USE_CATKIN ON)
    elseif("$ENV{ROS_VERSION}" STREQUAL "2")
        set(MASC_CHASSIS_CAN_SDK_DEFAULT_USE_AMENT ON)
        set(MASC_CHASSIS_CAN_SDK_DEFAULT_USE_CATKIN OFF)
    endif()
endif()

option(MASC_CHASSIS_CAN_SDK_USE_AMENT "Export the SDK with ament_cmake"
       ${MASC_CHASSIS_CAN_SDK_DEFAULT_USE_AMENT})
option(MASC_CHASSIS_CAN_SDK_USE_CATKIN "Export the SDK with catkin"
       ${MASC_CHASSIS_CAN_SDK_DEFAULT_USE_CATKIN})

if(MASC_CHASSIS_CAN_SDK_USE_CATKIN AND MASC_CHASSIS_CAN_SDK_USE_AMENT)
    message(FATAL_ERROR "MASC_CHASSIS_CAN_SDK_USE_CATKIN and "
                        "MASC_CHASSIS_CAN_SDK_USE_AMENT cannot both be ON")
endif()

if(MASC_CHASSIS_CAN_SDK_USE_CATKIN)
    find_package(catkin REQUIRED)
    set(MASC_CHASSIS_CAN_SDK_INSTALL_INCLUDEDIR ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    set(MASC_CHASSIS_CAN_SDK_INSTALL_LIBDIR ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(MASC_CHASSIS_CAN_SDK_INSTALL_BINDIR ${CATKIN_PACKAGE_BIN_DESTINATION})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
elseif(MASC_CHASSIS_CAN_SDK_USE_AMENT)
    find_package(ament_cmake REQUIRED)
    set(MASC_CHASSIS_CAN_SDK_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
    set(MASC_CHASSIS_CAN_SDK_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR})
    set(MASC_CHASSIS_CAN_SDK_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
else()
    message(FATAL_ERROR "Enable MASC_CHASSIS_CAN_SDK_USE_CATKIN or "
                        "MASC_CHASSIS_CAN_SDK_USE_AMENT")
endif()

set(MASC_CHASSIS_CAN_SDK_SOURCES
    src/chassis_protocol.cpp
    src/chassis_sdk.cpp
    src/unsupported_can_transport.cpp
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND MASC_CHASSIS_CAN_SDK_SOURCES src/socketcan_transport.cpp)
endif()

add_library(masc_chassis_can_sdk
    ${MASC_CHASSIS_CAN_SDK_SOURCES}
)
add_library(masc_chassis_can_sdk::masc_chassis_can_sdk ALIAS masc_chassis_can_sdk)

target_compile_features(masc_chassis_can_sdk PUBLIC cxx_std_17)

target_include_directories(masc_chassis_can_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(masc_chassis_can_sdk PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(masc_chassis_can_sdk PUBLIC Threads::Threads)

option(MASC_CHASSIS_CAN_SDK_BUILD_TESTS "Build MASC chassis CAN SDK tests" ON)
option(MASC_CHASSIS_CAN_SDK_BUILD_EXAMPLES "Build MASC chassis CAN SDK examples" ON)

if(MASC_CHASSIS_CAN_SDK_BUILD_EXAMPLES)
    add_executable(masc_chassis_can_sdk_demo
        examples/chassis_can_sdk_demo.cpp
    )
    target_link_libraries(masc_chassis_can_sdk_demo PRIVATE masc_chassis_can_sdk)
endif()

if(MASC_CHASSIS_CAN_SDK_BUILD_TESTS)
    enable_testing()

    add_executable(masc_chassis_can_sdk_protocol_tests
        tests/protocol_tests.cpp
    )
    target_link_libraries(masc_chassis_can_sdk_protocol_tests PRIVATE masc_chassis_can_sdk)
    add_test(NAME masc_chassis_can_sdk_protocol_tests COMMAND masc_chassis_can_sdk_protocol_tests)

    add_executable(masc_chassis_can_sdk_runtime_tests
        tests/runtime_tests.cpp
    )
    target_link_libraries(masc_chassis_can_sdk_runtime_tests PRIVATE masc_chassis_can_sdk)
    add_test(NAME masc_chassis_can_sdk_runtime_tests COMMAND masc_chassis_can_sdk_runtime_tests)
endif()

if(MASC_CHASSIS_CAN_SDK_USE_CATKIN)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES masc_chassis_can_sdk
    )
endif()

install(
    TARGETS masc_chassis_can_sdk
    EXPORT export_masc_chassis_can_sdk
    ARCHIVE DESTINATION ${MASC_CHASSIS_CAN_SDK_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${MASC_CHASSIS_CAN_SDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MASC_CHASSIS_CAN_SDK_INSTALL_BINDIR}
    INCLUDES DESTINATION ${MASC_CHASSIS_CAN_SDK_INSTALL_INCLUDEDIR}
)

install(
    DIRECTORY include/
    DESTINATION ${MASC_CHASSIS_CAN_SDK_INSTALL_INCLUDEDIR}
)

if(MASC_CHASSIS_CAN_SDK_BUILD_EXAMPLES)
    install(
        TARGETS masc_chassis_can_sdk_demo
        RUNTIME DESTINATION ${MASC_CHASSIS_CAN_SDK_INSTALL_BINDIR}
    )
endif()

if(MASC_CHASSIS_CAN_SDK_USE_AMENT)
    ament_export_targets(export_masc_chassis_can_sdk HAS_LIBRARY_TARGET)
    ament_export_dependencies(Threads)

    ament_package()
endif()
