/*
 * Copyright (C) 2026, MASC.
 * @Description: ROS2 API adapter for the chassis ROS driver.
 */
#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace masc_chassis_ros_driver {

// ROS2 封装层：统一 Node、Publisher、Subscription、Service、Timer 和时间接口。
// 上层代码调用这些小函数，不需要到处写 rclcpp 模板细节。

using RosNodeBase = rclcpp::Node;
using RosNodePtr = std::shared_ptr<RosNodeBase>;
using RosTime = rclcpp::Time;

template <typename MsgT>
using RosPublisher = typename rclcpp::Publisher<MsgT>::SharedPtr;
template <typename MsgT>
using RosSubscription = typename rclcpp::Subscription<MsgT>::SharedPtr;
template <typename SrvT>
using RosService = typename rclcpp::Service<SrvT>::SharedPtr;
using RosTimer = rclcpp::TimerBase::SharedPtr;

inline void RosInit(int argc, char **argv) { rclcpp::init(argc, argv); }
inline void RosSpin(const RosNodePtr &node) { rclcpp::spin(node); }
inline void RosShutdown() { rclcpp::shutdown(); }
inline RosTime Now(RosNodeBase *node) { return node->now(); }
inline double SecondsBetween(const RosTime &start, const RosTime &end) {
    return (end - start).seconds();
}

template <typename MsgT>
inline RosPublisher<MsgT> CreatePublisher(RosNodeBase *node, const std::string &topic,
                                          std::size_t queue_size = 10) {
    return node->create_publisher<MsgT>(topic, rclcpp::QoS(queue_size));
}

template <typename MsgT, typename CallbackT>
inline RosSubscription<MsgT> CreateSubscription(RosNodeBase *node, const std::string &topic,
                                                std::size_t queue_size, CallbackT callback) {
    return node->create_subscription<MsgT>(
        topic, rclcpp::QoS(queue_size),
        [callback](typename MsgT::SharedPtr msg) { callback(*msg); });
}

template <typename SrvT, typename CallbackT>
inline RosService<SrvT> CreateService(RosNodeBase *node, const std::string &service,
                                      CallbackT callback) {
    return node->create_service<SrvT>(
        service, [callback](const std::shared_ptr<typename SrvT::Request> request,
                            std::shared_ptr<typename SrvT::Response> response) {
            callback(*request, *response);
        });
}

template <typename CallbackT>
inline RosTimer CreateTimer(RosNodeBase *node, std::chrono::milliseconds period,
                            CallbackT callback) {
    return node->create_wall_timer(period, callback);
}

template <typename T>
inline T DeclareParameter(RosNodeBase *node, const std::string &name, const T &default_value) {
    return node->declare_parameter<T>(name, default_value);
}

template <typename MsgT>
inline void Publish(const RosPublisher<MsgT> &publisher, const MsgT &msg) {
    if (publisher) {
        publisher->publish(msg);
    }
}

#define MCHASSIS_INFO(node, ...)  RCLCPP_INFO((node)->get_logger(), __VA_ARGS__)
#define MCHASSIS_WARN(node, ...)  RCLCPP_WARN((node)->get_logger(), __VA_ARGS__)
#define MCHASSIS_ERROR(node, ...) RCLCPP_ERROR((node)->get_logger(), __VA_ARGS__)
#define MCHASSIS_DEBUG(node, ...) RCLCPP_DEBUG((node)->get_logger(), __VA_ARGS__)

}  // namespace masc_chassis_ros_driver
