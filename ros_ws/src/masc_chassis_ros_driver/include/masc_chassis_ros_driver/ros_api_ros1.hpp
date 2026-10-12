/*
 * Copyright (C) 2026, MASC.
 * @Description: ROS1 API adapter for the chassis ROS driver.
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <string>

namespace masc_chassis_ros_driver {

// ROS1 封装层：提供与 ROS2 适配层相同的函数名，方便同一套驱动源码双版本编译。

class RosNodeBase {
public:
    explicit RosNodeBase(const std::string & /*node_name*/) : m_nh(), m_private_nh("~") {}
    virtual ~RosNodeBase() = default;

    template <typename MsgT>
    ros::Publisher CreatePublisher(const std::string &topic, std::size_t queue_size = 10) {
        auto resolved = ResolveName(topic);
        return resolved.m_is_private
                   ? m_private_nh.advertise<MsgT>(resolved.m_name,
                                                  static_cast<std::uint32_t>(queue_size))
                   : m_nh.advertise<MsgT>(resolved.m_name, static_cast<std::uint32_t>(queue_size));
    }

    template <typename MsgT, typename CallbackT>
    ros::Subscriber CreateSubscription(const std::string &topic, std::size_t queue_size,
                                       CallbackT callback) {
        auto resolved = ResolveName(topic);
        boost::function<void(const typename MsgT::ConstPtr &)> wrapper =
            [callback](const typename MsgT::ConstPtr &msg) { callback(*msg); };
        return resolved.m_is_private
                   ? m_private_nh.subscribe<MsgT>(resolved.m_name,
                                                  static_cast<std::uint32_t>(queue_size), wrapper)
                   : m_nh.subscribe<MsgT>(resolved.m_name, static_cast<std::uint32_t>(queue_size),
                                          wrapper);
    }

    template <typename SrvT, typename CallbackT>
    ros::ServiceServer CreateService(const std::string &service, CallbackT callback) {
        using Request = typename SrvT::Request;
        using Response = typename SrvT::Response;

        auto resolved = ResolveName(service);
        boost::function<bool(Request &, Response &)> wrapper =
            [callback](Request &request, Response &response) {
                callback(request, response);
                return true;
            };
        return resolved.m_is_private
                   ? m_private_nh.advertiseService<Request, Response>(resolved.m_name, wrapper)
                   : m_nh.advertiseService<Request, Response>(resolved.m_name, wrapper);
    }

    template <typename CallbackT>
    ros::Timer CreateTimer(std::chrono::milliseconds period, CallbackT callback) {
        return m_nh.createTimer(ros::Duration(period.count() / 1000.0),
                                [callback](const ros::TimerEvent & /*event*/) { callback(); });
    }

    template <typename T>
    T DeclareParameter(const std::string &name, const T &default_value) {
        const std::string param_name = ToRosParamName(name);
        T value;
        if (m_private_nh.getParam(param_name, value) || m_nh.getParam(param_name, value)) {
            return value;
        }
        m_private_nh.setParam(param_name, default_value);
        return default_value;
    }

private:
    struct ResolvedName {
        std::string m_name;
        bool m_is_private = false;
    };

    static ResolvedName ResolveName(const std::string &name) {
        if (name.rfind("~/", 0) == 0) {
            return {name.substr(2), true};
        }
        if (name.rfind("~", 0) == 0) {
            return {name.substr(1), true};
        }
        return {name, false};
    }

    static std::string ToRosParamName(const std::string &name) {
        std::string ros_name = name;
        std::replace(ros_name.begin(), ros_name.end(), '.', '/');
        return ros_name;
    }

    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;
};

using RosNodePtr = std::shared_ptr<RosNodeBase>;
using RosTime = ros::Time;

template <typename MsgT>
using RosPublisher = ros::Publisher;
template <typename MsgT>
using RosSubscription = ros::Subscriber;
template <typename SrvT>
using RosService = ros::ServiceServer;
using RosTimer = ros::Timer;

inline void RosInit(int argc, char **argv) { ros::init(argc, argv, "masc_chassis"); }
inline void RosSpin(const RosNodePtr & /*node*/) { ros::spin(); }
inline void RosShutdown() { ros::shutdown(); }
inline RosTime Now(RosNodeBase * /*node*/) { return ros::Time::now(); }
inline double SecondsBetween(const RosTime &start, const RosTime &end) {
    return (end - start).toSec();
}

template <typename MsgT>
inline RosPublisher<MsgT> CreatePublisher(RosNodeBase *node, const std::string &topic,
                                          std::size_t queue_size = 10) {
    return node->CreatePublisher<MsgT>(topic, queue_size);
}

template <typename MsgT, typename CallbackT>
inline RosSubscription<MsgT> CreateSubscription(RosNodeBase *node, const std::string &topic,
                                                std::size_t queue_size, CallbackT callback) {
    return node->CreateSubscription<MsgT>(topic, queue_size, callback);
}

template <typename SrvT, typename CallbackT>
inline RosService<SrvT> CreateService(RosNodeBase *node, const std::string &service,
                                      CallbackT callback) {
    return node->CreateService<SrvT>(service, callback);
}

template <typename CallbackT>
inline RosTimer CreateTimer(RosNodeBase *node, std::chrono::milliseconds period,
                            CallbackT callback) {
    return node->CreateTimer(period, callback);
}

template <typename T>
inline T DeclareParameter(RosNodeBase *node, const std::string &name, const T &default_value) {
    return node->DeclareParameter<T>(name, default_value);
}

template <typename MsgT>
inline void Publish(const RosPublisher<MsgT> &publisher, const MsgT &msg) {
    if (publisher) {
        publisher.publish(msg);
    }
}

#define MCHASSIS_INFO(node, ...)  ROS_INFO(__VA_ARGS__)
#define MCHASSIS_WARN(node, ...)  ROS_WARN(__VA_ARGS__)
#define MCHASSIS_ERROR(node, ...) ROS_ERROR(__VA_ARGS__)
#define MCHASSIS_DEBUG(node, ...) ROS_DEBUG(__VA_ARGS__)

}  // namespace masc_chassis_ros_driver
