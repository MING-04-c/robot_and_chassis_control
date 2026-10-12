// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// ReversePort.hpp
// Provides the ReversePort class as a base for reverse communication.
#ifndef __ELITE_REVERSE_PORT_HPP__
#define __ELITE_REVERSE_PORT_HPP__

#include <memory>
#include "TcpServer.hpp"

namespace ELITE {

class ReversePort {
   protected:
    std::shared_ptr<TcpServer> server_;

    int write(void* data, int size) { return server_->writeClient(data, size); }

   public:
    ReversePort(int port, int receive_buffer_size, std::shared_ptr<TcpServer::StaticResource> resource) {
        server_ = std::make_shared<TcpServer>(port, receive_buffer_size, resource);
    }
    ~ReversePort() = default;

    bool isRobotConnect() { return server_->isClientConnected(); }
};

}  // namespace ELITE

#endif
