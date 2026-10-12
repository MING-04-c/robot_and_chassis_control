import socket


def start_tcp_server(host='localhost', port=50001):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        # 设置accept超时（1秒）
        server_socket.settimeout(1.0)

        server_socket.bind((host, port))
        server_socket.listen(5)
        print(f"TCP服务器启动在 {host}:{port}，等待客户端连接...")

        while True:
            try:
                # 接受客户端连接（现在会超时，让出控制权）
                client_socket, client_address = server_socket.accept()
                print(f"客户端 {client_address} 已连接")

                try:
                    while True:
                        data = client_socket.recv(1024)
                        if not data:
                            print(f"客户端 {client_address} 已断开连接")
                            break

                        received_string = data.decode('utf-8')
                        print(f"来自 {client_address} 的数据: {received_string}")

                except ConnectionResetError:
                    print(f"客户端 {client_address} 异常断开连接")
                except Exception as e:
                    print(f"处理客户端 {client_address} 时发生错误: {e}")
                finally:
                    client_socket.close()

            except socket.timeout:
                # 超时是正常的，继续循环
                continue

    except KeyboardInterrupt:
        print("\n服务器被用户中断")
    except Exception as e:
        print(f"服务器发生错误: {e}")
    finally:
        server_socket.close()
        print("服务器已关闭")


if __name__ == "__main__":
    start_tcp_server(host='0.0.0.0', port=50001)