
import socket
import json
import airsim
import threading

# 创建TCP/IP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定套接字到端口
server_address = ('172.16.15.188', 41453)  # 服务端接受指令
# server_address = ('127.0.0.1', 41453)  # 本机接受指令
sock.bind(server_address)

# 监听传入连接
sock.listen(1)

def handle_client_connection(client_connection):
    # 连接到AirSim模拟器
    client_ground = airsim.CarClient(ip='172.23.53.8', port=41452)  # 仿真客户端接受指令
    # client_ground = airsim.CarClient(port=41452)  # 本机接受指令
    client_ground.confirmConnection()
    client_ground.enableApiControl(True, "Car1")
    car_controls = airsim.CarControls()

    try:
        buffer = ""  # 创建一个缓冲区
        while True:
            # 接收数据
            data = client_connection.recv(1024).decode('utf-8')
            if data:
                buffer += data  # 累积数据
                while "\n" in buffer:  # 检查是否有完整的JSON消息
                    # 分离出完整的JSON消息
                    message, buffer = buffer.split("\n", 1)
                    # 解析手柄输入
                    inputs = json.loads(message)
                    
                    axis1 = inputs['axis1']
                    axis2 = inputs['axis2']

                    # 设置手柄控制参数
                    if axis1 < 0:  # 前进
                        car_controls.is_manual_gear = False
                        car_controls.throttle = -axis1
                        car_controls.brake = 0
                    elif axis1 > 0:  # 后退
                        car_controls.is_manual_gear = True
                        car_controls.manual_gear = -1
                        car_controls.throttle = axis1
                        car_controls.brake = 0
                    else:  # 停止
                        car_controls.is_manual_gear = False
                        car_controls.throttle = 0
                        car_controls.brake = 0
                    car_controls.steering = axis2  # 转向

                    # 更新无人车控制
                    if car_controls.throttle != 0 or car_controls.brake != 0 or car_controls.steering != 0:
                        client_ground.setCarControls(car_controls, "Car1")

                        # 更新无人车控制后，获取车辆状态
                        car_state = client_ground.getCarState("Car1")
                        print(f"Car State: Position: {car_state.kinematics_estimated.position}, "
                            f"Speed: {car_state.speed}, "
                            f"Gear: {car_state.gear}")

    except KeyboardInterrupt:
        # 断开连接
        print("从模拟器断开连接...")
        client_ground.reset()
        client_ground.enableApiControl(False, "Car1")
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        client_connection.close()

    # 当线程结束时关闭AirSim连接
    client_ground.enableApiControl(False, "Car1")
    client_ground.reset()

while True:
    # 等待客户端连接
    client_connection, client_address = sock.accept()
    client_thread = threading.Thread(target=handle_client_connection, args=(client_connection,))
    client_thread.daemon = True  # 设置为守护线程
    client_thread.start()
