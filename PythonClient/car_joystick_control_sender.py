import socket
import pygame
import json
import time


# 初始化pygame和游戏手柄
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 创建TCP/IP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 连接到服务器
server_address = ('172.16.15.188', 41453) # 指发送指令到服务器
# server_address = ('127.0.0.1', 41453) # 指发送指令到本机
sock.connect(server_address)

try:
    while True:
        pygame.event.pump()  # 更新事件队列
        # 捕获手柄输入
        axis1 = joystick.get_axis(1)  # 前进和后退
        axis2 = joystick.get_axis(2)  # 转向
        # 创建手柄输入的字典
        data = {'axis1': axis1, 'axis2': axis2}
        # 发送数据到服务器
        sock.sendall((json.dumps(data) + "\n").encode('utf-8'))
        time.sleep(0.05)  # 等待50毫秒

finally:
    # 清理连接
    sock.close()
    pygame.quit()
