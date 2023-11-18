import airsim
import pygame
import threading

# 设置pygame
pygame.init()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for joystick in joysticks:
    joystick.init()

# 连接到AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")

# 每架无人机的起飞程序
def takeoff_drone(vehicle_name):
    client.takeoffAsync(vehicle_name=vehicle_name).join()

# 启动两架无人机的起飞程序
threading.Thread(target=takeoff_drone, args=("Drone1",)).start()
threading.Thread(target=takeoff_drone, args=("Drone2",)).start()

# 主控制循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYAXISMOTION:
            # 根据手柄输入更新无人机控制
            if event.joy == 0:  # 第一个手柄控制Drone1
                # 这里添加控制Drone1的代码
                pass
            elif event.joy == 1:  # 第二个手柄控制Drone2
                # 这里添加控制Drone2的代码
                pass

# 断开与AirSim的连接
client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")
client.reset()

pygame.quit()
