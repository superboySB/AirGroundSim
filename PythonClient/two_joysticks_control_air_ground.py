import airsim
import pygame
import time

# 连接到AirSim模拟器
client_air = airsim.MultirotorClient(port=41451)
client_air.confirmConnection()
client_air.enableApiControl(True, "Drone1")
client_air.armDisarm(True, "Drone1")
# 初始控制变量
throttle = 0
pitch = 0
roll = 0
yaw_rate = 0

client_ground = airsim.CarClient(port=41452)
client_ground.confirmConnection()
client_ground.enableApiControl(True, "Car1")
car_controls = airsim.CarControls()

# 初始化pygame和游戏手柄
pygame.init()
joystick_ground = pygame.joystick.Joystick(0)
joystick_ground.init()
joystick_air = pygame.joystick.Joystick(1)
joystick_air.init()


try:
    while True:
        # 检查事件队列
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # 分别为无人车和无人机处理轴事件
                if event.joy == 0:  # 无人车的手柄
                    # 假设轴1控制油门
                    if event.axis == 1:
                        car_controls.throttle = -event.value if event.value < 0 else 0
                        car_controls.brake = -event.value if event.value > 0 else 0
                    # 假设轴0控制方向
                    elif event.axis == 0:
                        car_controls.steering = event.value
                elif event.joy == 1:  # 无人机的手柄
                    # 更新无人机的控制变量
                    # Mode 2 - 左杆垂直（左杆Y轴）控制油门，左杆水平（左杆X轴）控制偏航
                    if event.axis == 1:  # 假设左杆垂直轴是 1
                        throttle = (event.value + 1) / 2  # 油门（0到1）
                    elif event.axis == 0:  # 假设左杆水平轴是 0
                        yaw = event.value * 90  # 偏航速率（-90到90）

                    # Mode 2 - 右杆垂直（右杆Y轴）控制俯仰，右杆水平（右杆X轴）控制横滚
                    if event.axis == 3:  # 假设右杆垂直轴是 3
                        pitch = event.value
                    elif event.axis == 2:  # 假设右杆水平轴是 2
                        roll = event.value


        # 如果有必要，发送无人机控制命令
        if throttle != 0 or pitch != 0 or roll != 0 or yaw_rate != 0:
            client_air.moveByRollPitchYawrateThrottleAsync(roll, pitch, yaw_rate, throttle, 0.1, "Drone1").join()  # 异步后台运行
        

        # 更新无人车控制
        if car_controls.throttle!=0 or car_controls.brake !=0 or car_controls.steering!=0:
            client_ground.setCarControls(car_controls, "Car1")  # 占用主进程
            time.sleep(0.1)


except KeyboardInterrupt:
    # 断开连接
    print("从模拟器断开连接...")
    client_ground.reset()
    client_air.reset()
    pygame.quit()
    print("模拟结束")
