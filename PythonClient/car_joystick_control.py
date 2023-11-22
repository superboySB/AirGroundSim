import airsim
import pygame
import time

# 连接到AirSim模拟器
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
                if event.joy == 1:
                    continue
                else:
                    # 左侧遥感轴1控制前进和后退
                    if event.axis == 1:
                        if event.value < 0:  # 前进
                            car_controls.is_manual_gear = False  # 自动档
                            car_controls.throttle = -event.value
                            car_controls.brake = 0
                        elif event.value > 0:  # 后退
                            car_controls.is_manual_gear = True  # 手动档
                            car_controls.manual_gear = -1  # 倒档
                            car_controls.throttle = event.value
                            car_controls.brake = 0
                        else:  # 停止
                            car_controls.is_manual_gear = False
                            car_controls.throttle = 0
                            car_controls.brake = 0

                    # 右侧遥感轴3（或者轴4）控制左右转向
                    elif event.axis == 2:  # 根据你的控制器调整轴的编号
                        car_controls.steering = event.value

        # 更新无人车控制
        if car_controls.throttle != 0 or car_controls.brake != 0 or car_controls.steering != 0:
            client_ground.setCarControls(car_controls, "Car1")  # 占用主进程

except KeyboardInterrupt:
    # 断开连接
    print("从模拟器断开连接...")
    client_ground.reset()
    pygame.quit()
    print("模拟结束")
