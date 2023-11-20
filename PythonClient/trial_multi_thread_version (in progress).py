import airsim
import pygame
import time
import threading
import queue

# 初始化pygame和游戏手柄
pygame.init()
joystick_ground = pygame.joystick.Joystick(0)
joystick_ground.init()
joystick_air = pygame.joystick.Joystick(1)
joystick_air.init()

# 创建两个事件队列
event_queue_ground = queue.Queue()
event_queue_air = queue.Queue()

# 停止信号
stop_signal = threading.Event()

# 事件处理线程
def handle_events():
    while not stop_signal.is_set():
        events = pygame.event.get()  # 只在这里调用pygame.event.get()
        for event in events:
            if event.type in [pygame.JOYAXISMOTION, pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP]:
                if event.joy == 0:  # 无人车的手柄事件
                    event_queue_ground.put(event)
                elif event.joy == 1:  # 无人机的手柄事件
                    event_queue_air.put(event)
        time.sleep(0.01)  # 休眠以减少CPU使用


def process_car_event(event, car_controls):
    # 假设轴1控制油门
    if event.axis == 1:
        car_controls.throttle = -event.value if event.value < 0 else 0
        car_controls.brake = -event.value if event.value > 0 else 0
    # 假设轴0控制方向
    elif event.axis == 0:
        car_controls.steering = event.value


# 控制无人车
def control_car():
    client_ground = airsim.CarClient(port=41452)
    client_ground.confirmConnection()
    client_ground.enableApiControl(True, "Car1")
    car_controls = airsim.CarControls()

    while not stop_signal.is_set():
        while not event_queue_ground.empty():
            event = event_queue_ground.get_nowait()
            process_car_event(event, car_controls)
        # 更新无人车控制
        client_ground.setCarControls(car_controls)
        time.sleep(0.05)

    # 清空队列并处理剩余事件
    while not event_queue_ground.empty():
        event = event_queue_ground.get_nowait()
        process_car_event(event, car_controls)
            


def process_drone_event(event, drone_controls):
    # Mode 1 - 左杆垂直（左杆Y轴）控制俯仰，左杆水平（左杆X轴）控制横滚
    if event.axis == 1:  # 假设左杆垂直轴是 1
        drone_controls['pitch'] = -event.value

    if event.axis == 0:  # 假设左杆水平轴是 0
        drone_controls['roll'] = event.value
        
    # Mode 1 - 右杆垂直（右杆Y轴）控制油门，右杆水平（右杆X轴）控制偏航
    if event.axis == 3:  # 假设右杆垂直轴是 3
        drone_controls['throttle'] = (event.value + 1) / 2  # 油门（0到1）

    if event.axis == 2:  # 假设右杆水平轴是 2
        drone_controls['yaw_rate'] = event.value * 90  # 偏航速率（-90到90）


# 控制无人机
def control_drone():
    client_air = airsim.MultirotorClient(port=41451)
    client_air.confirmConnection()
    client_air.enableApiControl(True, "Drone1")
    client_air.armDisarm(True, "Drone1")

    drone_controls = {'roll': 0, 'pitch': 0, 'yaw_rate': 0, 'throttle': 0}

    while not stop_signal.is_set():
        while not event_queue_air.empty():
            event = event_queue_air.get_nowait()
            process_drone_event(event, drone_controls)

        # 如果有必要，发送无人机控制命令
        if any(drone_controls.values()):
            client_air.moveByRollPitchYawrateThrottleAsync(
                drone_controls['roll'], drone_controls['pitch'], 
                drone_controls['yaw_rate'], drone_controls['throttle'], 
                0.1, "Drone1"
            ).join()
        time.sleep(0.05)

    # 清空队列并处理剩余事件
    while not event_queue_air.empty():
        event = event_queue_air.get_nowait()
        process_drone_event(event, drone_controls)


# 创建并启动线程
event_thread = threading.Thread(target=handle_events)
car_thread = threading.Thread(target=control_car)
# drone_thread = threading.Thread(target=control_drone)

event_thread.start()
car_thread.start()
# drone_thread.start()

# 主线程工作直到收到终止信号
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    stop_signal.set()
    print("终止所有线程...")

# 等待线程结束
event_thread.join()
car_thread.join()
# drone_thread.join()
print("程序已终止")