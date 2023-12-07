import airsim
import numpy as np
import cv2
import os
import time

# 无人机和汽车的起始位置
# TODO： 需要从settings里面读一下
drone_start_pos = airsim.Vector3r(4, 2, -3)
car_start_pos = airsim.Vector3r(0, 0, -1)

# 汽车和飞机的起始位置差
start_pos_diff = airsim.Vector3r(car_start_pos.x_val - drone_start_pos.x_val,
                                 car_start_pos.y_val - drone_start_pos.y_val,
                                 car_start_pos.z_val - drone_start_pos.z_val)

# 连接到AirSim模拟器的无人机和汽车客户端
client_drone = airsim.MultirotorClient(ip="172.23.53.8", port=41451)
client_drone.confirmConnection()
client_drone.enableApiControl(True, "Drone1")
client_drone.armDisarm(True, "Drone1")

client_ground = airsim.CarClient(ip='172.23.53.8', port=41452)
client_ground.confirmConnection()
client_ground.enableApiControl(True, "Car1")


# 飞机的名称、摄像头的名称、图像类型
vehicle_name = "Drone1"
camera_name = "BottomCamera"
image_type = airsim.ImageType.Scene

# 设置图片保存的路径和图片计数器
image_save_path = os.getcwd()
image_count = 0

# 让无人机起飞
print("Taking off...")
client_drone.takeoffAsync(vehicle_name=vehicle_name).join()

# 设定无人机上升到特定的高度，这里我们设置为-5米（负值表示上升）
target_height = -5
print(f"Moving to target height: {target_height}")
client_drone.moveToZAsync(z=target_height, velocity=2, vehicle_name=vehicle_name).join()

# 确认无人机已达到目标高度
drone_initial_height = client_drone.getMultirotorState(vehicle_name=vehicle_name).kinematics_estimated.position.z_val
print(f"Drone's height after takeoff is {drone_initial_height}")

# 主循环：无人机跟随汽车并定时拍照
try:
    print("无人机开始跟随汽车...")
    last_time_photo_taken = time.time()
    while True:
        # 获取汽车的当前状态
        car_state = client_ground.getCarState("Car1")
        car_pos = car_state.kinematics_estimated.position
        # print(f"Car State: Position: {car_pos}, "
        #                     f"Speed: {car_state.speed}, "
        #                     f"Gear: {car_state.gear}")
        # 将汽车的相对位置转换为全局坐标
        global_car_pos = airsim.Vector3r(car_pos.x_val + start_pos_diff.x_val,
                                         car_pos.y_val + start_pos_diff.y_val,
                                         car_pos.z_val + start_pos_diff.z_val)



        # 让无人机飞到汽车上方的同一高度
        client_drone.armDisarm(True, "Drone1")
        flight_task = client_drone.moveToPositionAsync(
            x=global_car_pos.x_val,
            y=global_car_pos.y_val,
            z=drone_initial_height,
            velocity=5,
            vehicle_name=vehicle_name
        )

        # 检查是否到了拍照时间
        if time.time() - last_time_photo_taken > 10:
            print("正在拍照...")
            responses = client_drone.simGetImages([
                airsim.ImageRequest(camera_name, image_type, False, False)
            ], vehicle_name=vehicle_name)

            for response in responses:
                if response.image_data_uint8:
                    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                    img_rgb = img1d.reshape(response.height, response.width, 3)
                    image_filename = os.path.join(image_save_path, f"image_{image_count}.png")
                    cv2.imwrite(image_filename, img_rgb)
                    print(f"保存照片：{image_filename}")
                    image_count += 1
                    last_time_photo_taken = time.time()  # 更新上次拍照时间

        # 等待一段时间再次检查
        time.sleep(1)

except KeyboardInterrupt:
    print("用户中断了程序")