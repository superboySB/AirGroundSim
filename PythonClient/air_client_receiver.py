import airsim
import numpy as np
import cv2
import os

# 连接到AirSim模拟器
client = airsim.MultirotorClient(ip="172.23.53.8", port=41451)  # 从服务器来连接客户机
# client = airsim.MultirotorClient(port=41451)  # 本机使用
client.confirmConnection()

# 飞机的名称
vehicle_name = "Drone1"

# 摄像头的名称
camera_name = "BottomCamera"

# 图像类型：0表示场景（彩色）图像
image_type = airsim.ImageType.Scene

# 设置图片保存的路径
image_save_path = os.getcwd()
print("Images will be saved to: ", image_save_path)

# 是否为debug模式
debug_mode = True

image_count = 0  # 用于图片命名

print("Press Enter to capture images or Ctrl+C to stop...")

try:
    while True:
        # 等待用户输入
        input("Press Enter to capture an image...")

        # 从摄像头获取图像
        responses = client.simGetImages([
            airsim.ImageRequest(camera_name, image_type, False, False)  # 场景图像，未压缩，未校正
        ], vehicle_name=vehicle_name)

        for response in responses:
            if response.image_data_uint8:
                # 获取图像数据并转换为numpy数组
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                # 重塑为3通道图像数组H X W X 3
                img_rgb = img1d.reshape(response.height, response.width, 3)
                
                if debug_mode:
                    image_filename = f"image_{image_count}.png"
                    cv2.imwrite(os.path.join(image_save_path, image_filename), img_rgb)
                    image_count += 1
                    print(f"Saved {image_filename}")

except KeyboardInterrupt:
    print("Image capture stopped by user.")
