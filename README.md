# 一车一机独立控制实验流程
## 简要实现思想
简单来说就是一上来先启动computer vision/manual camera模式来移动键盘到合适的位置观看仿真，然后使用xbox、wfly两个手柄分别控制一个车和一个机。

这里有一些非常无奈的改动：

1. 由于airsim的车辆控制绑死要接受updateCarControls函数（接受手柄/方向盘/键盘等外设信息），如果直接全部删除后用ros/python api也无法控制，因为它们实现的时候虽然只控制RPM等、但也都会过这个函数来update state，我这里就只屏蔽了车辆的键盘控制功能，保留原来的功能。这样子的话，在本代码中，不论如何设置settings，键盘无法控制除了玩家视角以外的任何实体，便于调整观察视角。

2. 

## 具体流程
安装环境 (如果不使用代理，可以--proxy选项省略)
```sh
git clone https://github.com/superboySB/AirGroundSim.git && cd AirGroundSim && git checkout two_joystick_control

pip install --proxy="http://127.0.0.1:10809" sympy opencv-python opencv-contrib-python torch msgpack-rpc-python pygame

cd PythonClient && pip install -e .

```
尝试的settings如下，使用PX4飞机+PhysX车。需要先启动一个PX4-Autopilot实例，此时LocalHostIp设置为启动UE（Airsim）实例的机器ip，而controlIP设置为启动PX4实例的机器ip，如果都在本机应该设置为`127.0.0.1`即可。如果使用我提供的docker，可以直接实例化，一键启动（注意X4_SIM_HOST_ADDR对应设置为LocalHostIp的仿真机的ip）
```sh
docker build --pull --network host -t mypx4_image:v1 -f docker/ros2px4/Dockerfile docker

docker run -itd --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY --gpus all --user=user --env=PX4_SIM_HOST_ADDR=172.23.53.8 --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash

docker exec -it --user=user mypx4-dev /bin/bash

bash /home/user/PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh 1

# 需要关闭实例: pkill -x px4 || true
```
然后如下配置`settings.json`即可：
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Both",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "Car1": {
            "VehicleType": "PhysXCar",
            "X": 0,
            "Y": 0,
            "Z": -1,
            "Yaw": 0
        },
        "Drone1": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "LocalHostIp": "172.23.53.8",
            "ControlIp": "172.16.15.188",
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "Sensors":{
                "Barometer":{
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                }
            },
            "X": -2,
            "Y": -2,
            "Z": -3,
            "Yaw": 0
        }
    }
}
```


![](https://i.postimg.cc/NGD6MtB0/image.png)


## 一些感觉比较有用的issues
1. Recording of self-defined camera images: https://github.com/microsoft/AirSim/issues/1732#issue-402292365
2. 
