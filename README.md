# AirGroundSim
同时仿真车辆和无人机以支持空地协同相关研究

## 研究进度
[X] 尝试PhysX下的车机并存（暂时不支持RC切换，建议使用脚本控制）

[X] 尝试引入PX4 SITL

[ ] 添加PythonClient全面支持

[ ] 添加ROS/ROS2全面支持

[ ] 验证PX4在gazebo上的自主起降demo，看看能否在airsim跑通 (也可以参考一下[amov](https://wiki.amovlab.com/public/prometheus-wiki/%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9D%97-object_detection/%E6%95%99%E5%AD%A6%E4%BE%8B%E7%A8%8B/%E4%BA%8C%E7%BB%B4%E7%A0%81%E6%8E%A2%E5%AF%BB%E9%99%8D%E8%90%BD.html)、[xtdrone](https://github.com/robin-shaun/XTDrone))

[ ] 尝试用airsim的data recorder训练机载大模型，再用PX4 SITL方式验证这个工作：https://github.com/alaamaalouf/FollowAnything

## 简单部署流程
目前先在windows场景上做测试，基本安装流程参考宁子安的知乎[讲解1](https://zhuanlan.zhihu.com/p/618440744)、[讲解2](https://zhuanlan.zhihu.com/p/619214564)、[讲解3](https://zhuanlan.zhihu.com/p/620006613)，一些关键的用法大佬都已经给出。这里只补充一些对于本项目单独的流程，首先按上述讲解编译代码、进入block例程后，设置`settings.json`内容如下（所有可能的参数模板见下[教程](https://microsoft.github.io/AirSim/settings/)）
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Both",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": -1,
            "Y": 1,
            "Z": -5,
            "Yaw": 0
        },
        "Car1": {
            "VehicleType": "PhysXCar",
            "X": 0,
            "Y": 0,
            "Z": -1,
            "Yaw": 0
        }
    }
}
```
可以看到车机共存，这样可以继续进行PX4的部署。

## PX4部署流程（正在探索ing）
如果想要尝试使用PX4，还需要进一步打开一个PX4实例，考虑到官方教程里面的WSL2限制太多，为了便于部署，PX4+RL都建议在远程server（如`172.16.15.188`）的docker里运行，同时airsim在本地windows11开发机（如`172.23.53.8`）里运行,具体如下
```sh
git clone https://github.com/superboySB/SBDrone && cd SBDrone

docker build --pull --network host -t mypx4_image:v1 -f docker/ros2px4/Dockerfile docker

docker run -itd --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY --gpus all --user=user --env=PX4_SIM_HOST_ADDR=172.23.53.8 --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash

docker exec -it --user=user mypx4-dev /bin/bash
```
可以在容器内部拷贝`PythonClient`文件夹中的代码即可（client都要添加`ip = "172.23.53.8"`），然后安装对应的grpc包，以offboard-control的方式写相应算法来控制飞机
```sh
cd PythonClient && pip install -r requirements.txt && pip install -e .
```
由此，可以尝试在这个容器里进行后续开发。如果不满足于simpleflight想使用PX4的时候，需要预先启动sitl实例才可以，并且相应的settings要设置为
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "UAV_0": {
            "VehicleType": "PX4Multirotor",
            "PawnPath": "DefaultQuadrotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "LocalHostIp": "172.23.53.8",
            "ControlIp": "172.16.15.188",
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "X": 0,
            "Y": 1,
            "Z": 0,
            "Yaw": 0,
            "Sensors": {
                "Barometer": {
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                }
            },
            "Parameters": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1,
                "LPE_LAT": 47.641468,
                "LPE_LON": -122.140165
            },
    }
}
```

## 手动控制一台PX4无人机测试
建议在空题协同（Both）模式下不使用手动控制、改为python/ros client控制。
### 方法一：使用QGC（推荐）
如果需要手动控制无人机(remote control)，则在QGroundControl里面，必须手动设置通信链接，QGC的自动连接功能在多个机器的时候不起作用，如果在同一台机器有时候没有问题。具体做法是，添加一个14550的UDP监听，并且需要在可选的指定server处添加`172.16.15.188:18570`，并点击连接，如果有多台则需要连接多次，端口要累加。对应地，需要开启多个PX4实例，其它参数配置可以参考[官方教程](https://microsoft.github.io/AirSim/px4_sitl/)，可能教程会年久失修，但原理还是相应TCP/UDP同样端口需要累加。

![](https://i.postimg.cc/PrbLDvnj/c0c7c6df9ff1f6a4b581e8532b0f965.png)

### 方法二：不使用QGC
在`settings.json`中对需要控制的无人机添加手柄id
```json
"RC": {
		"RemoteControlID": 1
	}
```
打开一个airsim的UE实例，再开启一个PX4实例。
```sh
bash /home/user/PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh 1
```
暂不推荐，就算在代码里对应写死了键位，你需要首先也要保证在固件里校准手柄

## TroubleShooting
##### 1. 可以换一台网络好的机器直接拉镜像副本，解决内网服务器docker拉不下来的问题
```sh
docker save > <image-name>.tar mypx4_image:v1
docker load < <image-name>.tar
```
我对自己的docker都做了尽可能地预下载处理，保证移植机下载的内容最少。如果想跟朋友分享这个内容，可以配置一个https://transfer.sh/，分块来存：
```sh
docker save mypx4_image:v1 | split -b 5G -d - "mypx4_image.tar.part."
cat mypx4_image.tar.part.* | docker load
```
##### 2. 关于"WSL2本地跑PX4+Windows跑AirSim+Windows跑QGC"的连接问题

如果不用docker，而是在WSL本地跑cmake装PX4来调试，连接问题也会很烦。首先解决PX4与airsim的连接问题，需要在windows的powershell里用ipconfig来找本机的WSL IPv4 Address，这需要设置到AirSim中settings.json的LocalHostIp属性，以及上述教程中所有PX4_SIM_HOST_ADDR中。之后每次跑PX4以前，甚至需要人为指定环境变量来找windows本机，例如：
```sh
export PX4_SIM_HOST_ADDR=172.18.240.1
```
其次，需要解决PX4与QGC的连接问题，在QGroundControl里面，需要添加一个14550的UDP监听，并且需要在可选的指定server处添加<wsl-ip>:18570,其中wsl-ip可以在WSL里面输入ifconfig查到外部ip地址（针对windows网络的eth0）,每次重启WSL2这个ip都会刷新，形如：`172.18.243.55:18570`，最后的端口也不一定是18570，也要注意PX4版本（详见：PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink中的udp_gcs_port_local）。

##### 3.px4编译过程中的自动依赖安装问题

在运行`make px4_sitl_default none_iris`的时候如果遇到警报，可以hit 'u'，避免resolve manually，亲测会省心一点。如果半天卡在`Building for code coverag`e，请检查网速是不是太慢。

##### 4.我想修改编译后的UE游戏的窗口等设置

可以参考下面的链接进行调整:https://blog.csdn.net/qq_33727884/article/details/89487292

##### 5. 我不想用windows,我想全放在安装了ubuntu的本机上做仿真、算法、PX4（我懒得搞环境）

为了测试后续ROS2的offboard功能，可以把我构建的docker container作为虚拟机，后续验证流程可以参考这个[教程](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md)。如果不想用两台机器，想用一台机器做，可以考虑将Dockerfile中的github-token补全，并且取消对UE、Airsim编译的注释，运行`docker build -t mypx4_image:full .`，预计会生成一个300GB左右的image，请留好空间。


## [ROS用户选用-测试中] 使用mavros
如何在airsim上面用MAVROS给PX4无人机发送话题控制，源码编译方式同单无人机教程，需要先在“编译用容器”里编译，然后再启动“运行用容器”如下
```sh
docker run -itd --privileged --env=LOCAL_USER_ID="$(id -u)" --env=PX4_SIM_HOST_ADDR=172.16.13.104 -v /home/wangchao/daizipeng/SBDrone:/src:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --network=host --name=mypx4-0  mypx4_image:v1 /bin/bash
```
其中，`–-env=PX4_SIM_HOST_ADDR=172.16.13.104` 容器添加`PX4_SIM_HOST_ADDR`环境变量，指定远端airsim主机地址；`–-name`后面指定此容器名称。


### 逐步开启mavros服务
在windows设备中，先检查AirSim中setting.json，启动AirSim的某一个map，进入等待服务状态。然后，登录容器
```sh
docker exec -it --user $(id -u) mypx4-0 /bin/bash
```
打开一个窗口，运行2个PX4实例，需要观察到Airsim中有QGC（GPS lock）相关的提示才算成功：
```sh
bash /src/Scripts/run_airsim_sitl.sh 0
bash /src/Scripts/run_airsim_sitl.sh 1
```
注意每次使用ros相关命令时需要输入
```sh
source /opt/ros/melodic/setup.bash
```
打开一个窗口，运行mavros服务，其中第一个端口指定本地主机（127.0.0.1）上的接收端口号（udp_onboard_payload_port_remote），第二个端口指定飞行控制器上的发送端口号（udp_onboard_payload_port_local）。这些可以在上一个窗口的运行日志中，在mavlink的onboard udp port对应上。
```sh
roslaunch mavros px4.launch fcu_url:=udp://:14030@127.0.0.1:14280
roslaunch mavros px4.launch fcu_url:=udp://:14031@127.0.0.1:14281
```

### 使用mavros话题通信在Airsim里手动控制PX4无人机（有点受限于版本V1.12.1）
参考[教程](https://www.youtube.com/watch?v=ZonkdMcwXH4),打开一个窗口，基于mavros发送服务调用指令给px4，实现对无人机的控制，这里给出依次玩耍这些指令的结果：
```sh
# 发起起飞指令，此时不能起飞
rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"

# 解锁无人机，此时可以起飞
rosservice call /mavros/cmd/arming "value: true"

# 无人机起飞
rosservice call /mavros/cmd/arming "value: true"

# 无人机降落
rosservice call /mavros/cmd/land "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
```
也可以基于mavros发送话题给px4，以下是开一个窗口跑position controller：
```sh
# 发送position controller的话题指令
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
pose:
    position:
        x: 1.0
        y: 0.0
        z: 2.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0" -r 20
```
然后再换个窗口设置飞行模式
```sh
# 该服务的目的是让飞行控制器（例如PX4）切换到特定的飞行模式，这里使用的是'OFFBOARD'模式，该模式允许飞行控制器接受来自外部计算机的指令控制飞行。
rosservice call /mavros/set_mode "base mode: 0
custom_mode: 'OFFBOARD'"

# 解锁无人机，执行指令
rosservice call /mavros/cmd/arming "value: true"

# 可以继续发送其它position controller的话题指令
```
以下是velocity controller的画圈demo：
```sh
rostopic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "header
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
twist:
    linear:
        x: 1.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 1.0" -r 20
```