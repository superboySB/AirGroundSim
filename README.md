# AirGroundSim
同时仿真车辆和无人机以支持空地协同相关研究

## 研究进度
[X] 尝试PhysX下的车机并存

[ ] 尝试引入PX4 SITL

[ ] 尝试用airsim的data recorder训练机载大模型，再用PX4 SITL方式验证这个工作：https://github.com/alaamaalouf/FollowAnything

## 部署流程
目前先在windows场景上做测试，基本安装流程参考宁子安的知乎[讲解1](https://zhuanlan.zhihu.com/p/618440744)、[讲解2](https://zhuanlan.zhihu.com/p/619214564)、[讲解3](https://zhuanlan.zhihu.com/p/620006613)，一些关键的用法大佬都已经给出。这里只补充一些对于本项目单独的流程.....

设置`settings.json`内容如下
```json
{
 "SettingsVersion": 1.2,
 "SimMode": "Both",
 "Vehicles": {
    "Drone1": {
       "VehicleType": "SimpleFlight",
        "X": 0.1, "Y": 0, "Z": -3, "Yaw": 0
     },
    "Car1": {
       "VehicleType": "PhysXCar",
       "X": 0, "Y": 0, "Z": -1, "Yaw": 0
     }
 }
}
```
运行

## TroubleShooting
##### 1. 可以换一台网络好的机器直接拉镜像副本，解决docker拉不下来的问题

```sh
docker save > <image-name>.tar sbdrone_image:v1
docker load < <image-name>.tar
```
如果想分享，可以配置一个https://transfer.sh/，分块来存：
```sh
docker save sbdrone_image:v1 | split -b 5G -d - "sbdrone_image.tar.part."
cat sbdrone_image.tar.part.* | docker load
```
##### 2. 关于"WSL2本地跑PX4+Windows跑AirSim+Windows跑QGC"的连接问题

如果不用docker，而是在WSL本地跑cmake装PX4来调试，连接问题也会很烦。首先解决PX4与airsim的连接问题，需要在windows的powershell里用ipconfig来找本机的WSL IPv4 Address，这需要设置到AirSim中settings.json的LocalHostIp属性，以及上述教程中所有PX4_SIM_HOST_ADDR中。之后每次跑PX4以前，甚至需要人为指定环境变量来找windows本机，例如：
```sh
export PX4_SIM_HOST_ADDR=172.18.240.1
```
其次，需要解决PX4与QGC的连接问题，在QGroundControl里面，需要添加一个14550的UDP监听，并且需要在可选的指定server处添加<wsl-ip>:18570,其中wsl-ip可以在WSL里面输入ifconfig查到外部ip地址（针对windows网络的eth0）,每次重启WSL2这个ip都会刷新，形如：172.18.243.55:18570，最后的端口也不一定是18570，也要注意PX4版本（详见：PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink中的udp_gcs_port_local）。

##### 3.px4编译过程中的自动依赖安装问题

在运行`make px4_sitl_default none_iris`的时候如果遇到警报，可以hit 'u'，避免resolve manually，亲测会省心一点。如果半天卡在`Building for code coverag`e，请检查网速是不是太慢。

##### 4.我想修改编译后的UE游戏的窗口等设置

可以参考下面的链接进行调整:https://blog.csdn.net/qq_33727884/article/details/89487292