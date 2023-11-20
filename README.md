# 基于Airsim二次开发的空地协同仿真系统

## 使用方法

为了使用本系统，我们需要先安装环境 (如果不使用代理，可以--proxy选项省略)

```sh
git clone https://github.com/superboySB/AirGroundSim.git && cd AirGroundSim && git checkout two_joystick_control

pip install --proxy="http://127.0.0.1:10809" sympy opencv-python opencv-contrib-python torch msgpack-rpc-python pygame

cd PythonClient && pip install -e .
```

现在编译airsim，如果是windows系统在`VS Developer Command Prompts`窗口使用`build.cmd`，如果是linux系统则用`bash`运行`build.sh`，最好每次修改源码后都用相应的`clean`先刷一下再重新编译，这个过程会在线安装一点依赖，如果看到下面的内容说明编译成功。

[![1fe8a88781982cb705374d58a019c78.png](https://i.postimg.cc/6602XLRY/1fe8a88781982cb705374d58a019c78.png)](https://postimg.cc/2318dW0Z)

这里，一个非常好的设计模式在于，airsim将无人机的传感器、控制、优化逻辑与具体的地图、材质、任务开发完全解耦，我们编译好的airsim可以作为通用的插件适配到`UE4.27`以下的绝大多数地图中，再次进行协同编译，组合成一个完整的无人系统UE实例。接下来，按照airsim的要求，参考如下方式配置`settings.json`即可，在这里可以看到本系统实现了airsim原版不具备的`Both`模式，可以支持车机共存：

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Both",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": -2,
            "Y": -2,
            "Z": -3,
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

我们以官方默认编译好的UE实例`Blocks`环境为例，在配置完成并启动airsim实例后，我们可以看到一个无人机和一个无人车同时出现，他们的绝对位置和上述的定义相同。

[![24789885c5c208503ce374805d5bd00.png](https://i.postimg.cc/T3Vkc4BT/24789885c5c208503ce374805d5bd00.png)](https://postimg.cc/68QLWY9P)

为了便于后续观看，我们可以先用manual camera模式（快捷键`M`）来移动键盘到合适的位置待命，当然在仿真的全程过程中，我们使用可以使用这个模式来移动玩家的第一人称视角，并用小键盘123来调用不同类型的相机（相机内容和位置均可以在上述`settings.json`中进一步自定义，使用方法和之前airsim默认用法完全一致，故省略）。接下来，为了同时控制无人车和无人机，我们分别选用xbox、wfly两个手柄，前者相当于车辆的方向盘控制器，后者是有名的国产无人机专业遥控，均支持无线控制。

[![95a6cae5c516edc818a452da1a12a35.jpg](https://i.postimg.cc/Xv3ywH0S/95a6cae5c516edc818a452da1a12a35.jpg)](https://postimg.cc/18CtSKtJ)

以windows系统为例，在插入无线接收器后，我们能够在windows系统的`游戏控制器`中分别查到相应的设备标记，这些设备标记也会在后续程序识别不同输入设备的时候，作为参考的index来区分。

[![60c88db9e04ae806f5c484b4066db71.png](https://i.postimg.cc/5yPJjtqk/60c88db9e04ae806f5c484b4066db71.png)](https://postimg.cc/SYz3vkPW)

有了Airsim的UE实例，现在我们启动PythonClient节点，以基于grpc的话题订阅方式来实时读取控制流

```sh
python PythonClient/two_joysticks_control_air_ground.py
```

程序启动后，我们就可以分别用两个手柄同时控制无人机和无人车了，实现一个车机协同控制的简单例子。

[![5a3ad53a5075b67bebe3df9ff923964.png](https://i.postimg.cc/7brJLmHL/5a3ad53a5075b67bebe3df9ff923964.png)](https://postimg.cc/H83kZ07G)

类比于airsim中添加多个无人机和无人车的方式，我们同样可以设置更多数量的无人机和无人车。

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Both",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": -2,
            "Y": -2,
            "Z": -3,
            "Yaw": 0
        },
        "Drone2": {
            "VehicleType": "SimpleFlight",
            "X": 2,
            "Y": 2,
            "Z": -3,
            "Yaw": 0
        },
        "Car1": {
            "VehicleType": "PhysXCar",
            "X": 0,
            "Y": 0,
            "Z": -1,
            "Yaw": 66
        },
        "Car2": {
            "VehicleType": "PhysXCar",
            "X": 5,
            "Y": 5,
            "Z": -1,
            "Yaw": 5
        }
        
    }
}
```

由于遥控的数量有限，此处仅展示多个车机共存的情况，在必要时，仍然可以通过PythonClient来指定交替控制的逻辑。

[![cdafbc5136612f35f5e78535dfc9d29.png](https://i.postimg.cc/ZqssHYLw/cdafbc5136612f35f5e78535dfc9d29.png)](https://postimg.cc/zLWkXr6h)

## 技术创新点1：实现车机共存

整个的实现过程中挖掘出了airsim对车支持能力的严重不足，对此做了江湖救急的改动：

1. 由于airsim的车辆控制绑死要接受`updateCarControls`函数（接受手柄/方向盘/键盘等外设信息），哪怕你用键盘移动视角、也会被视为车辆移动，严重影响体验。此外，虽然有手柄的读取代码，但其实不支持飞机用的set/get rc_data的功能，导致手柄配对了RC后、反而[无法控制车辆](https://github.com/microsoft/AirSim/issues/4843)了，而如果飞机接受了正确的RC、车辆却会跟着飞机做协动。综上，我决定屏蔽车辆的所有读取外设的功能，并且禁止用户实际设置`RCData`这个属性，使得键盘无法控制除了玩家视角以外的任何实体，便于调整全局观察的视角。

2. 车辆的另一个问题就是会默认开启`SpringArm`模式，它会自动强制玩家视角跟随主体Actor移动，导致不管之前怎么控制飞机、用的什么模式，只要你开始控制车，画面就立即变成了跟随车辆的模式。为了能够一直保持俯视视角下观察车机的宏观移动，并响应上一点修改，我取消了车辆强制调用`attachSpringArm`的代码。

3. 在进一步的测试中，发现车辆


如果直接全部删除后用ros/python api也无法控制，因为它们实现的时候虽然只控制RPM等、但也都会过这个函数来update state，我这里就只屏蔽了车辆的键盘控制功能，保留原来的功能。这样子的话，在本代码中，不论如何设置settings，键盘无法控制除了玩家视角以外的任何实体，便于调整观察视角。


## 技术创新点2：支持车机联控