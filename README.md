# 基于AirSim二次开发的空地协同仿真系统

## 使用方法

为了使用本系统，我们需要先安装环境 (如果不使用代理，可以--proxy选项省略)

```sh
git clone https://github.com/superboySB/AirGroundSim.git && cd AirGroundSim && git checkout two_joystick_control

pip install --proxy="http://127.0.0.1:10809" sympy opencv-python opencv-contrib-python torch msgpack-rpc-python pygame

cd PythonClient && pip install -e .
```

现在编译AirSim，如果是windows系统在`VS Developer Command Prompts`窗口使用`build.cmd`，如果是linux系统则用`bash`运行`build.sh`，最好每次修改源码后都用相应的`clean`先刷一下再重新编译，这个过程会在线安装一点依赖，如果看到下面的内容说明编译成功。

[![1fe8a88781982cb705374d58a019c78.png](https://i.postimg.cc/6602XLRY/1fe8a88781982cb705374d58a019c78.png)](https://postimg.cc/2318dW0Z)

这里，一个非常好的设计模式在于，AirSim将无人机的传感器、控制、优化逻辑与具体的地图、材质、任务开发完全解耦，我们编译好的AirSim可以作为通用的插件适配到`UE4.27`以下的绝大多数地图中，再次进行协同编译，组合成一个完整的无人系统UE实例。接下来，按照AirSim的要求，参考如下方式配置`settings.json`即可，在这里可以看到本系统实现了AirSim原版不具备的`Both`模式，可以支持车机共存：

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Both",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": 4,
            "Y": 2,
            "Z": -3,
            "Yaw": 0
        },
        "Car1": {
            "VehicleType": "PhysXCar",
            "X": 0,
            "Y": 0,
            "Z": -1,
            "Yaw": 20
        }     
    },
    "SubWindows": [
        {
            "WindowID": 0,
            "CameraName": "fpv",
            "ImageType": 0,
            "VehicleName": "Car1",
            "Visible": true
        },
        {
            "WindowID": 2,
            "CameraName": "bottom_center",
            "ImageType": 0,
            "VehicleName": "Drone1",
            "Visible": true
        }
    ]
}
```

具体设置可以遵循[参数教程](https://microsoft.github.io/AirSim/settings/)、[摄像头教程](https://microsoft.github.io/AirSim/image_apis/#available-cameras)。我们以官方默认编译好的UE实例`Blocks`环境为例，在配置完成并启动AirSim实例后，我们可以看到一个无人机和一个无人车同时出现，他们的绝对位置和上述的定义相同。

[![24789885c5c208503ce374805d5bd00.png](https://i.postimg.cc/T3Vkc4BT/24789885c5c208503ce374805d5bd00.png)](https://postimg.cc/68QLWY9P)

为了便于后续观看，我们可以先用manual camera模式（快捷键`M`）来移动键盘到合适的位置待命，当然在仿真的全程过程中，我们使用可以使用这个模式来移动玩家的第一人称视角，并用小键盘123来调用不同类型的相机（相机内容和位置均可以在上述`settings.json`中进一步自定义，使用方法和之前AirSim默认用法完全一致，故省略）。接下来，为了同时控制无人车和无人机，我们分别选用xbox、wfly两个手柄，前者相当于车辆的方向盘控制器，后者是有名的国产无人机专业遥控，均支持无线控制。

[![95a6cae5c516edc818a452da1a12a35.jpg](https://i.postimg.cc/Xv3ywH0S/95a6cae5c516edc818a452da1a12a35.jpg)](https://postimg.cc/18CtSKtJ)

以windows系统为例，在插入无线接收器后，我们能够在windows系统的`游戏控制器`中分别查到相应的设备标记，这些设备标记也会在后续程序识别不同输入设备的时候，作为参考的index来区分。

[![60c88db9e04ae806f5c484b4066db71.png](https://i.postimg.cc/5yPJjtqk/60c88db9e04ae806f5c484b4066db71.png)](https://postimg.cc/SYz3vkPW)

有了Airsim的UE实例，现在我们启动PythonClient节点，分别开启控制车机的节点，分别同时以基于grpc的话题订阅方式来实时读取控制流

```sh
# 开启顺序最好是如下顺序：
python PythonClient/air_joystick_control.py
python PythonClient/car_joystick_control.py
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

值得一提的是，我们还可以使用UE编辑器来启动AirSim，这样就能够自由地编辑我们的场景、引入更加逼真的要素。

[![image.png](https://i.postimg.cc/D0h2vcbJ/image.png)](https://postimg.cc/wyWKFL2g)

下面，我们构建一个更加逼真的山河景观场景，并演示一个最终的效果作为参考，这也再次印证了本系统遵循着“自主单位、场景任务之间松耦合"的系统工程范式，非常灵活、易于扩展，接下来会从两个角度讲解本项目主要在工程实现上的亮点。

[![1700516534888.png](https://i.postimg.cc/h4rMh5Nm/1700516534888.png)](https://postimg.cc/yWD0bv16)

## 工程亮点1：实现车机共存

目前业界支持空地协同、并且支持大多数车载/机载传感器的仿真较少，而AirSim虽然支持无人机、无人车的仿真多年，但仍然只能单独仿真其中一种，不能支持车机共存。本系统通过一定的代码重构与扩展，最终实现了这一缺失已久的功能，现在列出主要的几个自研的修改逻辑：

1. 原有代码在`AirLib/include/common/AirSimSettings.hpp`文件中实现了读取用户定义的`settings.json`文件的相应配置、并且进入不同类型的模式（无人机/无人车/浏览模式），并进一步迭代读取相应的详细子配置，本系统修改了这个逻辑，引入一个`Both`模式，并在此时支持读取所有原先模式的子配置，除了仿真无人机、无人车需要这些应有的配置意外，保留浏览模式的配置也便于我们随时设置观察视角，这对于一个大规模的空地协同仿真是比较关键的，因为全局视角往往会比单个无人系统的第一人称视角更加宏观。

[![image.png](https://i.postimg.cc/tJjXfT9s/image.png)](https://postimg.cc/VrKQ0zrw)

2. 与前一点类似，`Unreal/Plugins/AirSim/Source/SimHUD/SimHUD.cpp`文件中，考虑到用户也有可能在仿真实例中希望临时修改配置，因此支持通过`F1`快捷键来可交互地尝试其它仿真模式，团队也增加了对这种零代码方式的支持，确保Both模式可以正常使用。

[![image.png](https://i.postimg.cc/KjCYZ2ST/image.png)](https://postimg.cc/9wt2psfX)

3. 当前AirSim中的所有实例都是通过API来调用传感器信息或进行控制的，考虑到无人机、无人车构建出来的相应对象彼此以后，这里将原先的API状态机由单个指针扩展到了一个指针数组，并将这样的扩展应用到了大量代码中，实现车机共存。鉴于相关代码修改较多，以下仅展示`Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp`及其头文件中的相应修改方式作为参考。

[![image.png](https://i.postimg.cc/763vCr27/image.png)](https://postimg.cc/34xf669J)

[![image.png](https://i.postimg.cc/wxDxNSpw/image.png)](https://postimg.cc/bdNjcVdn)

4. 基于上述代码的扩展，本系统完整实现了一个能够包含车机两种`SimMode`的`Both`模式，并且可以灵活实例化相应的Pawn实例，并集成了两种模式之前的全部功能，并尽可能抽象化出了通用的接口，这里给出相应的头文件作为演示。

[![image.png](https://i.postimg.cc/NFXxGQ2q/image.png)](https://postimg.cc/RJMtX5TX)

## 工程亮点2：支持车机联控

相比于无人机，AirSim对无人车的各方面支持能力较为欠缺，且默认只能接受一条手柄的控制流，因此，为了保证遥控器能独立控制无人机、无人车，本系统开发了相应的控制代码`PythonClient/two_joysticks_control_air_ground.py`，通过PythonClient（实现方式类似ROS Client），采用grpc的方式调用各个遥控器的摇杆内容，并发送话题给相应的无人机和无人车，实现对它们的灵活控制。

[![image.png](https://i.postimg.cc/Nfrh3XNL/image.png)](https://postimg.cc/bs8FbZfj)

为了支持该功能，团队对源码做出了进一步修改，此处简单列出主要的开发细节：

1. 由于AirSim原生的车辆控制默认要接受`updateCarControls`函数（接受手柄/方向盘/键盘等外设信息），哪怕你用键盘移动视角、也会被视为车辆移动，严重影响体验。此外，虽然有手柄的读取代码，但其实不支持飞机用的set/get rc_data的功能，导致手柄配对了RC后、反而[无法控制车辆](https://github.com/microsoft/AirSim/issues/4843)了，而如果飞机接受了正确的RC、车辆却会跟着飞机做协动。综上，我决定屏蔽车辆的所有读取外设的功能，并且禁止用户实际设置`RCData`这个属性，使得键盘无法控制除了玩家视角以外的任何实体，便于用户能随时用键盘来调整全局观察的视角。

[![image.png](https://i.postimg.cc/vZ05s42x/image.png)](https://postimg.cc/yDSDh8H7)


2. 车辆的另一个问题就是会默认开启`SpringArm`模式，它会自动强制玩家视角跟随主体Actor移动，导致不管之前怎么控制飞机、用的什么模式，一旦开始控制车，画面就立即变成了跟随车辆的模式。为了能够一直保持俯视视角下观察车机的宏观移动，并响应上一点修改，我取消了车辆强制调用`attachSpringArm`的代码，详见`Unreal/Plugins/AirSim/Source/CameraDirector.cpp`的相应位置。

[![image.png](https://i.postimg.cc/CLvNsrCq/image.png)](https://postimg.cc/hfdTSr5D)

[![image.png](https://i.postimg.cc/pd95Mqn3/image.png)](https://postimg.cc/xk21KG5P)

3. 在进一步的测试中，发现车辆的控制指令都是在主进程中调用的，不支持飞机控制时可以自行设置的Async模式，这对于车机同时控制的场景容易出现资源竞争的问题，为了尽量减少该问题，团队在保证仿真正确性的前提下，对空指令做了处理，并且在无人机控制的时候添加原生的`join`功能（因为AirSim的无人车控制不支持这个功能），确保无人机收到指令后、无人车再继续收到指令，实现了一个简单的同步。

[![image.png](https://i.postimg.cc/DyhDMNbp/image.png)](https://postimg.cc/vDP313h5)

4. 此外，团队发现原有的AirSim代码在不启动QGroundControl地面站软件的情况下，无法通过Python/ROS/原生UE中的任何一种方式来控制`PX4Multirotor`无人机，因此在`AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`文件补全了相应的mavlink话题内容，与PX4-Autopilot的SITL节点之间进行有效地通信，确保可以不额外打开地面站软件的情况下直接控制无人机，提高平台与用户间的可交互性。

[![image.png](https://i.postimg.cc/jjxyVxdk/image.png)](https://postimg.cc/18d8wZFp)

## Acknowledgement

The work was done when the author visited Qiyuan Lab, supervised by [Chao Wang](https://scholar.google.com/citations?user=qmDGt-kAAAAJ&hl=zh-CN).