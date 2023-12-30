# AirGroundSim
同时仿真车辆和无人机以支持空地协同相关研究

## What's News!
本系统默认`Airsim（UE4.27）仿真`、`QGroundControl节点`、`手柄节点`在windows里面运行，而`PX4飞控`、`mavros上飞机算法`在同网络环境下的linux机器运行，并支持以下教程。理论上全部都放在linux上也可以运行（主要是`build.cmd`换成`build.sh`，QGC从`.exe`换成相应的`.AppImage`，这部分暂时作为experimental supports。

## 研究进度
- [X] 尝试PhysX下的车机并存（受限于car的残破实现，暂时不支持RC切换，建议使用脚本控制）

- [X] 尝试引入PX4 SITL

- [ ] 添加PythonClient全面支持

- [ ] 添加ROS/ROS2全面支持

- [ ] 验证PX4在gazebo上的自主起降demo，看看能否在airsim跑通 (也可以参考一下[amov](https://wiki.amovlab.com/public/prometheus-wiki/%E7%9B%AE%E6%A0%87%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9D%97-object_detection/%E6%95%99%E5%AD%A6%E4%BE%8B%E7%A8%8B/%E4%BA%8C%E7%BB%B4%E7%A0%81%E6%8E%A2%E5%AF%BB%E9%99%8D%E8%90%BD.html)、[xtdrone](https://github.com/robin-shaun/XTDrone))

- [ ] 尝试用airsim的data recorder训练机载大模型，再用PX4 SITL方式验证这个工作：https://github.com/alaamaalouf/FollowAnything


## 首先是配置的教学，分为两侧
### Windows仿真侧（我的ip：172.23.53.8）
我们需要先安装UE环境，然后下拉代码（此处也可以考虑使用workspace集成的代码）
```sh
git clone https://gitee.com/autopilot-x/air-ground-sim && cd air-ground-sim
```
现在编译AirSim，如果是windows系统需要参考子安大佬的[教程](https://zhuanlan.zhihu.com/p/618440744)，在`VS Developer Command Prompts`窗口使用`build.cmd`，如果是linux系统则用`bash`运行`build.sh`，最好每次修改源码后都用相应的`clean`先刷一下再重新编译，这个过程会在线安装一点依赖，如果看到下面的内容说明编译成功。

[![1fe8a88781982cb705374d58a019c78.png](https://i.postimg.cc/6602XLRY/1fe8a88781982cb705374d58a019c78.png)](https://postimg.cc/2318dW0Z)

这里，一个非常好的设计模式在于，AirSim将无人机的传感器、控制、优化逻辑与具体的地图、材质、任务开发完全解耦，我们编译好的AirSim可以作为通用的插件适配到`UE4.27`以下的绝大多数地图中，再次进行协同编译，组合成一个完整的无人系统UE实例。接下来，按照AirSim的要求，参考如下方式配置`settings.json`即可，在这里可以看到本系统实现了AirSim原版不具备的`Both`模式，可以支持车机共存：
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Both",
    "ClockType": "SteppableClock",
    "PawnPaths":{
        "CustomVehicle":{"PawnBP":"Class'/AirSim/VehicleAdv/SUV/SuvCarPawn.SuvCarPawn_C'"}
    },
    "Vehicles": {
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
            "Cameras": {
                "BottomCamera": {
                    "CaptureSettings": [
                        {
                            "ImageType": 0,
                            "Width": 640,
                            "Height": 480,
                            "FOV_Degrees": 90
                        }
                    ],
                    "X": 0, "Y": 0, "Z": -0.05,
                    "Pitch": -90, "Roll": 0, "Yaw": 0
                }
            },
            "Parameters": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1,
                "LPE_LAT": 47.641468,
                "LPE_LON": -122.140165
            },
            "X": 4,
            "Y": 2,
            "Z": -3,
            "Yaw": 0
        },
        "Car1": {
            "VehicleType": "PhysXCar",
            "PawnPath":"CustomVehicle",
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
            "WindowID": 1,
            "CameraName": "bottom_center",
            "ImageType": 0,
            "VehicleName": "Drone1",
            "Visible": true
        },
        {
            "WindowID": 2,
            "CameraName": "front_center",
            "ImageType": 0,
            "VehicleName": "Drone1",
            "Visible": true
        }
    ]
}
```
具体设置可以遵循[参数教程](https://microsoft.github.io/AirSim/settings/)、[摄像头教程](https://microsoft.github.io/AirSim/image_apis/#available-cameras)。我们以官方默认编译好的UE实例`Blocks`环境为例，在配置完成并启动AirSim实例后，我们可以看到一个无人机和一个无人车同时出现，他们的绝对位置和上述的定义相同。

[![24789885c5c208503ce374805d5bd00.png](https://i.postimg.cc/T3Vkc4BT/24789885c5c208503ce374805d5bd00.png)](https://postimg.cc/68QLWY9P)

为了便于后续观看，我们可以先用manual camera模式（快捷键`M`）来移动键盘到合适的位置待命，当然在仿真的全程过程中，我们使用可以使用这个模式来移动玩家的第一人称视角，并用小键盘123来调用不同类型的外部视角（**目前默认左右分别是车机第一人称，中间是无人机俯视视角**，视角内容和位置的用法均可以在上述`settings.json`中进一步自定义，使用方法和之前AirSim默认用法完全一致，故省略）。下面是在editor里面启动的样例，打包为游戏还会更快些,仅供参考.

[![image.png](https://i.postimg.cc/PqLPqv4F/image.png)](https://postimg.cc/Z9ST7RXr)

### Linux服务侧（我用的ip：172.16.15.188）

接下来是代码运行角度，首先在服务端电脑，适配建飞老师目前的workspace，下拉代码（需要自己解决网络问题）
```sh
git clone --recursive https://gitee.com/autopilot-x/vscode_ros2_workspace && cd vscode_ros2_workspace
```
然后vscode打开后，使用`reopen in container`，会自动下载一个docker并且以container内部方式重新启动这个工程，然后我先用建飞老师本地改的、微信发送的那个版本（看起来就是改了一个飞机的安全高度）的代码适配，所以具体查看这个代码应该`ssh daizipeng@xxx.xxx.xxx.188`的某个container，密码为123456，直接去找对应mavros相关的containers即可，后续服务侧部署教程默认都在这个container里来做。

接下来还是下拉我的projects，为了方便上网我暂时放进了submodules里面，不过这里我们主要关注python client的部分，安装相应依赖(如果不使用代理，可以--proxy选项省略，或用清华源`-i https://pypi.tuna.tsinghua.edu.cn/simple`)，出现问题重装时可以加`--force-reinstall`
```sh
apt-get install libgl1-mesa-glx # 缺个GL的包

python3 -m pip install sympy opencv-python opencv-contrib-python msgpack-rpc-python pygame

git clone https://gitee.com/autopilot-x/air-ground-sim && cd src/air-ground-sim/PythonClient && python3 -m pip install -e .
```

## 接下来是运行demo的教学
这里面因为两侧机器有时候互为发送端、接收端，还容易角色互换，这里给出一个顺序启动的流程。

### 第1步：启动Windows仿真侧的车机仿真
保证客户机上，ue下airsim是正确运行且正确配置的（以上面demo为准），然后保证手柄的接收器插在运行了仿真环境的机器上。

### 第2步：启动Linux服务侧的无人机PX4飞控
在服务端电脑，因为我们可以看到airsim里面的飞机在等待一个PX4的实例，这里需要启动一个PX4-Autopilot，具体方法略，直接编译也可以、用docker也可以、反正注意一下端口正确、都可以，要说注意的主要还是要把这个host地址改为运行仿真的那台机器的ip，以我的ip为例：
```sh
PX4_SIM_HOST_ADDR=172.23.53.8
```
需要注意**每次reset环境（不论什么原因导致）都要重新启动这个PX4实例**，因为飞机会在不pause physics的情况下直接传送回起始地点，不保证PX4是否会崩溃。

如果使用的是这个工程里面的docker来做px4镜像的话，参考指令如下
```sh
bash /home/user/PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh 1
```

### 第3步：启动Windows仿真侧的无人机地面站来监控
看到飞机有`GPS Lock`之后可以打开QGC，做类似如下这种配置即可，这里不做赘述，QGC放linux也没问题，不开QGC其实对于正式演示应该也没问题

[![image.png](https://i.postimg.cc/CKX2D9Vs/image.png)](https://postimg.cc/TyjQv71K)

如果本机就能连接PX4的话这里不需要做设置。

### 第4步：启动Linux服务侧的无人机航拍demo（可不运行，仅用于调试）
这里添加一个基于airsim的拍照demo，每收到一个回车就照一张，保存在工作目录，此时飞机可以用QGC的UI来控制。
```sh
cd AirGroundSim/PythonClient

python3 air_client_receiver.py
```
这个具体拍到的样子应该和当前UE实例中的第二个自定义窗口的无人机俯视视角，是类似的，可以多测试。当然，可以和后续第5步、第8步同步测试，看看UE资源占用而崩溃的情况是否可以接受，因为我现在也没有能力解决这种位置的崩溃问题。

### 第5步：启动Linux服务侧的车辆手柄信令接收端（现在可以运行，最后应该集成到第7步的脚本内部，像第6步那样子做ros的车机控制，之后就不需要再运行了）
运行一个负责接受手柄socket指令，实时控制这辆车的服务，可以看到在控制车辆的过程中，此时可以用QGC的UI指令来控制飞机，多玩玩看，看引入车辆控制后是否容易阻塞，导致程序资源竞争崩溃。这里想手柄控制飞机可以尝试打开这个地方的手柄控制，但后续如果offboard control的时候忘记关，会出现一个手柄同时控制车机的问题，谨记谨记。

[![image.png](https://i.postimg.cc/nLzL0DbF/image.png)](https://postimg.cc/zys5f3G6)

对于任何用户层面修改，目前都没有实际意义解决车辆占用主进程、线程不安全的问题，只能等待有人重写车辆代码。
```sh
cd AirGroundSim/PythonClient

python3 car_client_receiver.py
```
然后我们走第8步，发现可以和第5步组成一个socket控制无人车，如果你加入QGC这块相当于已经可以控制车机，然后再去看崩溃的现象是否可以一定程度上接受。如果能接受，我们就我们尝试关掉第4、5步，跳过第6步，直接走第7步、第8步

### 第6步：启动Linux服务侧的opencv-python-based drone following节点（可不运行，仅用于调试）
这里纯粹写着玩，可以不跑，具体实现的是一个用pythonclient/opencv-python来做追踪。。
```sh
cd AirGroundSim/PythonClient

python3 drone_follow_car.py
```
第3步结束后，可以选择先打开这个玩意，让飞机稳定，然后就可以打开第5步、第8步，我依然加入了拍照的功能，每10秒拍一张，存工作目录下。

### 第7步：启动Linux服务侧的mavros节点（正式演示用，与第4、5、6步互斥）
我把第4步、第5步的`调试用`功能与建飞老师写的主要代码，尝试集成在了一起，相当于建飞老师的代码现在可以直接调用airsim client的api来获取车的信息，我觉得这样对后续开发简单一点，不然这些信息的处理就要交给win编程，发特定的话题给mavros，那样子首先在windows里面写其它信息的socket、然后转码后又转ros topics，写这玩意我就有点麻，先按好写的写。
```sh
# todo 主要是我忘记了ros2与mavros用法了....
# todo 主要是我忘记了ros2与mavros用法了....
# todo 主要是我忘记了ros2与mavros用法了....
# todo 主要是我忘记了ros2与mavros用法了....
# todo 主要是我忘记了ros2与mavros用法了....
```

### 第8步：启动Windows仿真侧的车辆手柄信令发送端
现在我们运行socket实例的发送端节点，发送手柄信息给服务端，这个代码里看到的ip指的是服务端ip，如果你都用的是一台linux跑可视化+算法，其实设置为`localhost`也没关系，这里不再赘述:
```sh
cd src/air-ground-sim/PythonClient

python3 car_joystick_control_sender.py
```
然后我们尝试控制车辆，看看飞机的控制（包括QGC控制、基于我写在python这边的算法控制、基于mavros背后的算法控制）能不能和车辆的手动控制完美共存，完美一定是不可能的，就是能不能勉强接受哈哈哈。

## 后续扩展的一点简要说明

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

## 本工程亮点1：实现车机共存

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

## 本工程亮点2：支持车机联控

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

为了测试后续ROS2的offboard功能，可以把我构建的docker container作为虚拟机，后续验证流程可以参考这个[教程](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md)。如果不想用两台机器，想用一台机器做，可以考虑将[我的Dockerfile](https://github.com/superboySB/AirGroundSim/blob/main/docker/ros2px4/Dockerfile)中的github-token补全，并且取消对UE、Airsim编译的注释，运行`docker build -t mypx4_image:full .`，预计会生成一个300GB左右的image，请留好空间。


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

## Acknowledgement

The work was done when the author visited Qiyuan Lab, supervised by [Chao Wang](https://scholar.google.com/citations?user=qmDGt-kAAAAJ&hl=zh-CN).
