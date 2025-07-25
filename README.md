# ros2_beginner
しんせかい! 

* [鱼香ROS](https://fishros.com/d2lros2)：糟糕
* [古月居](https://book.guyuehome.com/)：较好，不过全是Python
* [创客智造](https://ros.ncnynl.com/)：翻译的不错
* [ROS](https://docs.ros.org/)：唯一真神

## 杂谈
### 250725
学习Nav2中BT相关，该[教程](https://zhuanlan.zhihu.com/p/681879055)不错。
调整了下镜像中的相关配置，现在可以通过ssh直接连接了。拿vscode还不错。
```SH
ssh root@localhost -p 2222
```


### 250724
原来osrf也有jazzy的官方镜像，那没事了。
```SH
# With GPU
sudo docker run -it \
  --gpus all \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --net=host \
  spike667/ros2_jazzy:v0.2 \
  zsh

# No GPU
sudo docker run -it \
  --net=host \
  spike667/ros2_jazzy:v0.2 \
  zsh

# 可能低版本docker不支持上面的类似--env，需要用以下类似方式启动
sudo docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  spike667/ros2_jazzy:v0.2 \
  zsh
```
可以运行如下指令测试，测试GPU在容器中的启用。宿主机GPU还要安装个`nvidia-container-toolkit`。宿主机需要执行类似`xhost +`的操作才能呼出图形界面。
```SH
ros2 launch ros_gz_sim_demos rgbd_camera_bridge.launch.py
```
运行存在以下问题：
* `--net=host`指令加了后，在同一个宿主机上的多个容器间就没法通信了；但是在多个宿主机上启动的话，加上该指令可以正常通信，因此按需配置容器；
* jazzy无法跟宿主机20.04的foxy完美通信，还遇到过cpu占用飙升的情况，没有humble与foxy的适配好。

测试了多个宿主机各自启动容器的测试，没问题，host外加在同一局域网内，没有通信问题。


### 250723
今天主要搞了docker的配置，建了个docker hub的账号，基于官方的humble镜像，配了下环境还有zsh啥的，上传到自己的hub里了，指令是`docker pull spike667/ros2_humble:v0.2`*（250724备注:已弃用，转向jazzy，参考杂谈-250724）*，tag可能更新。
对于在docker启动指令，现在使用的是：
```SH
sudo docker run -it spike667/ros2_humble:v0.2 /bin/zsh
```
可以满足宿主机foxy和镜像里humble的通信，对于其中有特别几点做如下说明：
* 不可以使用`--network host`，这会导致能获得话题，但是读取不到话题值；
* 当前版本在ce进行测试，未在docker desktop进行测试（经测试，desktop版宿主机ping不通容器，可能需要进一步配置），p.s.两者的切换可以通过以下指令修改：
   ```SH
   (ros2) ~ docker context ls                    
   NAME            DESCRIPTION                               DOCKER ENDPOINT                                 ERROR
   default *       Current DOCKER_HOST based configuration   unix:///var/run/docker.sock                     
   desktop-linux   Docker Desktop                            unix:///home/chen/.docker/desktop/docker.sock   
   (ros2) ~ docker context use default
   ```
* 不同节点间的通信要满足一下几点：
   * `ROS_DOMAIN_ID`：阈必然要一致，默认都是0；
   * `RMW_IMPLEMENTATION`：这个不确定需不需要一致，[教程](https://discourse.openrobotics.org/t/ros-cross-distribution-communication/27335)都说要一致，本次都用的`rmw_fastrtps_cpp`，humble的镜像也默认都是这个。

### 250722
ros2的action实际大差不差。配了下docker还没配好。
* action服务时通过action/status维护的，当status isDone触发后，才会call服务端要result，以此确保接收到指令；
* 通过QoS配置好像甚至能获得历史数据。

### 250721
编了Service相关的逻辑，了解了ros2的在新c++版本下的新特性：
* future真挺未来的。没想到是c++11的内容，之前完全没听过；
* QoS还要确保收发相匹配，感觉是有点要求高了；
* node lifecycle看起来挺完备。

<div style="text-align: center;">
    <img src="https://design.ros2.org/img/node_lifecycle/life_cycle_sm.png" alt="ROS2 node lifecycle" height="300">
</div>

### 250718
学了下基础的发布订阅的编写，以下几点印象：
* colcon有点垃圾了，不如catkin-tools；
* 现在写类可以直接继承官方的类，感觉又闭环了，真不错；
* 全是sharedptr，啥都sharedptr，不知道是我这个教程的人喜欢，还是这就是规范；
* 话题间的QoS比ros1扩展了许多，值得深究。

## 小记
### this_thread
`std::this_thread` 提供了与**当前线程**相关的功能，比如：
- 让当前线程休眠（暂停执行）
- 获取当前线程的 ID
- 让当前线程让出 CPU 时间片

#### 常用函数列表（`std::this_thread::`）

| 函数 | 说明 |
|------|------|
| `sleep_for(...)` | 让当前线程休眠一段**固定时间** |
| `sleep_until(...)` | 让当前线程休眠到**某个时间点** |
| `yield()` | 主动让出 CPU 时间片，让其他线程先运行 |
| `get_id()` | 获取当前线程的唯一 ID |

### atomic vs mutex
* `std::mutex` 是一个互斥量，用于保护共享资源。  
* `std::lock_guard` 是 RAII（资源获取即初始化）风格的锁管理器，**在构造时加锁，析构时自动解锁**。

| 特性 | `std::atomic` | `std::lock_guard` + `std::mutex` |
|------|----------------|-----------------------------|
| 类型 | 原子变量 | 互斥锁 |
| 是否使用锁 | 否（通常是无锁的） | 是 |
| 适用对象 | 单一变量 | 多个变量、代码块、资源 |
| 性能 | 更快（无锁） | 稍慢（有锁） |
| 线程安全 | ✅ | ✅ |
| 可读性 | 更简洁 | 更复杂（但更灵活） |
| 使用难度 | 简单 | 稍复杂 |

* **对于大多数基础类型（如 `int`、`bool`、指针等）的 `std::atomic<T>` 变量，直接使用 `=` 赋值和 `=` 读取是可以的，它们是原子操作**。  
* 但如果你需要更**精细的控制**（比如指定内存顺序 `memory_order`），就需要使用 `store()` 和 `load()`。

---
### placeholders
在 C++ 中，`std::placeholders::_1` 是一个**占位符（placeholder）**，用于 `std::bind` 函数中，表示将来调用绑定函数时传入的 **第一个参数**。
```cpp
#include <functional>
#include <iostream>

void print_sum(int a, int b) {
    std::cout << a + b << std::endl;
}

int main() {
    // 固定 a = 2，b 是将来传入的参数
    auto f = std::bind(print_sum, 2, std::placeholders::_1);
    f(5);  // 输出 7
}
```
`std::placeholders::_1` 表示调用 `f(x)` 时传入的第一个参数（这里是 `5`）

---
### sharedPtr
**`shared_ptr` 必须配合 `std::make_shared<T>()` 使用，否则就是空指针，访问成员会导致崩溃。**
那些ros2里的create相当于就是make_shared了。

```cpp
example_interfaces::srv::AddTwoInts::Request::SharedPtr req;
```

这行代码**只是声明了一个 `shared_ptr` 变量 `req`，但没有分配内存，也没有指向任何对象**。

此时 `req` 是一个**空指针（null pointer）**，当你执行：

```cpp
req->a = count_;
```

就会访问空指针，**导致未定义行为（Undefined Behavior）**，**最常见的是程序直接崩溃或退出**。


正确写法：使用 `std::make_shared`

```cpp
auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
```

---
### IDL
`.idl` 文件是 **DDS（Data Distribution Service）标准的接口定义语言文件**，它定义了 ROS 2 中使用的消息、服务、动作的结构，用于跨平台、跨语言通信。

常见数据类型：

| IDL 类型 | 对应 ROS 2 类型 | 说明 |
|----------|------------------|------|
| `long` | `int32` | 32 位整型 |
| `unsigned long` | `uint32` | 无符号 32 位整型 |
| `long long` | `int64` | 64 位整型 |
| `float` | `float32` | 单精度浮点数 |
| `double` | `float64` | 双精度浮点数 |
| `boolean` | `bool` | 布尔值 |
| `string` | `string` | 字符串 |
| `sequence<type>` | `type[]` | 动态数组 |
| `MyType_ my_field` | `MyType my_field` | 自定义类型嵌套 |


| ROS 2 文件 | 内容 | 自动生成的 `.idl` 文件 |
|-------------|------|-------------------------|
| `Person.msg` | `string name;`<br>`int32 age;`<br>`float32 height;` | `struct Person_ {`<br>`string name;`<br>`long age;`<br>`float height;`<br>`};` |
| `AddTwoInts.srv` | `int32 a;`<br>`int32 b;`<br>`---`<br>`int32 sum;` | `struct AddTwoInts_Request_ {`<br>`long a;`<br>`long b;`<br>`};`<br>`struct AddTwoInts_Response_ {`<br>`long sum;`<br>`};` |
| `Fibonacci.action` | `int32 order;`<br>`---`<br>`int32[] sequence;` | `struct Fibonacci_Goal_ {`<br>`long order;`<br>`};`<br>`struct Fibonacci_Result_ {`<br>`sequence<long> sequence;`<br>`};` |


.idl 和 ROS 2 的关系图
```
ROS 2 .msg/.srv/.action 文件
        ↓
    rosidl 工具链
        ↓
生成 .idl 文件（DDS 标准）
        ↓
Fast DDS / Cyclone DDS 使用
        ↓
生成 C++、Python 等语言的通信代码
```

---
### QoS

在 ROS2（Robot Operating System 2）中，**QoS（Quality of Service，服务质量）** 是一种用于控制和优化通信行为的机制，主要用于发布-订阅（Publisher-Subscriber）模型和客户端-服务器（Client-Service）模型中的消息传递。QoS 允许开发者根据应用需求调整通信的可靠性、性能和资源使用。以下是对 ROS2 QoS 的详细介绍，包括其主要策略、区分方式，以及以实际代码为例的实现。

#### 1. QoS 概述
QoS 定义了消息传递的行为，例如消息是否可靠、是否保存历史记录、优先级等。ROS2 的 QoS 基于 DDS（Data Distribution Service）标准，提供了灵活的配置选项。QoS 策略主要应用于：
- **发布者（Publisher）** 和 **订阅者（Subscriber）** 的主题匹配。
- **服务（Service）** 和 **客户端（Client）** 的通信。
- **动作（Action）** 的目标、反馈和结果通信。

QoS 配置通过 **QoS 配置文件（QoS Profile）** 设置，包含多个 QoS 策略的组合。ROS2 提供了预定义的 QoS 配置文件，也可以自定义配置。

#### 2. QoS 主要策略
ROS2 中的 QoS 由以下几个关键策略组成，每个策略控制通信的某一方面：

1. **可靠性（Reliability）**：
   - **Reliable**：确保消息可靠传输，适合需要高可靠性的场景（如控制命令）。
   - **Best Effort**：不保证消息送达，适合高频但允许丢失的场景（如传感器数据）。
   - **选项**：`rmw_qos_profile_t::reliability`（`RELIABLE` 或 `BEST_EFFORT`）。

2. **历史记录（History）**：
   - **Keep Last**：保留最近的 N 条消息（由 `depth` 参数决定），新消息覆盖旧消息。
   - **Keep All**：保留所有消息，直到队列满（受资源限制）。
   - **选项**：`rmw_qos_profile_t::history`（`KEEP_LAST` 或 `KEEP_ALL`）。

3. **队列深度（Depth）**：
   - 定义历史记录中保存的消息数量（仅在 `KEEP_LAST` 模式下有效）。
   - **选项**：`rmw_qos_profile_t::depth`（整数，典型值为 1 到 10）。

4. **寿命（Lifespan）**：
   - 指定消息的有效时间，超过此时间的消息将被丢弃。
   - **选项**：`rmw_qos_profile_t::lifespan`（以纳秒为单位的时间间隔）。

5. **截止时间（Deadline）**：
   - 定义消息发送或接收的预期时间间隔。如果未按时发送或接收，触发回调。
   - **选项**：`rmw_qos_profile_t::deadline`（以纳秒为单位的时间间隔）。

6. **存活时间（Liveliness）**：
   - 确保发布者定期声明其存在，防止订阅者认为发布者已断开。
   - **Automatic**：由中间件自动管理。
   - **Manual by Topic**：发布者需定期发送消息以证明存活。
   - **选项**：`rmw_qos_profile_t::liveliness`（`AUTOMATIC` 或 `MANUAL_BY_TOPIC`）。
   - **租期（Lease Duration）**：`rmw_qos_profile_t::liveliness_lease_duration`（存活声明的超时时间）。

7. **耐久性（Durability）**：
   - **Volatile**：消息不保存，新订阅者无法接收历史消息。
   - **Transient Local**：保存最近的消息，新订阅者可接收（需 `KEEP_LAST` 和非零 `depth`）。
   - **选项**：`rmw_qos_profile_t::durability`（`VOLATILE` 或 `TRANSIENT_LOCAL`）。

#### 3. QoS 预定义配置文件
ROS2 提供了以下常用 QoS 配置文件（在 `rmw_qos_profiles.h` 中定义），便于快速应用：

- **`rmw_qos_profile_sensor_data`**：
  - 可靠性：Best Effort
  - 历史记录：Keep Last
  - 深度：5
  - 耐久性：Volatile
  - 适合：传感器数据（如激光雷达、摄像头）。
- **`rmw_qos_profile_default`**：
  - 可靠性：Reliable
  - 历史记录：Keep Last
  - 深度：10
  - 耐久性：Volatile
  - 适合：通用场景。
- **`rmw_qos_profile_services_default`**：
  - 可靠性：Reliable
  - 历史记录：Keep Last
  - 深度：10
  - 耐久性：Volatile
  - 适合：服务和客户端通信。
- **`rmw_qos_profile_system_default`**：
  - 由底层 DDS 中间件决定，通常与 `rmw_qos_profile_default` 类似。
- **`rmw_qos_profile_unknown`**：
  - 未定义，依赖底层实现。

自定义 QoS 配置文件可以通过设置 `rclcpp::QoS` 对象实现。

#### 4. QoS 兼容性
发布者和订阅者的 QoS 配置必须兼容，否则无法建立通信。关键兼容性规则：
- **可靠性**：Reliable 发布者只能与 Reliable 订阅者匹配；Best Effort 发布者可与两者匹配。
- **耐久性**：Transient Local 发布者需与 Transient Local 订阅者匹配；Volatile 发布者可与两者匹配。
- **历史记录和深度**：订阅者的深度需大于或等于发布者的深度（对于 `KEEP_LAST`）。
- **截止时间和存活时间**：订阅者的要求需不严格于发布者。

---
### 通信模型
<div style="text-align: center;">
    <img src="https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.10_DDS/image-20220528020740057.jpg" alt="Model" height="200">
</div>

第一种，点对点模型，许多客户端连接到一个服务端，每次通信时，通信双方必须建立一条连接。当通信节点增多时，连接数也会增多。而且每个客户端都需要知道服务器的具体地址和所提供的服务，一旦服务器地址发生变化，所有客户端都会受到影响。

第二种，Broker模型，针对点对点模型进行了优化，由Broker集中处理所有人的请求，并进一步找到真正能响应该服务的角色。这样客户端就不用关心服务器的具体地址了。不过问题也很明显，Broker作为核心，它的处理速度会影响所有节点的效率，当系统规模增长到一定程度，Broker就会成为整个系统的性能瓶颈。更麻烦是，如果Broker发生异常，可能导致整个系统都无法正常运转。之前的ROS1系统，使用的就是类似这样的架构。

第三种，广播模型，所有节点都可以在通道上广播消息，并且节点都可以收到消息。这个模型解决了服务器地址的问题，而且通信双方也不用单独建立连接，但是广播通道上的消息太多了，所有节点都必须关心每条消息，其实很多是和自己没有关系的。

第四种，就是以数据为中心的DDS模型了，这种模型与广播模型有些类似，所有节点都可以在DataBus上发布和订阅消息。但它的先进之处在于，通信中包含了很多并行的通路，每个节点可以只关心自己感兴趣的消息，忽略不感兴趣的消息，有点像是一个旋转火锅，各种好吃的都在这个DataBus传送，我们只需要拿自己想吃的就行，其他的和我们没有关系。

### ament & colcon
在 ROS 2（Robot Operating System 2）中，**ament** 和 **colcon** 是构建系统中的两个重要工具，但它们的作用和职责不同，相互协作以完成 ROS 2 软件包的构建、测试和安装。以下是它们的关系和各自的功能：

#### 1. **ament**
**定义**: ament 是一个构建系统框架，专为 ROS 2 设计，用于定义和管理软件包的构建过程。

**作用**:
- 提供了一种标准化的方式来描述 ROS 2 软件包的元数据（如依赖、构建规则等）。
- 定义了软件包的结构和构建逻辑，通常通过 `CMake`（对于 C++）或 `Python` 脚本（对于Python 包）来实现。
- ament 提供了 `ament_cmake` 和 `ament_python` 等工具，用于支持不同类型的软件包（++ 或 Python）。
- 负责处理软件包的依赖管理、编译、测试和安装等步骤。
- 它是一个“前端”工具，定义了如何构建单个软件包。

**核心组件**:
- `ament_cmake`: 用于 C++ 软件包的 CMake 扩展，简化了 ROS 2 软件包的 CMakeListstxt 编写。
- `ament_python`: 用于 Python 软件包的构建工具，简化 Python 包的安装和依赖管理。
- `ament_lint`: 用于代码静态检查和格式化，确保代码质量。

**配置文件**:
- 每个 ROS 2 软件包的根目录下通常有一个 `package.xml` 文件，定义了软件包的元数据（名称、版本、依赖等），这是 ament 识别和管理软件包的基础。

#### 2. **colcon**
**定义**: colcon（Collection of packages）是一个命令行工具，用于协调和管理多个 ROS 2 软件包的构建过程。

**作用**:
  - 作为一个“后端”工具，colcon 负责批量处理工作空间中的多个软件包，调用 ament 或其他构建系统来完成实际的构建。
  - 提供并行构建、依赖解析、构建顺序优化等功能。
  - 支持多种构建系统（如 ament_cmake、ament_python、纯 CMake、Python setuptools 等），因此不仅限于 ROS 2 软件包。
  - 提供用户友好的命令行接口，用于构建、测试和安装整个工作空间。

**核心功能**:
  - 解析工作空间中所有软件包的依赖关系，确定构建顺序。
  - 调用适当的构建工具（如 ament_cmake 或其他）来执行构建。
  - 支持多种构建配置（如 Debug、Release）和隔离安装（如 `--merge-install` 或 `--symlink-install`）。

#### 3. **ament 和 colcon 的关系**
**协作关系**:
  - ament 是构建系统的“前端”，定义了单个软件包的构建逻辑和规则。
  - colcon 是构建系统的“后端”，负责管理整个工作空间中多个软件包的构建流程，调用 ament（或其他构建系统）来完成具体构建任务。
  - 简单来说，ament 告诉 colcon “如何构建一个软件包”，而 colcon 负责“何时以及以什么顺序构建多个软件包”。

**工作流程**:
  1. 你在 ROS 2 工作空间中创建多个软件包，每个软件包包含 `package.xml` 和相应的构建脚本（CMakeLists.txt 或 setup.py），这些都由 ament 定义和支持。
  2. 使用 `colcon build` 命令，colcon 会扫描工作空间中的所有软件包，解析依赖关系，并按正确的顺序调用 ament（或其他构建工具）来构建每个软件包。
  3. 构建完成后，colcon 将生成的二进制文件、库、脚本等安装到指定目录（如 `install/`）。

**类比**:
  - 可以把 ament 看作“厨师的食谱”，定义了如何烹饪一道菜（单个软件包的构建规则）。
  - colcon 则是“厨房经理”，负责协调多个厨师（ament 构建工具），确保所有菜品（软件包）按正确顺序完成。

#### 4. **与Catkin的对比**
* catkin ≈ ament（构建规则）+ colcon（工作空间管理）。
* ament 类似于 catkin 的“核心构建逻辑”，专注于定义软件包如何构建。
* colcon 类似于 catkin_make 或 catkin build，但更通用、灵活，负责协调整个工作空间的构建。
* ROS 2 的设计将 catkin 的功能拆分，模块化更强，适合更复杂的开发场景。

## 技巧
* 多线程的循环一定要加 `running_`，并在线程循环中检查它，确保线程在对象销毁前退出。
* 类似于`rclcpp::Client`和`this->get_logger()`都是线程安全的，随便用。
* AI说多线程spin也能无脑用。

