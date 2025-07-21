# ros2_beginner
しんせかい! 

[参考教程](https://fishros.com/d2lros2)


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

## 技巧
* 多线程的循环一定要加 `running_`，并在线程循环中检查它，确保线程在对象销毁前退出。
* 类似于`rclcpp::Client`和`this->get_logger()`都是线程安全的，随便用。
* AI说多线程spin也能无脑用。

## 杂谈
### 250718
学了下基础的发布订阅的编写，以下几点印象：
* colcon有点垃圾了，不如catkin-tools；
* 现在写类可以直接继承官方的类，感觉又闭环了，真不错；
* 全是sharedptr，啥都sharedptr，不知道是我这个教程的人喜欢，还是这就是规范；
* 话题间的QoS比ros1扩展了许多，值得深究。

### 250721
编了Service相关的逻辑，了解了ros2的在新c++版本下的新特性：
* future真挺未来的。没想到是c++11的内容，之前完全没听过；
* QoS还要确保收发相匹配，感觉是有点要求高了；
* node lifecycle看起来挺完备;


<div style="text-align: center;">
    <img src="https://design.ros2.org/img/node_lifecycle/life_cycle_sm.png" alt="ROS2 node lifecycle" height="300">
</div>
