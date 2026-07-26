# Role: 高级自动驾驶C++控制算法工程师 & ROS架构师

# Task: 开发面向虚拟牵引车队的【单文件版：CPS融合二次规划与空间域NMPC轨迹跟踪节点】

## 1. 系统核心目标与极致工程约束 (Core Objectives & Constraints)

* **核心目标**：在低速（0.1m/s ~ 1.0m/s）工况下，基于纯几何跳变路径，彻底消除“一走一停”，实现对前车轨迹的极高保真度绝对复刻。
* **工程架构极简约束 (CRITICAL)**：**所有的面向对象代码（包括类的声明、实现、ROS Node Wrapper）必须全部高内聚实现在同一个 `virtual_towing_node.cpp` 文件中！** 配合一个 `virtual_towing.launch` 文件完成参数加载与节点启动。严禁拆分 `.h` 和多余的 `.cpp` 文件。
* **目标硬件**：阿克曼底盘小车（轴距 $L=0.6m$，最大转角 $45^\circ$）。
* **输入限制**：`reference_path` 仅有每次跳变更新的空间几何点 $(x, y)$，无速度参考。
* **技术栈**：C++11/14, ROS Melodic, `Eigen3`, `CasADi`, `OSQP-Eigen`。

---

## 2. 核心算法理论推导 (Mathematical Formulations - 必须严格遵循)

### 2.1 动态跳变系下的平坦误差提取 (Frenet Error Extraction)
由于 `reference_path` 基于自车当前坐标系，自车状态恒为 $(x=0, y=0, \theta=0)$。
* 寻找距离 $(0,0)$ 最近的路径点作为投影点 $P_{proj}$（弧长 $s=0$）。
* 提取横向误差 $e_y$ 和航向误差 $e_\theta = 0 - \theta_{proj}$。
* 沿路径以 $\Delta s$（如0.1m）重采样提取前方曲率序列 $[\kappa_0, \dots, \kappa_N]$。

### 2.2 【核心创新】CPS 信息物理融合的 SQP 速度规划 (CPS-Fused SQP Planning)
**传统方法弊端**：仅用几何曲率规划速度，忽略了底盘执行的物理迟滞，容易导致极限工况下底层失控。
**CPS 融合算法设计**：将底盘的物理反馈抽象为“CPS风险因子”，反向调制上层的速度边界。
1. **物理层风险提取 (Physical Risk)**：
   实时计算转向执行残差 $d_\delta = |\delta_{real} - \delta_{cmd\_history}|$ 和 IMU Z轴加速度方差 $\sigma_z$。
   定义 CPS 风险因子：$\gamma_{cps} = \alpha_1 d_\delta + \alpha_2 \sigma_z$ ，并将其限幅在 $[0, 0.8]$ 之间。
2. **信息层二次规划 (Cyber SQP)**：
   设 $b_i = v_i^2$，控制量 $a_i = \frac{db_i}{ds}$。
   目标函数：$\min \sum \left( w_v(b_i - v_{cruise}^2)^2 + w_a a_i^2 + w_j (a_i - a_{i-1})^2 \right)$。
   状态约束：$b_{i+1} = b_i + a_i \Delta s$。
3. **【关键】CPS 融合动态安全边界 (CPS-Modulated Bound)**：
   定义基础附着力对应极限上限 $b_{limit\_base} = \frac{\mu_{base} \cdot g}{|\kappa_i| + \epsilon}$。
   加入 CPS 风险降级约束：
   $$b_i \le b_{limit\_base} \cdot (1 - \gamma_{cps})$$
   *(原理：底盘响应越迟滞、越颠簸，物理风险越大，规划层主动压低速度上限)*。
4. **防停滞底线约束 (Anti-Stop)**：
   强制 $b_i \ge v_{\min}^2$（如 $0.1^2$），彻底拒绝一走一停。

### 2.3 空间域非线性模型预测控制 (Spatial-Domain NMPC)
* **状态变量**：$X = [e_y, e_\theta]^T$；**控制变量**：$U = [\delta]$。
* **空间偏微分方程 (解耦时间与速度)**：
  $$e_y' = (1 - \kappa_k \cdot e_y) \tan(e_\theta)$$
  $$e_\theta' = \frac{\tan(\delta)}{L} \frac{1 - \kappa_k \cdot e_y}{\cos(e_\theta)} - \kappa_k$$
  利用 `CasADi` 构建以 $s$ 为步长（$\Delta s$）的欧拉离散化约束方程。
* **CPS 前馈**：最终下发转向角 $\delta_{final} = \delta_{NMPC} + k_{comp} \cdot (\delta_{real} - \delta_{cmd\_history})$。

---

## 3. 单文件 C++ OOP 架构设计 (Single-File Architecture)

请在 `virtual_towing_node.cpp` 中自上而下严格按以下顺序定义并实现类：

1. **结构体定义区**：定义 `VehicleState`, `ControlCmd` 等数据结构。
2. **`CPSStateObserver` 类**：
   * 负责缓存物理层反馈，计算转向延迟残差 $d_\delta$。
   * 计算并对外提供 CPS 风险因子 $\gamma_{cps}$。
3. **`SpatialSQPPlanner` 类**：
   * 内部集成 `OSQP-Eigen`。
   * 提供方法：`std::vector<double> plan(curvatures, gamma_cps, current_v)`，内部构建 2.2 节的矩阵并求解。
4. **`FlatnessErrorExtractor` 类**：
   * 接收 `nav_msgs::Path`，在局部基底 $(0,0,0)$ 下投影提取 $e_y, e_\theta$ 和重采样的 $\kappa$ 序列。
5. **`SpatialNMPC` 类**：
   * 内部集成 `CasADi`。定义符号变量 `SX`，严格依据 2.3 节构建空间运动学模型与代价函数。
   * 提供方法：`double solve(e_y, e_theta, curvatures)`。
6. **`VirtualTowingNode` 主节点类**：
   * 包含以上所有类的实例成员。
   * 订阅：`/reference_path`, `/vehicle/wheel_speed`, `/vehicle/steering`, `/imu/data`。
   * 发布：`/cmd_vel`。
   * 核心逻辑：在 `ros::Timer` (如 50Hz) 回调中串联：
     `提取误差 -> CPS观测风险 -> SQP解算速度 -> NMPC解算转角 -> 合并前馈 -> 下发指令 -> 打印终端Log`。
7. **`main` 函数**：初始化 ROS，实例化 `VirtualTowingNode`，调用 `ros::spin()`。

---

## 4. 智能体执行要求 (Execution Instructions)

由于所有代码必须生成在同一个 `.cpp` 文件中，代码量较大。请你作为智能体，严格按照以下步骤生成，**每完成一个 Step 请停下来等我回复“继续”后再输出下一段代码，以防截断。**

* **Step 1: ROS 包外壳与 Launch 文件**。
  输出 `CMakeLists.txt`、`package.xml` 和 `launch/virtual_towing.launch` (包含 L=0.6, v_min=0.1 等参数)。
* **Step 2: CPP 文件头部与观测器/规划器**。
  输出 `virtual_towing_node.cpp` 的 `#include` 区、结构体定义、`CPSStateObserver` 类的完整实现、以及 **核心的 `SpatialSQPPlanner` 类的完整实现 (必须包含 CPS 风险因子 $\gamma_{cps}$ 对速度上限的缩放逻辑)**。
* **Step 3: 误差提取与空间域 NMPC**。
  继续输出 `FlatnessErrorExtractor` 类和 `SpatialNMPC` 类的完整实现 (严格使用 CasADi 编写对弧长 $s$ 偏导的预测模型)。
* **Step 4: ROS Wrapper 与 Main 函数**。
  继续输出 `VirtualTowingNode` 类的完整实现和 `main()` 函数，并在定时器回调中整合数据流向.
