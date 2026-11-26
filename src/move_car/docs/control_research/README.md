# 控制研究工程入口

当前验收阶段为 **P3.5，研究 NMPC 仍仅限影子计算及离线闭环**。S8 原控制器仍是生产默认；未开展草地/沥青参数寻优，也未开放实车 NMPC 接管。

- [P35_AUDIT.md](P35_AUDIT.md)：真实数学与调用链、12 个决策变量与 5 个积分状态、平坦性差距及最小侵入升级计划。
- [NMPC_RUNTIME_BENCHMARK.md](NMPC_RUNTIME_BENCHMARK.md)：同集前后计时、完整预算测试、编译/验证命令和实车准入限制。
- [STATE_ESTIMATION_VALIDATION.md](STATE_ESTIMATION_VALIDATION.md)：传感器来源、坐标和时序契约、RLS 可观测性及未完成实测项。
- [ADAPTATION_ABLATION.md](ADAPTATION_ABLATION.md)：全部退化组合、独立消融、物理状态与紧急停车分开评价。
- [P3.5 证据包](results/p35/README.md)：源码差异、原始逐周期结果、CPU/构建元数据和测试日志；[最终清单](results/p35/manifest.json)保存对应哈希。

当前 P2/P3 实现、调用链、运行方法、数学边界见 [P2/P3 实施报告](P23_IMPLEMENTATION.md)，实际验证结果见 [P2/P3 测试结果](P23_TEST_RESULTS.md)。已实现共享低维求解器、状态/工况估计和自适应运行时，接入原控制节点的可选影子计算及独立闭环仿真；生产默认仍为旧控制器。以下保留 P0/P1 交付时的说明，描述的是该阶段状态。

## P0/P1 历史交付

本轮保留稳定轨迹重构、SLQR、纵向仲裁、所有现有安全分支和两个生产 launch。新增数据/配置功能默认不参与控制。没有实现或启用实车 NMPC、路面自动分类、滑移估计、自适应参数更新和自动寻优发布。

阅读顺序：

1. [P0 审计与映射](P0_AUDIT.md)：真实调用链、坐标/传感器源、安全边界、生效/未生效功能与性能限制。
2. [数学架构及 P2 计划](ARCHITECTURE_P2_PLAN.md)：平坦重构公式、12 维决策参数化、耦合模型/代价/约束、SLQR关系和阶段验收。
3. [测试结果](TEST_RESULTS.md)：实际运行结果及失败项；不把历史测试或仿真计划记为通过。
4. [代码差异](code_changes.patch)：生产 C++ 小改动及新增接口/脚本/配置/测试。参数全文见 parameter_inventory.csv 和 baseline_launch_parameters.json。

## 修改范围与隔离

- `src/control_pub.cpp`：默认 false 的 enable_research_telemetry；可选 steady_clock 求解/回调计时及诊断发布；SolveInfo 增加迭代计数；统一 run_id 仅用于日志。computeCommand 的输出公式、调用顺序与安全分支没有替换。
- `src/trajectory_reconstructor.cpp`：原 Path/status 发布后，可选旁路 ReferenceInfo，复制几何/解析曲率和制动/限速/质量；统一日志 run_id。原输出路径、切线和 BRK1 不变。
- `msg/ReferenceSample.msg / ReferenceInfo.msg / ControlResearchTelemetry.msg`：新增消息，不修改既有消息 MD5 或语义。
- `config/control/*`：研究配置与元数据，不在生产 launch 中 rosparam load。新参数当前不驱动控制器。
- `scripts/control_research/{config,recording,analysis}.py`，record_control_run.py、analyze_control_run.py、validate_control_config.py：离线配置/分析与独立记录进程。
- `launch/control_research_record.launch`：只启动被动 recorder，既不启动原控制器，也不启动 CAN/IMU 驱动。
- CMake/package：消息生成、脚本安装和 YAML/tf2_msgs 运行依赖。ROS 部分使用系统 Python2（Melodic），离线部分使用 Python3+PyYAML。
- 新测试：test_control_research.py、test_research_ros_serialization.py、run_p1_baseline_equivalence.py、run_research_recorder_integration.py；旧测试保持原样并记录其失配。

## 分层配置

```
config/control/
  base.yaml                 车辆/名义模型及P2性能种子
  safety.yaml               当前部分重要安全边界的冻结研究快照
  terrain/smooth.yaml
  terrain/grass_bumpy.yaml
  terrain/asphalt.yaml
  terrain/unknown.yaml
  adaptation.yaml           P3幅度/可信度/滤波种子，未执行
  tuning/candidates/README.md
```

每份 YAML 包含 schema_version、kind、terrain、baseline_version、status、evidence、valid_range、features；每个数值包含 value/unit/class/bounds/basis。

A 类：校验器内冻结锚点与 YAML 双重检查，候选不能通过改 bounds 同时放宽 max_speed 等。此表未迁移所有旧安全设置，完整生产规则仍在原 launch/CPP。
B 类：几何/轮速比例/执行器时间常数与横摆增益，后续修改必须走有观测证据的辨识审核；P1 无自动修改接口。
C 类：代价/预测和滤波等性能参数，仅允许硬编码 catalog 的有限范围；缺参、未知键、重复 YAML 键、NaN/Inf、错误单位/等级、场景、版本、超范围、非整数离散时域参数均拒绝。

初始三个已知场景权重相同，不隐含附着排序；unknown 的 .10 m/s 只是未来保守配置种子，**本轮不会改变真实车辆速度**。这些不是分场景最优配置，也不是已验证平滑场地基线的替代。正式基线仍是原生产 launch + 当前 CPP。

验证所有场景（只读）：

```bash
for terrain in smooth grass_bumpy asphalt unknown; do
  python3 src/move_car/scripts/validate_control_config.py src/move_car/config/control --terrain "$terrain"
done
```

## 实验记录操作（本轮没有执行实车启动）

先选择唯一的字母/数字/下划线/短横线 run_id。以下示例用于以后受控实验：

```bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/drive_car_copy/devel/setup.bash
roslaunch move_car control_research_record.launch \
  run_id:=20261106_smooth_routeA_01 terrain:=smooth route:=routeA \
  target_speed_mps:=0.30 conditions:=dry_battery_full
```

该 launch **只启动 recorder**，只写 `/control_research/run_id` 参数；不加载生产控制配置、不启动车辆控制节点。run 目录存在则拒绝覆盖。若要记录完整碰撞点云输入，加 `record_clouds:=true` 并单独验证带宽/磁盘，默认 false 的数据不支持完整碰撞回放。

可选遥测开关需在以后实验的两个节点启动前设置，不能以重启节点为由覆盖尚未归档的旧 CSV：

```bash
rosparam set /publish_speed_turn/enable_research_telemetry true
rosparam set /trajectory_reconstructor/enable_research_telemetry true
```

此处开关只发诊断；**不是 NMPC 开关**。两个节点启动时读取同一 `/control_research/run_id`，所以 recorder/该参数应先准备好。已经运行的节点不会动态采纳新 run_id 或 telemetry 开关；否则旁路消息中的 ID 不一致会被分析器计入 mismatch 并排除。不要为补齐日志盲目重启正在运行的车。

记录目录默认 `research_runs/<run_id>/`：

- manifest.json：人工工况标签、路线、目标速度/条件、Git HEAD 和 src dirty 状态、两个 CPP/launch 哈希与快照、已构建二进制哈希、活动参数快照/哈希、研究配置哈希、采集完整性/写盘错误。磁盘二进制哈希不证明运行进程身份一致；标志始终未验证。
- config/：本 run 使用的 base/safety/terrain/adaptation 快照。
- source/、source.diff：两个 CPP/launch 快照及工作树 tracked diff（不把 Git HEAD 单独当作脏工作区版本）。
- events.jsonl：**每个原始消息各自一条**，话题/type/MD5/publisher、源 header 纳秒（没有就是 null/0）、接收 ROS/wall/monotonic 时间、frame、有效性原因与置信度、可读 payload、完整 ROS 二进制 base64。

/car_message、/carvel、/system_speed 分别记录，不做绝对值或融合；原始负轮速、NaN/Inf 也保留在二进制中。JSON 中非有限数写为带 nonfinite 标签的对象，不能错误读成 0。惯导未填 quaternion/frames 会有明确异常原因。`usable_for_state_estimation=false/confidence=0` 表示 P1 未验证标定和时钟，不代表原始包被丢弃。

订阅回调只取时标、把消息引用放入容量 1024 的队列；序列化和磁盘写入在独立进程的写线程。满队列丢弃并计数，不阻塞车辆控制回调；写盘异常触发 recorder 失败/关闭，不能伪称记录成功。accepted/written 是队列与缓冲写计数，不是 fsync 持久性证明；ROS 传输/订阅队列丢包数仍 unknown。异常断电可能留 manifest.state=recording 或尾部不完整行，分析器不把它视为闭合、完整实验。

## 离线分析接口

```bash
python3 src/move_car/scripts/analyze_control_run.py research_runs/20261106_smooth_routeA_01 \
  --output /tmp/20261106_smooth_routeA_01_metrics.json
```

输出文件必须是未存在的 `.json`，防止误覆盖原始日志和 YAML。默认打印也可以。输入必须有同一 run_id 的 manifest/events；遥测内部 ID 不一致会被剔除并计数。

已支持：绝对横向/航向误差 RMSE/P50/P95/P99/max、**混合速度**跟踪误差、yaw响应误差、命令转角变化率/变向数、命令加速度/jerk、限幅/限速率/安全停/碰撞停样本比例、求解失败比例、求解/compute/部分回调耗时分位数、相对拟议预算超限比例、源龄/非单调时间/接收间隔分布、原因计数。

“命令”加速度不是真实车体加速度；转角变向计数不是完整频域振荡判定；sample ratio 不是事件率。跨零/重复/倒退时标和 >.20 s gap 不求指令导数。没有数据时输出 null/count=0，不输出虚假的 0 失败率。

P1 未支持独立定位真值性能、完整模型预测/约束违反、路线完成率、μ/滑移/侧偏可信估计，结果 `unavailable` 给出原因。solve_over_proposed_budget_ratio 是观测统计；旧 solver 没有 deadline，因此不是“超时已安全回退率”。

旧文件可只读汇总，不能获得统一 run_id 或精确回放：

```bash
python3 src/move_car/scripts/analyze_control_run.py \
  --legacy-controller cdata/runtime_debug.csv \
  --legacy-reconstructor data/runtime_debug.csv
```

## 阶段验收与后续

P0 基础完成：代码/来源/安全/参数映射、实际构建和历史测试缺口、host helper 性能与旧数据局限。未完成：现场传感器协议/固件独立性、尺度与坐标标定、整节点目标硬件预算。

P1 基础完成：配置文件/拒绝校验、独立原始采集实现、统一 run_id、旁路参考及控制诊断、离线分析入口。已验证消息序列化和独立写线程、配置与指标处理、基线 helper 数值等价，以及隔离ROS合成流的端到端记录/分析；未验证完整 ROS 实时采集吞吐、目标机运行期开销及实车连续性。

P0/P1 当时的下一阶段计划见 ARCHITECTURE_P2_PLAN；其 P2/P3 软件实施已更新到 P23_IMPLEMENTATION。P4 配置寻优发布及 P5 实车闭环验收仍未实施。
