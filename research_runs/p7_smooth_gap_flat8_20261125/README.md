# smooth 感知断续恢复与 flat8

## 实车失效定位

归档 `p7real_20261125T220440_58acac12` 中，PointPillars 于 22:13:33.055–34.766 输出空结果，约 1.71 s。前车重新被识别后又累积了 35 个确认点。控制收到的却始终是段 1→1 的参考，约 0.97 m，证据仍停在 22:13:32.455。整轮实发 569 条 NMPC 命令，控制发布最大间隔 30.47 ms，终段 623 个候选无超时或数值异常。

回放中，旧算法只在后续观测的离散节点选目标。第一个点刚好落在最小 0.24 m 支持距离以内，下一个点超过原有 2.50 m 搜索半径。两点之间还有可用测量线段。只插值选点时，部分曲线仍会在旧段的短折线处超出 0.12 m 观测走廊。

本版在后续已确认观测的连续线段上选取目标，并在旧段范围内尝试一个端点位置、一阶及二阶导数均为零的横向修正。插值用于几何求解，证据时刻取真实测量。原接续身份、新鲜度、观测走廊、起始转向、角度和变化率上限、停车空间与前向进展判定继续生效。

600 帧历史回放中，22:13:36.640 的参考从 **0.972 m 恢复到 2.144 m**，新证据年龄 **0.103 s**，停车余量 **2.044 m**。78 帧恢复轨迹全部通过新鲜度检查。两套既有归档的安静复跑分别为 719 和 706 帧，均无超过 0.12 m 的长度差。20:21 归档首次并发回放曾出现 3 次普通拟合短缩，安静复跑未再出现。最新回放新版本最长计算 50.13 ms，出现在之后的证据过期制动帧。 [重构对照](evidence/gap_results_summary.json)

## 求解更新

flat8 仍实际优化八个样条自由度。每个延迟查询点复用固定的前三个样条控制点的导数和，剩余四个随优化变化。迟滞正向预测和对应解析导数共用主值区间快速路径，只有越过 ±π 或范围异常时走原三角归一化。上次已实现的转向先验投影一并编入，用实际发布间隔估计前一拍转向斜率。状态、速度、转向、jerk、参考与命令复验约束保持原值。

| 实车请求来源 | 样本 | 配对 8 ms P50 旧→新 | P95 旧→新 | 完整约束复验通过 旧→新 |
| --- | ---: | ---: | ---: | ---: |
| 20:39 | 96 × 3 | 4.40→3.23 ms | 6.72→6.11 ms | 273→274 |
| 21:49 | 47 × 3 | 5.30→3.36 ms | 7.76→5.93 ms | 99→103 |
| 22:04 | 95 × 3 | 4.25→2.91 ms | 7.00→5.66 ms | 215→219 |

数值复算在 80 ms 诊断窗口中可行性逐条一致，三个集合最大首拍转角差分别为 0.00438、约 0 和 0.00283 rad。8 ms 设置保持固定，三组预算失败分别为 14→14、6→2 和 7→3。车体速度已超过动态安全限速的失败仍受原保护。22:04 轮的相关拒绝中，195 次连下一条指令也无法在该限速和原 jerk 约束内到达，放宽速度限制会掩盖来源。这些数值衡量冻结因果输入下的求解和约束，真实车辆的横摆与跟踪收益等待新一轮闭环。 [配对记录](evidence/flat8_ab.json)

## 原启动入口

当前受监督试跑入口选择 `p7_smooth_supported_course_20261126`，会话名为 `smooth_supported_course_20261126`。标准 `devel` 的 flat8 worker 在[同向曲率收紧预瞄](turn_timing_review/README.md)基础上，加入[支持段拟合与横摆率目标](straight_sway_review/README.md)。carTop 使用本工作区的原启动入口和修复缓冲容量的驱动。独立 PointPillars 工作区的 `platoon_tracking.launch` 使用 0.60 m 轴距。

18:19 室内 smooth 实测选为后续对照基线。标准入口的控制器、重构器和求解进程已经与该轮实测逐一核对。源码、配置、二进制和运行身份保存在 [基线清单](straight_sway_review/baseline_181938/BASELINE.json)及其快照。原始事件单独归档，启动下一轮仍用 `prepare` 分配新的 run_id。

控制和重构终端先进入 `/home/ubuntu/drive_car_copy`，加载 ROS：

```bash
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```

每轮准备一次。准备命令分配新的 run_id 并更新两个 launch 共用的选择器。使用标准 `devel/setup.bash`，无需设置 `TRIAL`、添加 launch 参数或另外运行 `run.sh start`。

```bash
bash research_runs/p7_smooth_gap_flat8_20261125/run.sh prepare
```

原来的重构和控制指令按顺序在各自终端运行。为保留实测时的 RAM 日志设置，在这两个终端启动前执行 `export ROS_LOG_DIR="$(readlink -f src/move_car/.smooth_trial)/ros_logs"`。

```bash
roslaunch move_car trajectory_reconstructor.launch
roslaunch move_car movecar_improved.launch
```

另开终端运行以下两条。先启动记录器，等收到必要话题后再启动操作者授权。原有唯一出口、候选复验、回退和授权保护继续执行。限速 0.40 m/s。

```bash
bash research_runs/p7_smooth_gap_flat8_20261125/run.sh record
bash research_runs/p7_smooth_gap_flat8_20261125/run.sh enable
```

结束时先退出 `enable` 并确认停稳，再停止重构、控制和记录，然后归档。

```bash
bash research_runs/p7_smooth_gap_flat8_20261125/run.sh archive
```

底盘保留原 `rosrun move_car movecar` 入口，两套工作区只启动一个 carTop。感知继续从 PointPillars 工作区执行 `roslaunch pointpillars_ros platoon_tracking.launch`。如需查阅基础配置，两条 move_car launch 均接受 `smooth:=false`，用于关闭新试跑覆盖层。同一时刻只运行一套控制入口。

`build_smooth_integrated` / `devel_smooth_integrated` 用于源码构建检查。受监督试跑锁定[软件验证记录](straight_sway_review/SOFTWARE_ACCEPTANCE.json)中的运行文件和策略。重编后 SHA 改变时，原软件验证记录不再对应运行文件。17:54 实测的 worker 备份在 `turn_timing_review/runtime`。
