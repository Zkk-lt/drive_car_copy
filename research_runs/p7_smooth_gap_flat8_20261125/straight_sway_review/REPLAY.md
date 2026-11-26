# 实测冻结请求的同模型配对复算

从 17:54 的原始归档筛出 157 条请求，检查发布序号、消息时间、参考几何、有效配置与状态，以及对应的 256 条因果命令历史。两组都执行原直线规整和 0.025 m/s 弯前速度预瞄。实验组再在可信下游走向门控内应用新增的横向参考及名义车身横摆率代价。两组冷启，采用同一求解网格，8 ms 总预算，预留 1 ms。交替调整两组顺序防止系统负载偏向。每条已接受解都用同一未整形参考独立滚动检查。

从工作区根目录运行，输出到新建临时目录。

```bash
r=research_runs/p7_smooth_gap_flat8_20261125
raw=experiment_runs/smooth_turn_timing_20261126/p7real_20261126T175421_ba218cba
out=$(mktemp -d)
python3 "$r/turn_timing_review/replay_turn_timing.py" extract "$raw" "$out/fixture" --stride 3 \
  --window steady_straight 17:56:03.225 17:56:04.325 \
  --window first_sway 17:56:10.799 17:56:14.376 \
  --window late_sway 17:57:38.150 17:57:39.650 \
  --window winding 17:56:18.400 17:56:21.800 \
  --window broad_right_entry 17:57:27.500 17:57:29.100 \
  --window right_active_turn 17:57:05.800 17:57:07.000
g++ -O2 -std=c++14 -Isrc/move_car/include -I/usr/include/eigen3 \
  "$r/straight_sway_review/replay_supported_course.cpp" -o "$out/replay"
"$out/replay" .025 0 8 < "$out/fixture/requests.txt" > "$out/pairs.csv"
python3 "$r/straight_sway_review/summarize_supported_course.py" \
  "$out/fixture" "$out/pairs.csv" --output "$out/summary.json"
```

复算用于评估实际输入下的门控、名义预测、约束和首拍动作。每拍保留实车的原历史，假设指令没有注入下一拍。因此预测变化无法证明连贯实车闭环改善。墙钟超时与系统并行负载有关，也会影响有预算时选定的解。

## 已执行结果

| 区间 | 请求数 | 旧→新可行 | 走向参考参与 |
|---|---:|---:|---:|
| 第一段横摆 | 48 | 47→47 | 19 |
| 后段横摆 | 20 | 18→18 | 8 |
| 稳定直行 | 13 | 7→7 | 0 |
| 强正反曲率 | 41 | 23→23 | 0 |
| 右弯入口 | 21 | 15→15 | 0 |
| 右弯转向 | 14 | 12→12 | 0 |

两组各 122 个可行序列通过了原始限值复验，均无超时。157 对请求的硬限速、终点和速度时序保持一致。第一段有 17 个请求增加名义横摆率代价。第一段双可行激活样本的首拍绝对转角变化中位数为 +0.00126 rad，0.6 s 预测绝对横摆率变化中位数为 +0.00005 rad/s；后段分别为 +0.00243 rad 和 +0.00081 rad/s。17:56:13.049 首拍仍为 +0.1373 rad，17:56:13.274 首拍从 +0.1079 到 +0.1191 rad。因此这批复算确认了门控作用和可行性，未显示实际样本差异足以降低横摆。

第一次低门槛的二次拟合在 8/14 实际右转和 12/41 强正反曲率样本生效。拉远支持后，正反曲率退出拟合，仍有 2/14 实际右转样本触发；加入名义车身航向门限后，本批明确转弯样本均旁通新增策略。 [最终对照](real_course_replay_summary.json) · [逐请求](real_course_pairs.csv) · [最初筛选](quadratic_initial_screen.json) · [窄门控筛选](quadratic_narrow_screen.json)
