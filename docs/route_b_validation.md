# 路线 B 本次验证记录

日期：2026-10-11。范围：当前仓库仿真基线，RG 默认、AG95 可选。未连接真机，未进行本次头显人工验收。

## 已完成的验证

| 项目 | 方法与结果 | 验证范围 |
|---|---|---|
| 启动描述与退出 | 四个 launch 的 `--show-args` 通过；受管独立进程组的立即退出、运行后退出检查通过 | 没有启动完整 ROS 图 |
| 构建与安装 | `colcon build --packages-select aubo_i5_teleop --cmake-clean-cache` 通过 | ROS 包构建，不代表 ROS 图运行通过 |
| 映射数学 | `scripts/test_clutch_mapper.py` 全部通过 | 局部旋转、增益连续性、滑移限制、四元数半球连续性、连续步进 |
| 路线 B 回归 | `tests/test_route_b.py` 14 项通过 | 实际 tracker 回调、实测锚点与会话握手、旧目标拒收、反馈缺失、心跳断流、跟随误差、限速、mapper／tracker 同锚点、Mock／Quest 夹爪输入契约及录制 |
| 跨模型工具帧 | `tools/verify_tool_frames.py`：RG、AG95 各 30 个随机姿态 | 独立 URDF 链 FK 与 MuJoCo TCP 位置误差 < 10⁻¹⁵ m、旋转矩阵误差 < 3×10⁻¹⁵；并非实机标定 |
| 夹爪开合 | 开、合端点比较完整 TCP 位姿 | RG、AG95 开合不改变工具位姿 |
| 厂家型号参数 | `tools/check_robot_spec.py` 通过 | 自有 URDF、MoveIt、正式 MJCF 与位置执行器的限位一致；外部依赖保持固定版本 |
| MuJoCo 动态执行 | 真实物理积分，tracker 回调直接接收状态、输出执行器命令 | 10 mm 平移测试误差约 0.13 mm；后续 −0.05 rad 局部旋转测试位置误差约 0.22 mm、角误差约 0.0186 rad，未触发跟随异常；保留现有 IK 姿态偏置与到达容差 |
| 真实 rosbag 写入 | SQLite rosbag writer 开始、暂停、恢复、结束 | 生命周期事件及时间索引条数与袋内消息数一致，原始消息时间戳保持不变 |
| 相机离屏渲染 | RG、AG95 各三路 `--selftest` | 腕部 1280×720，左右 640×480，图像非空；不证明 DDS 传输率或动态图像同步 |

以上 tracker 和 logger 回归通过直接调用回调、使用真实 MuJoCo／rosbag writer 完成，**没有经过 DDS 消息传输**。数值结果来自固定种子的短轨迹，只用于回归，不能外推到任意工作空间或真机稳定性。

## 可复现命令

在现有依赖环境中运行：

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/VR_teleoperation/ros2_ws/install/setup.bash
PY=/home/lcw/tomato_robot/.venv/bin/python
PKG=/home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop
"$PY" "$PKG/scripts/test_clutch_mapper.py"
"$PY" "$PKG/tests/test_route_b.py"
"$PY" "$PKG/tools/check_robot_spec.py"
"$PY" "$PKG/tools/verify_tool_frames.py"
MUJOCO_GL=egl "$PY" "$PKG/scripts/multi_camera_renderer.py" --selftest
```

正式启动和录制服务见根目录 README。原路线 A 的文件保存在 `docs/archive_route_a/`，不再参与包构建、安装或正式启动。

## 参数依据与适用范围

型号限位来自仓库内 **AUBO-i5 & CB4 user manual V4.5.11，2021.05.19**，正文印刷 p29、附录 E pVI：六轴 ±175°；前三轴 150°/s，后三轴 180°/s。`config/robot_spec.yaml` 记录出处和实际设备待确认状态。没有据此编造厂家加速度参数。

工作界限 ±2.94 rad、输入与反馈超时、跟随误差阈值是项目控制参数。RG 已认可归位角度及相机安装保留；腕3=2.94 已在项目工作界限上，需要操作者在多姿态下检查可用方向。外部官方 URDF 副本只覆写六轴限位，未改几何／惯性。

固定版本的官方 AUBO ROS2 驱动要求并导出 position、velocity 命令接口，但当前 `write()` 执行的是 `Servoj(position)`／`servoJoint`。这说明当前驱动实际位置命令路径，不能推断整个 AUBO 产品或所有 SDK 都无速度 API。当前 MuJoCo 硬件块不能只替换插件便接真机：关节范围、命令／状态接口数、夹爪独立驱动都需要另建硬件描述。

## 尚未完成的运行验收

1. **完整本机 ROS 图。** 沙箱内 DDS 初始化报 `failed to enumerate interfaces for udp`。尝试在独立 ROS domain 运行本机测试时，自动审批服务返回 HTTP 503，升级执行未获准执行。因此未验证完整 launch 的运行、跨节点握手、退出行为和录制吞吐；只验证了构建、启动描述和离线逻辑。
2. **Quest 手感。** WebXR `xyzw` 顺序已按本地 relay 源码修正；局部轴约定 `Rx(π)` 为明确数学约定。仍需在夹爪朝前、朝侧和朝下时分别验证三轴旋转、组合旋转、纯旋转 TCP 漂移及松开／重接 Grip。测试时观察仿真实际状态与目标，不能只看手柄可视化。
3. **相机视场与动态场景图像。** 离屏样图已人工查看：RG 当前 ready 姿态下，三路画面主要为背景／地面，TCP 不在视场内。腕部相机坐标中的 TCP 约为 `(−0.0014, −0.1060, −0.1308)` m，垂直偏角约39°，超过当前58°垂直视场的一半；左右相机也未覆盖该 ready TCP。遵照已确认约束，未改变相机安装与归位姿态。需要结合正式采集任务检查移动后的覆盖范围，再明确是否调整。 旁路相机只重建关节；RG 当前场景无动态物体。AG95 旧 demo 自由方块未同步，不能用它的旁路图像直接采动态抓取示范。需要物理实例直接出图，或同步完整物体状态。
4. **LeRobot 数据集生成。** Logger 输出保留源时间和各阶段时间的异步原始流；统一时钟不等于每条信号同时采样。采样率、时间匹配容差、旧帧／重复帧／缺帧策略及最终 action 表达尚需 Dataset Builder 明确。
5. **真机部署。** 控制柜与固件、模型/TCP、设备速度限制、实际停止行为和 RG 通信协议仍待确认。本次未创建虚构真机驱动，未设置未确认的厂家参数。当前 B 没有在线自碰撞和环境碰撞命令拦截；跟随保护只是仿真初值，不构成真机安全验收。
