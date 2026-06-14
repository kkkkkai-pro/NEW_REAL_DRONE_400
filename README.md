# NEW_REAL_DRONE_400

这是一个基于 [REAL_DRONE_400](https://github.com/NEU-REAL/REAL_DRONE_400/) 扩展的比赛任务版本。它在原始无人机硬件、EGO-Planner、PX4Ctrl、FAST-LIO 能力基础上，新增了面向空中具身智能赛项的任务编排、移动障碍物区逻辑、拱形障碍物区逻辑，以及 Ubuntu 20.04 最小仿真脚本。

本文档按你提供的调整后比赛口径编写：

- 场地按 `12m x 8m` 理解；
- 精确降落按机体几何中心与降落区中心的水平距离判定；
- 现场得分环中心、降落区精确坐标通常赛前给出，因此 [`src/planner/plan_manage/config/competition_mission.yaml`](src/planner/plan_manage/config/competition_mission.yaml) 仍需要按现场进行二次标定。

## 1. 仓库总览

- [`docs/`](docs/)：补充说明文档。目前主要是 Ubuntu 20.04 最小仿真快速开始说明。
- [`files/`](files/)：项目配套材料目录，例如采购清单等非源码文件。
- [`home_shfiles/`](home_shfiles/)：原始仓库提供的真机快捷启动脚本，偏向传感器、定位、规划、控制、起降等人工分步启动。
- [`misc/`](misc/)：图片和展示素材。
- [`release/`](release/)：结构件、装配件、STEP、切片等硬件发布资料。
- [`scripts/`](scripts/)：新增的辅助脚本目录，目前重点是 [`scripts/sim/`](scripts/sim/) 下的 Ubuntu 20.04 最小仿真脚本。
- [`src/`](src/)：ROS 源码目录，包含原始规划、仿真、真机控制模块，以及这次新增的竞赛任务管理代码。

## 2. 相对原始 REAL_DRONE_400 的新增内容

以下内容以 `origin/main` 为基线。

### 2.1 新增文件总表

| 文件 | 作用 | 在真机/仿真中的实际用途 |
| --- | --- | --- |
| [`.gitignore`](.gitignore) | 过滤构建产物、IDE 缓存、日志和本地环境文件。 | 不参与飞行逻辑；避免把 `build/`、`devel/`、`.vscode/`、日志等误提交。 |
| [`docs/ubuntu20_sim_quickstart.md`](docs/ubuntu20_sim_quickstart.md) | 说明 Ubuntu 20.04 下的最小仿真链路。 | 帮助先验证规划/感知链路是否能跑通，再考虑真机联调。 |
| [`scripts/sim/install_ubuntu20_deps.sh`](scripts/sim/install_ubuntu20_deps.sh) | 安装 ROS Noetic 和最小仿真所需依赖。 | 仿真准备脚本，不直接参与真机飞行。 |
| [`scripts/sim/create_minimal_ws.sh`](scripts/sim/create_minimal_ws.sh) | 创建仅包含仿真所需包的最小工作区，并用软链接引入源码。 | 用于隔离 PX4、RealSense、Livox 等真机依赖，先单独验证规划链路。 |
| [`scripts/sim/build_minimal_ws.sh`](scripts/sim/build_minimal_ws.sh) | 构建最小仿真工作区。 | 仿真准备脚本，不直接参与真机飞行。 |
| [`scripts/sim/run_single_sim.sh`](scripts/sim/run_single_sim.sh) | 启动最小仿真入口。 | 实际执行 `roslaunch ego_planner single_run_in_sim.launch`，用于跑通单机规划仿真。 |
| [`scripts/sim/check_single_sim_topics.sh`](scripts/sim/check_single_sim_topics.sh) | 检查关键仿真话题。 | 用于确认点云、里程计、轨迹话题是否正常发布。 |
| [`scripts/sim/publish_goal.sh`](scripts/sim/publish_goal.sh) | 从命令行向 `/move_base_simple/goal` 发布 3D 目标点。 | 仿真下替代 RViz 手点目标点，便于快速回归测试。 |
| [`scripts/sim/find_gazebo_assets.sh`](scripts/sim/find_gazebo_assets.sh) | 搜索仓库里是否存在 Gazebo world/model/launch 资产。 | 仅用于摸清仓库是否额外带了 Gazebo 资源，不参与飞行逻辑。 |
| [`src/planner/plan_manage/config/competition_mission.yaml`](src/planner/plan_manage/config/competition_mission.yaml) | 定义竞赛任务顺序、话题、起降参数、区域③/④几何参数。 | 真机竞赛任务的核心配置文件；决定默认任务节奏、动态障碍参数、拱门参数、起降策略。 |
| [`src/planner/plan_manage/include/plan_manage/competition_mission_common.h`](src/planner/plan_manage/include/plan_manage/competition_mission_common.h) | 定义任务阶段、状态码、Block3/Block4 配置、门参数等共享结构体。 | 为竞赛任务管理器和区域专属节点提供统一接口和数据结构。 |
| [`src/planner/plan_manage/src/competition_mission_common.cpp`](src/planner/plan_manage/src/competition_mission_common.cpp) | 负责从 ROS 参数加载任务配置，并提供几何辅助函数。 | 真机启动竞赛任务时，负责把 `competition_mission.yaml` 解析成可执行任务；也负责穿越门、动态障碍相关几何计算。 |
| [`src/planner/plan_manage/src/competition_mission_manager.cpp`](src/planner/plan_manage/src/competition_mission_manager.cpp) | 竞赛总任务管理器。 | 负责自动起飞、任务阶段串联、给原始规划器下发目标点、激活区域③/④专属节点、任务结束后请求自动降落。 |
| [`src/planner/plan_manage/src/block3_task_manager.cpp`](src/planner/plan_manage/src/block3_task_manager.cpp) | 区域③移动障碍物区专属状态机。 | 负责观察动态障碍、等待通行时机、决定何时进入通道、何时提交穿越、何时触发紧急悬停。 |
| [`src/planner/plan_manage/src/block4_gate_manager.cpp`](src/planner/plan_manage/src/block4_gate_manager.cpp) | 区域④拱形障碍区专属状态机。 | 负责按门中心与法向生成门前/门后过渡点，完成对齐、穿门、恢复三个阶段。 |
| [`src/planner/plan_manage/launch/competition_run_in_exp.launch`](src/planner/plan_manage/launch/competition_run_in_exp.launch) | 真机竞赛入口。 | 在原始真机规划链路之上，额外启动 `competition_mission_manager`、`block3_task_manager`、`block4_gate_manager`。它是竞赛版真机入口，不是当前推荐的仿真入口。 |

### 2.2 改动过的原始文件

| 文件 | 改动点 | 作用 |
| --- | --- | --- |
| [`README.md`](README.md) | 从原始课程式说明重写为仓库说明书。 | 现在重点说明新增文件、比赛区域映射、真机链路和仿真入口。 |
| [`src/planner/plan_manage/launch/advanced_param_exp.xml`](src/planner/plan_manage/launch/advanced_param_exp.xml) | 新增 `goal_topic`、`grid_resolution`、`obstacles_inflation`、`control_points_distance` 等参数。 | 让竞赛任务管理器可以把目标点发给原始 `ego_planner`，同时允许按赛道特性调整地图分辨率、膨胀半径和控制点距离。 |
| [`src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h`](src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h) | 新增 `goal_topic_` 成员。 | 为原始规划器接管自定义任务目标话题做准备。 |
| [`src/planner/plan_manage/src/ego_replan_fsm.cpp`](src/planner/plan_manage/src/ego_replan_fsm.cpp) | 改成支持自定义 `fsm/goal_topic`，支持 3D 目标点，重复目标不重复重规划。 | 真机里它仍然是核心轨迹规划器，只是目标来源从固定 `/move_base_simple/goal` 改成了可被任务管理器接管的目标话题。 |
| [`src/planner/plan_manage/CMakeLists.txt`](src/planner/plan_manage/CMakeLists.txt) | 新增 `competition_mission_common` 库和 `competition_mission_manager`、`block3_task_manager`、`block4_gate_manager` 三个可执行节点。 | 把新增竞赛逻辑真正编译进 `ego_planner` 包。 |
| [`src/planner/plan_manage/package.xml`](src/planner/plan_manage/package.xml) | 补充 `geometry_msgs`、`nav_msgs`、`quadrotor_msgs` 等依赖。 | 让新增节点在真机里能够正常使用里程计、目标点、起降消息。 |
| [`src/uav_simulator/local_sensing/package.xml`](src/uav_simulator/local_sensing/package.xml) | 调整依赖，去掉一部分真机/历史包依赖，保留最小仿真需要的消息与点云依赖。 | 便于在 Ubuntu 20.04 最小仿真工作区里单独编译 `local_sensing_node`。 |

## 3. 新增代码和比赛区域的对应关系

本节按你提供的调整后规则说明：

- 比赛场地按 `12m x 8m` 理解；
- 精确降落按机体几何中心与降落区中心的水平距离判定；
- 当前默认任务配置里的 `x=3 -> x=8 -> x=12` 更接近调整后的场地节奏，而不是原始 `8m x 8m` 口径；
- 现场得分环中心和降落点坐标赛前给出，因此 [`competition_mission.yaml`](src/planner/plan_manage/config/competition_mission.yaml) 本质上还是一个需要按现场标定的模板。

### 起飞与降落

区域目标：

- 自主起飞；
- 依次完成赛道任务后进入降落区；
- 在降落区完成自动降落。

负责代码：

- [`src/planner/plan_manage/src/competition_mission_manager.cpp`](src/planner/plan_manage/src/competition_mission_manager.cpp)
- [`src/planner/plan_manage/config/competition_mission.yaml`](src/planner/plan_manage/config/competition_mission.yaml)
- [`src/realflight_modules/px4ctrl/launch/run_ctrl.launch`](src/realflight_modules/px4ctrl/launch/run_ctrl.launch)

真机中的实际作用：

- `competition_mission_manager` 在收到里程计后，按配置决定是否自动发布起飞请求到 `/px4ctrl/takeoff_land`。
- 它会等待飞行器达到 `takeoff_ready_height`，然后进入任务阶段。
- 任务全部完成后，它会先让飞行器到 `hover_land` 点短暂停留，再发布降落请求。
- `competition_mission.yaml` 中的 `auto_takeoff`、`auto_land`、`takeoff_wait_timeout`、`hover_land_hold_time`、`land_handoff_delay` 等参数，决定了起降阶段的自动化方式。

当前实现边界：

- 代码负责“把飞机带到降落目标点附近并请求自动降落”，但不会自动根据评分圆心做最后一层高精度校正。
- 调整后的评分规则是裁判侧判定标准，代码侧仍需要根据现场坐标和机体控制效果继续调参。

### 区域① 结构化障碍区

区域目标：

- 穿越随机分布的长方体障碍物。

负责代码：

- [`src/planner/plan_manage/src/ego_replan_fsm.cpp`](src/planner/plan_manage/src/ego_replan_fsm.cpp)
- [`src/planner/plan_manage/src/planner_manager.cpp`](src/planner/plan_manage/src/planner_manager.cpp)
- [`src/planner/plan_manage/launch/advanced_param_exp.xml`](src/planner/plan_manage/launch/advanced_param_exp.xml)
- 原始 `grid_map` / `local_sensing` 感知建图链路

真机中的实际作用：

- 当前没有为区域①单独新增任务管理器。
- 区域①主要依赖原始 `ego_planner` 的通用能力：接收当前阶段目标点，建立局部占据地图，然后搜索并优化避障轨迹。
- `planner_manager.cpp` 负责搜索与 B-spline 轨迹优化，`ego_replan_fsm.cpp` 负责在飞行过程中不断重规划。
- 在真机模式下，`advanced_param_exp.xml` 会把真实的里程计、深度/点云话题接入原始规划器。

当前实现边界：

- 区域①没有“专属状态机”。
- 当前实现更像是“给原始规划器一个阶段目标，让它自己完成结构化障碍穿越”，效果依赖感知质量、地图分辨率、膨胀半径和控制器调参。

### 区域② 密集树林区 + 风扰

区域目标：

- 穿越密集非结构化障碍；
- 在风扰存在的情况下保持稳定飞行。

负责代码：

- [`src/planner/plan_manage/src/ego_replan_fsm.cpp`](src/planner/plan_manage/src/ego_replan_fsm.cpp)
- [`src/planner/plan_manage/src/planner_manager.cpp`](src/planner/plan_manage/src/planner_manager.cpp)
- [`src/realflight_modules/px4ctrl/launch/run_ctrl.launch`](src/realflight_modules/px4ctrl/launch/run_ctrl.launch)
- [`src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml`](src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml)
- [`src/realflight_modules/mid360_fastlio/src/FAST_LIO/launch/mapping_mid360.launch`](src/realflight_modules/mid360_fastlio/src/FAST_LIO/launch/mapping_mid360.launch)

真机中的实际作用：

- 当前也没有为区域②单独新增任务管理器。
- 区域②仍主要依赖原始规划器、定位链路和控制链路协同完成：`FAST-LIO` 提供 `/Odom_high_freq`，`ego_planner` 负责密集障碍避障，`px4ctrl` 负责轨迹跟踪与姿态稳定。
- 在代码层面，风扰并没有对应一个独立“风扰处理节点”；更大程度上依赖控制器参数、定位稳定性和规划保守性。

当前实现边界：

- README 需要如实理解为：区域②没有“专属节点”，它更多依赖原始系统的稠密障碍避障和真机控制稳定性。
- 如果现场风扰更强，通常需要继续调 `px4ctrl` 参数以及 `advanced_param_exp.xml` 中的规划安全边界。

### 区域③ 移动障碍物区

区域目标：

- 在存在移动障碍物的情况下，自主等待合适时机并安全穿越。

负责代码：

- [`src/planner/plan_manage/src/block3_task_manager.cpp`](src/planner/plan_manage/src/block3_task_manager.cpp)
- [`src/planner/plan_manage/src/competition_mission_manager.cpp`](src/planner/plan_manage/src/competition_mission_manager.cpp)
- [`src/planner/plan_manage/src/competition_mission_common.cpp`](src/planner/plan_manage/src/competition_mission_common.cpp)
- [`src/planner/plan_manage/include/plan_manage/competition_mission_common.h`](src/planner/plan_manage/include/plan_manage/competition_mission_common.h)
- [`src/planner/plan_manage/config/competition_mission.yaml`](src/planner/plan_manage/config/competition_mission.yaml)

真机中的实际作用：

- `competition_mission_manager` 在任务序列进入 `block3` 阶段后，通过 `block3_activate_topic` 激活 `block3_task_manager`。
- `block3_task_manager` 会监听 `dynamic_topics`，默认是 `/dynamic/pose_0`，要求外部感知侧给出动态障碍位姿。
- 进入区域③后，它会先飞到 `wait_point`，在等待点观察动态障碍。
- 然后它会根据 `dynamic_object_radius`、`corridor_width`、`prediction_horizon`、`safety_margin`、`min_gap_time` 评估通行走廊是否安全。
- 当连续多次观测都认为走廊可通行时，它会先推进到 `cross_entry`。
- 到达 `cross_entry` 附近后，它会提交穿越动作，把目标点改成 `cross_exit`。
- 在提交穿越后，如果判断当前走廊不再安全，会触发 `STATUS_EMERGENCY`，并把目标点收回到当前悬停位置，进入紧急保持。
- 成功到达 `cross_exit` 后，它会上报 `STATUS_DONE`，由 `competition_mission_manager` 接回下一阶段。

当前实现边界：

- 这部分代码解决的是“任务层何时穿越”的问题，不负责从传感器原始数据中检测和跟踪动态障碍。
- 因此在真实比赛前，仍需要保证感知侧能稳定发布动态障碍位姿话题。

### 区域④ 拱形障碍物区

区域目标：

- 在狭窄的拱形通道前完成对齐，并稳定穿门。

负责代码：

- [`src/planner/plan_manage/src/block4_gate_manager.cpp`](src/planner/plan_manage/src/block4_gate_manager.cpp)
- [`src/planner/plan_manage/src/competition_mission_manager.cpp`](src/planner/plan_manage/src/competition_mission_manager.cpp)
- [`src/planner/plan_manage/src/competition_mission_common.cpp`](src/planner/plan_manage/src/competition_mission_common.cpp)
- [`src/planner/plan_manage/include/plan_manage/competition_mission_common.h`](src/planner/plan_manage/include/plan_manage/competition_mission_common.h)
- [`src/planner/plan_manage/config/competition_mission.yaml`](src/planner/plan_manage/config/competition_mission.yaml)

真机中的实际作用：

- `competition_mission_manager` 在任务序列进入 `block4` 阶段后，会通过 `block4_start_topic` 下发 `gate_index`。
- `block4_gate_manager` 根据该门的 `center`、`normal`、`pre_distance`、`post_distance` 自动生成 `pre_gate` 和 `post_gate`。
- 它会先把目标点设为门前等待点 `pre_gate`。
- 到达门前后，它会检查位置、速度、朝向是否满足对齐条件，阈值由 `align_position_tolerance`、`align_speed_tolerance`、`align_yaw_tolerance_deg` 给出。
- 对齐成功后，它会把目标点切到门中心，提交穿门动作。
- 穿过门中心后，再把目标点切到 `post_gate`，完成门后恢复。
- 成功到达 `post_gate` 后，它会上报 `STATUS_DONE`，由总任务管理器继续后续阶段。

当前实现边界：

- 当前实现默认门中心、法向量和缓冲距离都来自 YAML 人工配置，而不是在线识别门框后自动生成。
- 因此在比赛现场，区域④的参数同样需要根据真实门位置和机体控制表现做标定。

### 当前实现的显式覆盖范围

- 显式新增状态机覆盖的是区域③移动障碍物区和区域④拱形障碍物区。
- 区域①结构化障碍区和区域②密集树林区更多依赖原始 `REAL_DRONE_400` 的通用感知建图、避障规划和飞控能力。
- 因此不要把当前实现理解成“四个区域都有独立任务节点”；当前更准确的说法是“前半程靠原始系统通用能力，后半程对区域③/④增加了专门任务管理逻辑”。

## 4. 原始 REAL_DRONE_400 关键文件说明

### 4.1 规划主链路

| 文件 | 作用 |
| --- | --- |
| [`src/planner/plan_manage/launch/single_run_in_exp.launch`](src/planner/plan_manage/launch/single_run_in_exp.launch) | 原始真机规划入口。只启动原始实验版 `ego_planner` 和 `traj_server`，不包含区域③/④任务管理器。 |
| [`src/planner/plan_manage/launch/single_run_in_sim.launch`](src/planner/plan_manage/launch/single_run_in_sim.launch) | 原始单机最小仿真入口。当前 `scripts/sim/run_single_sim.sh` 实际启动的就是它。 |
| [`src/planner/plan_manage/launch/advanced_param.xml`](src/planner/plan_manage/launch/advanced_param.xml) | 原始仿真参数入口，负责把仿真里程计、点云、深度和轨迹参数接给 `ego_planner`。 |
| [`src/planner/plan_manage/launch/advanced_param_exp.xml`](src/planner/plan_manage/launch/advanced_param_exp.xml) | 原始真机参数入口，当前竞赛入口仍然复用它，只是额外开放了 `goal_topic` 等参数。 |
| [`src/planner/plan_manage/src/ego_replan_fsm.cpp`](src/planner/plan_manage/src/ego_replan_fsm.cpp) | 原始 EGO-Planner 顶层状态机。负责接收目标点、管理重规划时机、发布轨迹。 |
| [`src/planner/plan_manage/src/planner_manager.cpp`](src/planner/plan_manage/src/planner_manager.cpp) | 原始规划核心，负责搜索、轨迹初始化、B-spline 优化。 |
| [`src/planner/plan_manage/src/traj_server.cpp`](src/planner/plan_manage/src/traj_server.cpp) | 把规划出的 B-spline 轨迹转成 `/position_cmd`，供控制器跟踪。 |

### 4.2 仿真支撑

| 文件 | 作用 |
| --- | --- |
| [`src/planner/plan_manage/launch/simulator.xml`](src/planner/plan_manage/launch/simulator.xml) | 原始仿真组件装配文件，负责把假机体、局部感知、地图生成器等串起来。 |
| [`src/uav_simulator/fake_drone/src/poscmd_2_odom.cpp`](src/uav_simulator/fake_drone/src/poscmd_2_odom.cpp) | 把规划器输出的位置指令直接转换成仿真里程计，作为“假机体”运动模型。 |
| [`src/uav_simulator/local_sensing/src/pcl_render_node.cpp`](src/uav_simulator/local_sensing/src/pcl_render_node.cpp) | 根据当前位姿从地图中渲染局部点云/深度，给规划器提供局部感知输入。 |
| [`src/uav_simulator/map_generator/src/random_forest_sensing.cpp`](src/uav_simulator/map_generator/src/random_forest_sensing.cpp) | 生成随机障碍地图，供单机仿真测试避障规划。 |

### 4.3 真机定位与控制

| 文件 | 作用 |
| --- | --- |
| [`src/realflight_modules/px4ctrl/launch/run_ctrl.launch`](src/realflight_modules/px4ctrl/launch/run_ctrl.launch) | 启动 PX4 控制节点，把 `/position_cmd` 变成真实飞行控制输出。 |
| [`src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml`](src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml) | PX4 控制器参数，包括质量、悬停油门、PID 增益、自动起降配置等。 |
| [`src/realflight_modules/mid360_fastlio/src/FAST_LIO/launch/mapping_mid360.launch`](src/realflight_modules/mid360_fastlio/src/FAST_LIO/launch/mapping_mid360.launch) | MID360 的 FAST-LIO 启动入口，为真机规划链路提供 `/Odom_high_freq`。 |

### 4.4 原始快捷脚本

| 文件 | 作用 |
| --- | --- |
| [`home_shfiles/start_sensor.sh`](home_shfiles/start_sensor.sh) | 原始真机启动脚本，依次拉起 MAVROS、RealSense、Livox 驱动。 |
| [`home_shfiles/start_mapping.sh`](home_shfiles/start_mapping.sh) | 启动 FAST-LIO。 |
| [`home_shfiles/start_planner.sh`](home_shfiles/start_planner.sh) | 启动原始 `single_run_in_exp.launch`，不包含新增竞赛任务管理器。 |
| [`home_shfiles/start_run_ctrl.sh`](home_shfiles/start_run_ctrl.sh) | 启动 `px4ctrl run_ctrl.launch`。 |
| [`home_shfiles/start_rviz.sh`](home_shfiles/start_rviz.sh) | 启动 RViz。 |
| [`home_shfiles/start_takeoff.sh`](home_shfiles/start_takeoff.sh) | 手动向 `/px4ctrl/takeoff_land` 发布起飞命令。 |
| [`home_shfiles/start_land.sh`](home_shfiles/start_land.sh) | 手动向 `/px4ctrl/takeoff_land` 发布降落命令。 |

## 5. 真机飞行链路

如果你要运行“原始真机链路”，仓库里原本的快捷脚本顺序大致是：

1. 运行 [`home_shfiles/start_sensor.sh`](home_shfiles/start_sensor.sh)，启动 MAVROS、RealSense、Livox 驱动。
2. 运行 [`home_shfiles/start_mapping.sh`](home_shfiles/start_mapping.sh)，启动 FAST-LIO，发布 `/Odom_high_freq`。
3. 运行 [`home_shfiles/start_run_ctrl.sh`](home_shfiles/start_run_ctrl.sh)，启动 `px4ctrl`。
4. 运行 [`home_shfiles/start_planner.sh`](home_shfiles/start_planner.sh)，启动原始 `single_run_in_exp.launch`。
5. 用 [`home_shfiles/start_takeoff.sh`](home_shfiles/start_takeoff.sh) / [`home_shfiles/start_land.sh`](home_shfiles/start_land.sh) 手动起降。

如果你要运行“当前新增的竞赛任务链路”，重点区别在于第 4 步不再使用 `start_planner.sh`，而是应直接启动：

```bash
source /path/to/REAL_DRONE_400/devel/setup.bash
roslaunch ego_planner competition_run_in_exp.launch
```

这条竞赛链路的数据流是：

1. 传感器和定位侧启动后，`FAST-LIO` 提供 `/Odom_high_freq`。
2. [`competition_run_in_exp.launch`](src/planner/plan_manage/launch/competition_run_in_exp.launch) 同时启动原始 `ego_planner_node`、`traj_server` 以及新增的 `competition_mission_manager`、`block3_task_manager`、`block4_gate_manager`。
3. `competition_mission_manager` 负责自动起飞、任务阶段切换、向 `/competition/mission_goal` 发布当前阶段目标点，以及在区域③/④激活专属节点。
4. `block3_task_manager` 与 `block4_gate_manager` 在被激活后，会暂时接管“当前阶段应该飞向哪里”的决策，并把目标点继续发给原始规划器。
5. `ego_replan_fsm.cpp` 和 `planner_manager.cpp` 根据当前目标点、里程计和点云，生成避障 B-spline 轨迹。
6. `traj_server.cpp` 把 B-spline 轨迹转换成 `/position_cmd`。
7. `px4ctrl` 订阅 `/position_cmd`，执行真实飞行控制。

需要注意：

- `home_shfiles/start_planner.sh` 仍然是原始入口，并不会自动启用区域③/④逻辑。
- 如果 `competition_mission.yaml` 中 `auto_takeoff`、`auto_land` 都为 `true`，那么起降可以由 `competition_mission_manager` 自动请求；原始 `start_takeoff.sh` 和 `start_land.sh` 更多适合调试或手动链路。

## 6. 仿真怎么跑

### 6.1 仿真文件位置

- 仿真辅助脚本在 [`scripts/sim/`](scripts/sim/)。
- 当前实际规划入口是 [`src/planner/plan_manage/launch/single_run_in_sim.launch`](src/planner/plan_manage/launch/single_run_in_sim.launch)。
- 当前推荐仿真链路不是 `competition_run_in_exp.launch`，而是最小单机规划仿真链路。

### 6.2 仿真依赖

- Ubuntu 20.04
- ROS Noetic

### 6.3 当前仿真链路包含什么

当前最小仿真链路主要由以下组件组成：

- `ego_planner`
- `poscmd_2_odom`
- `local_sensing_node`
- `map_generator`

它的目标是先验证规划和局部感知链路能否跑通，而不是直接复现完整的国赛 Gazebo 场地。

### 6.4 仿真启动命令

```bash
cd /path/to/REAL_DRONE_400
bash scripts/sim/install_ubuntu20_deps.sh
bash scripts/sim/create_minimal_ws.sh ~/ws_guosai_sim
bash scripts/sim/build_minimal_ws.sh ~/ws_guosai_sim
bash scripts/sim/run_single_sim.sh ~/ws_guosai_sim
```

其中：

- [`install_ubuntu20_deps.sh`](scripts/sim/install_ubuntu20_deps.sh) 安装 ROS Noetic 和最小依赖；
- [`create_minimal_ws.sh`](scripts/sim/create_minimal_ws.sh) 创建最小工作区；
- [`build_minimal_ws.sh`](scripts/sim/build_minimal_ws.sh) 编译最小工作区；
- [`run_single_sim.sh`](scripts/sim/run_single_sim.sh) 实际执行 `roslaunch ego_planner single_run_in_sim.launch`。

### 6.5 仿真检查命令

新开一个终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/ws_guosai_sim/devel/setup.bash
bash /path/to/REAL_DRONE_400/scripts/sim/check_single_sim_topics.sh ~/ws_guosai_sim
```

这个脚本会重点检查：

- `/map_generator/global_cloud`
- `/drone_0_visual_slam/odom`
- `/drone_0_pcl_render_node/cloud`
- `/drone_0_planning/bspline`

### 6.6 仿真发目标点命令

```bash
cd /path/to/REAL_DRONE_400
bash scripts/sim/publish_goal.sh ~/ws_guosai_sim 5.0 0.0 1.0
```

这个命令会向默认话题 `/move_base_simple/goal` 发送一个三维目标点。你也可以在 RViz 中使用 `2D Nav Goal` 手动下点。

### 6.7 Gazebo 相关说明

- [`scripts/sim/find_gazebo_assets.sh`](scripts/sim/find_gazebo_assets.sh) 只负责搜索仓库里是否存在额外的 Gazebo world/model/launch 文件。
- 它的作用是帮助你判断“仓库里有没有别的 Gazebo 入口”，不是说当前仓库已经完整提供了国赛 Gazebo 比赛场地。
- 当前默认仿真链路仍然是最小规划仿真链路，而不是完整 Gazebo 国赛场地复现。
