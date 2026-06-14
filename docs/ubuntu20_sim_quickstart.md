# Ubuntu 20.04 仿真快速开始

当前仓库现成可跑通的仿真链路是：

- `ego_planner`
- `poscmd_2_odom`
- `local_sensing_node`
- `map_generator`

这条链路运行在 ROS Noetic 下，当前并不是完整的 Gazebo world 或 `gazebo_ros` 启动链。

## 1. 安装 Ubuntu 20.04 依赖

```bash
cd /path/to/REAL_DRONE_400
bash scripts/sim/install_ubuntu20_deps.sh
```

## 2. 创建最小仿真工作区

完整仓库里还包含 PX4、MAVROS、RealSense、Livox 等实机相关包。为了先把仿真稳定跑起来，建议先创建一个只链接仿真所需包的最小工作区。

```bash
cd /path/to/REAL_DRONE_400
bash scripts/sim/create_minimal_ws.sh ~/ws_guosai_sim
```

## 3. 编译最小工作区

```bash
cd /path/to/REAL_DRONE_400
bash scripts/sim/build_minimal_ws.sh ~/ws_guosai_sim
```

## 4. 启动现有仿真入口

```bash
cd /path/to/REAL_DRONE_400
bash scripts/sim/run_single_sim.sh ~/ws_guosai_sim
```

当前实际使用的入口是：

- `src/planner/plan_manage/launch/single_run_in_sim.launch`

## 5. 检查关键话题

新开一个终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/ws_guosai_sim/devel/setup.bash
bash /path/to/REAL_DRONE_400/scripts/sim/check_single_sim_topics.sh ~/ws_guosai_sim
```

重点看这些话题是否有数据：

- `/map_generator/global_cloud`
- `/drone_0_visual_slam/odom`
- `/drone_0_pcl_render_node/cloud`
- `/drone_0_planning/bspline`

## 6. 发送目标点

### 方式 A：RViz

- `Fixed Frame` 设为 `world`
- 使用 `2D Nav Goal`
- 先点一个离起点不太远的前方目标

### 方式 B：命令行

```bash
cd /path/to/REAL_DRONE_400
bash scripts/sim/publish_goal.sh ~/ws_guosai_sim 5.0 0.0 1.0
```

## 7. 你应该看到什么

- RViz 正常打开
- 地图点云出现
- `/drone_0_visual_slam/odom` 持续发布
- 发目标后开始出现轨迹
- 小飞机 marker 开始移动

## 8. 检查是否真的有 Gazebo 资产

如果你自己的 `gezogo-guosai` 仓库额外加了 Gazebo world、model 或 launch，可以运行：

```bash
cd /path/to/your/repo
bash scripts/sim/find_gazebo_assets.sh src
```

如果命令找到了 `.world`、`.sdf`、`gazebo_ros`、`gzserver` 等入口，再根据搜索结果里的 launch 文件继续启动对应 Gazebo 仿真。

## 说明

- `competition_run_in_exp.launch` 主要面向比赛逻辑和实机话题，不适合作为第一次仿真入口。
- `local_sensing_node` 当前默认走 CPU 点云渲染路径；如果你后面要切回 CUDA 路径，还需要额外安装对应依赖。
