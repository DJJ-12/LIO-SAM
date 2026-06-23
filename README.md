# LIO-SAM 离线版工程包

这个压缩包已经整理成可直接放入 ROS2 工作空间 `src/` 目录下的 package 结构。顶层目录名是 `LIO-SAM`，ROS 包名仍然是 `lio_sam`。

## 目录结构

```text
LIO-SAM/
├── CMakeLists.txt
├── package.xml
├── msg/CloudInfo.msg
├── srv/SaveMap.srv
├── include/
├── src/
├── launch/
├── config/
└── docs/
```

## 使用方式

请先删除你之前错误放进去的补丁目录：

```bash
cd ~/ws_lio-sam/src
rm -rf lio_sam_offline_patch
```

然后把本压缩包解压到 `src` 下：

```bash
cd ~/ws_lio-sam/src
unzip /path/to/LIO-SAM.zip
```

此时应该是：

```text
~/ws_lio-sam/src/LIO-SAM/package.xml
~/ws_lio-sam/src/LIO-SAM/CMakeLists.txt
```

编译：

```bash
cd ~/ws_lio-sam
source /opt/ros/humble/setup.bash
colcon build --packages-select lio_sam --symlink-install
source install/setup.bash
```

## 离线运行

```bash
ros2 launch lio_sam offline.launch.py \
  input_bag:=/path/to/rosbag2_dir \
  params_file:=/path/to/params.yaml \
  save_directory:=/tmp/lio_sam_offline_map
```

或者：

```bash
ros2 run lio_sam lio_sam_run_offline \
  --input_bag /path/to/rosbag2_dir \
  --params_file /path/to/params.yaml \
  --save_directory /tmp/lio_sam_offline_map
```

## 说明

1. 这版保留原来的在线四节点可执行程序，同时新增 `lio_sam_run_offline`。
2. 离线模式不依赖四个 ROS topic 节点互相通信，而是在一个进程中按 rosbag 时间顺序直接调用原算法回调。
3. 默认回环不是依赖 `loopInfoHandler()`，而是在离线流程中周期调用 `RunOfflineLoopClosureOnce()`，内部执行 `performLoopClosure()`。
4. `loopInfoHandler()` 仍保留，只用于 bag 中存在 `/lio_loop/loop_closure_detection` 或未来外部回环检测节点发布消息的情况。
