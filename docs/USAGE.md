# 离线模式说明

离线运行链路：

```text
rosbag IMU        -> ImageProjection::imuHandler()
rosbag IMU        -> IMUPreintegration::imuHandler()
rosbag PointCloud -> ImageProjection::cloudHandler()
ImageProjection   -> FeatureExtraction::laserCloudInfoHandler()
FeatureExtraction -> mapOptimization::laserCloudInfoHandler()
mapOptimization   -> IMUPreintegration::odometryHandler()
IMUPreintegration -> ImageProjection::odometryHandler()
```

回环逻辑：

```text
RunOfflineLoopClosureOnce()
  -> performLoopClosure()
     -> detectLoopClosureExternal()
     -> detectLoopClosureDistance()
```

所以默认距离回环会被离线主动触发；`loopInfoHandler()` 只是外部回环消息入口。
