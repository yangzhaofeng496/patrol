# patrol

## 作用

执行机器人巡检流程：读取示教 CSV，控制底盘导航和机械臂运动，调用检测服务并上报巡检结果。

## 运行

配置文件：`config/patrol.yaml`，主要配置运行模式、关节/笛卡尔模式、CSV 路径、日志路径和记录路径。

运行巡检节点：

```bash
roslaunch patrol patrol.launch
```

单独运行机械臂点位发送节点：

```bash
roslaunch patrol arm_point_sender.launch
```

## 接口

订阅/发布：

- 订阅 `/odom` (`nav_msgs/Odometry`)：底盘运动状态。
- 发布 `/detect/context` (`std_msgs/String`)：当前检测上下文。

服务：

- `/robot/start` (`moma_service/RobotStart`)：接收巡检任务。
- `/detectPic` (`moma_service/DetectPic`)：请求图像检测。
- `/robot/report/data` (`moma_service/RobotReportData`)：上报检测结果。
- `/robot/complete` (`moma_service/RobotCompleted`)：上报任务完成。
- `send_arm_point` (`patrol/SendArmPoint`)：单独发送指定机械臂点位。

节点还会调用底盘 `move_base`、Dobot 机械臂控制服务和可选的笛卡尔运动服务。
