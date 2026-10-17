#!/usr/bin/env python3
import os
import datetime
import math
import time
import csv

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from moma_service.msg import ResultCount
from moma_service.srv import (
    # MoveJoint,
    DetectPic,
    RobotStart,
    RobotReportData,
    RobotCompleted,
)

from moma_service.srv import ServoJ
from moma_service.srv import EnableRobot
from moma_service.srv import DisableRobot

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_matrix, inverse_matrix, quaternion_from_matrix

try:
    from .path_utils import resolve_ros_path
except ImportError:
    from path_utils import resolve_ros_path

class PatrolNode(object):
    def __init__(self):
        rospy.init_node('patrol_node')
        
        # 读取yaml中的配置信息
        self.mode = rospy.get_param('~mode', 'offline').lower()
        self.execute_type = rospy.get_param('~execute_type', 'joint').lower()
        if self.execute_type not in ('joint', 'cartesian'):
            raise ValueError('execute_type must be joint or cartesian')
        if self.execute_type == 'cartesian':
            self.reference_from_user = self.read_cartesian_transform('cart·esian_reference_from_user')
            self.tcp_from_tip = inverse_matrix(
                self.read_cartesian_transform('cartesian_tip_from_tcp'))
        self.log_path_prefix = resolve_ros_path(
            rospy.get_param('~log_path_prefix', '')
        )
        self.record_path_prefix = resolve_ros_path(
            rospy.get_param('~record_path_prefix', '')
        )
        self.min_base_move_distance = rospy.get_param('~min_base_move_distance', 0.05)
        self.img_dir_prefix = ''
        self.detect_context_pub = rospy.Publisher(
            '/detect/context', String, queue_size=1, latch=True
        )
        
        # 创建日志文件，格式如2025-10-02_14-52-44.log
        os.makedirs(self.log_path_prefix, exist_ok=True)  # 没有则创建
        now_str = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.log_file_path = os.path.join(self.log_path_prefix, f'{now_str}.log')
        self.logger = open(self.log_file_path, 'w')
        
        # imu变量
        self.odom_linear_x = 0.0
        self.odom_angular_z = 0.0


        # client子节点
        self.waypoints = []
        self.temp_waypoints_before = []
        self.temp_waypoints_after = []
        # self.publisher_base = None
        self.client_base = None
        self.client_arm_enable_robot = None
        self.client_arm = None
        self.client_img = None
        self.client_report_data = None
        self.client_complete = None
        self.odom_subscriber = None

        # 任务属性参数
        self.taskID = None
        self.trainTypeName = None
        self.trainNumber = None
        self.repairClassName = None
        self.verbType = None
        self.verbTypeStr = None
        self.startTimeStamp = None
        self.robotId = None
        self.workMode = None

        # 初始化任务统计量
        self.total_summary = ResultCount()
        self.count_verify = ResultCount()
        self.installation_position = ResultCount()
        self.physical_measurement = ResultCount()

        self.received_robot_start_request = False
        self.task_active = False

        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)

        # ONLINE
        if self.mode == 'online':
            self.server_robot_start = rospy.Service(
                '/robot/start', 
                RobotStart, 
                self.handle_robot_start_request
            )
        
        else:
            offline_cfg = rospy.get_param('~offline')
            self.taskID = offline_cfg.get('task_id', 100)
            self.trainTypeName = offline_cfg.get('train_type_name', 'HXD1C')
            self.startTimeStamp = offline_cfg.get('start_timestamp', '2025-12-26_19-09-00')
            self.img_dir_prefix = offline_cfg.get(
                'img_dir_prefix',
                os.path.join(self.record_path_prefix, self.trainTypeName, self.startTimeStamp)
            )
        
    def handle_robot_start_request(self, request):
        response = RobotStart._response_class()
        if self.task_active:
            response.code = '1'
            response.message = 'A patrol task is already pending or running.'
            rospy.logwarn('收到新的巡检任务，但当前任务尚未结束')
            return response

        try:
            self.task_active = True
            self.taskID = request.id
            self.trainTypeName = request.train_type_name
            self.trainNumber = request.train_number
            self.verbType = request.verb_type
            self.verbTypeStr = request.verb_type_str
            self.repairClassName = request.repair_class_name
            self.startTimeStamp = request.timestamp.replace(' ', '_').replace(':', '-')
            self.robotId = request.robot_id
            self.workMode = request.work_mode

            self.logger.write(f"taskID: {self.taskID}\n")
            self.logger.write(f"trainTypeName: {self.trainTypeName}\n")
            self.logger.write(f"trainNumber: {self.trainNumber}\n")
            self.logger.write(f"verbType: {self.verbType}\n")
            self.logger.write(f"verbTypeStr: {self.verbTypeStr}\n")
            self.logger.write(f"robotId: {self.robotId}\n")
            self.logger.write(f"repairClassName: {self.repairClassName}\n")
            self.logger.write(f"Robot Start Timestamp: {request.timestamp}\n")
            self.logger.flush()
            rospy.loginfo(f"Robot Start Timestamp: {request.timestamp}")

            self.received_robot_start_request = True
            response.code = '0'
        except Exception as e:
            self.task_active = False
            response.code = '1'
            response.message = str(e)
            self.logger.write(f'[error] {e}')
        finally:
            return response

    def spin_until_start(self):
        rospy.loginfo("Spin for Robot Start Command...")
        self.logger.write("Spin for Robot Start Command...\n")
        self.logger.flush()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.received_robot_start_request:
            rate.sleep()  # 非阻塞自旋
        rospy.loginfo("Robot Start Command Received!")
        self.logger.write("Robot Start Command Received!\n")
        self.logger.flush()

    def reset_task_state(self):
        self.waypoints = []
        self.temp_waypoints_before = []
        self.temp_waypoints_after = []
        self.total_summary = ResultCount()
        self.count_verify = ResultCount()
        self.installation_position = ResultCount()
        self.physical_measurement = ResultCount()

    def init_path_clients(self):
        _path_prefix = os.path.join(self.record_path_prefix, self.trainTypeName)
        if not self.img_dir_prefix:
            self.img_dir_prefix = os.path.join(_path_prefix, self.startTimeStamp)
        os.makedirs(self.img_dir_prefix, exist_ok=True)  # 没有则创建
        img_dir_result_prefix = os.path.join(self.img_dir_prefix, 'result')
        os.makedirs(img_dir_result_prefix, exist_ok=True)  # 没有则创建

        self.waypoints_file_name = (
            rospy.get_param('~waypoints_file_name', '') or f'{self.trainTypeName}.csv')
        waypoints_dir_path = resolve_ros_path(
            rospy.get_param('~waypoints_dir_path', '')
        )
        self.waypoints_file_path = os.path.join(
            waypoints_dir_path, self.waypoints_file_name
        )
        self.read_waypoints_from_csv()
        if not self.waypoints:
            self.logger.write("[error] Waypoints Imported Error!\n")
            self.logger.flush()
            rospy.logerr("Waypoints Imported Error!")
            raise ValueError("CSV 中没有可执行项点")

        # self.publisher_base = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)  # 这是原来的 ros1好像没这个 需要用 ROS 1 的 actionlib 和 move_base 相关代码替换
        self.client_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client_arm_enable_robot = rospy.ServiceProxy('/dobot_v4_bringup/srv/EnableRobot', EnableRobot)
        self.client_arm_disable_robot = rospy.ServiceProxy('/dobot_v4_bringup/srv/DisableRobot', DisableRobot)
        if self.execute_type == 'cartesian':
            # 仅笛卡尔模式需要 source 含此服务定义的 whole_body_ws 环境。
            from moveit_whole_body_ik.srv import MoveToPose
            self.arm_service_type = MoveToPose
            self.client_arm = rospy.ServiceProxy('/move_to_pose', MoveToPose)
        else:
            self.arm_service_type = ServoJ
            self.client_arm = rospy.ServiceProxy('/dobot_v4_bringup/srv/ServoJ', ServoJ)
        self.client_img = rospy.ServiceProxy('/detectPic', DetectPic)
        self.client_report_data = rospy.ServiceProxy('/robot/report/data', RobotReportData)
        self.client_complete = rospy.ServiceProxy('/robot/complete', RobotCompleted)
        
        # self.navigator.waitUntilNav2Active() 这是原来的 ros1好像没这个 需要用 ROS 1 的 actionlib 和 move_base 相关代码替换
        while not rospy.is_shutdown():
            try:
                self.client_arm_enable_robot.wait_for_service(timeout=1.0)
                self.client_arm_disable_robot.wait_for_service(timeout=1.0)
                self.client_arm.wait_for_service(timeout=1.0)
                break
            except rospy.ROSException:
                rospy.loginfo('等待机械臂服务端上线')
                self.logger.write('等待机械臂服务端上线\n')
                self.logger.flush()
        
        while not rospy.is_shutdown():
            try:
                self.client_img.wait_for_service(timeout=1.0)
                break
            except rospy.ROSException:
                rospy.loginfo('等待图像处理服务端上线')
                self.logger.write('等待图像处理服务端上线\n')
                self.logger.flush()

        req = EnableRobot._request_class()
        response = self.client_arm_enable_robot(req)
        rospy.loginfo('enable_robot response:' + str(response))
        self.logger.write('enable_robot response:' + str(response))
        self.logger.flush()

    def odom_callback(self, msg):
        # 读取msg并保存到self变量中
        self.odom_linear_x = msg.twist.twist.linear.x
        self.odom_angular_z = msg.twist.twist.angular.z

    def send_base_request(self, waypoint):
        # 构造目标点
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = waypoint['底盘x']
        goal_pose.pose.position.y = waypoint['底盘y']
        goal_pose.pose.orientation.z = math.sin(waypoint['底盘yaw']/2)
        goal_pose.pose.orientation.w = math.cos(waypoint['底盘yaw']/2)

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose



        # self.publisher_base.publish(goal_pose) 

        rospy.loginfo(f"发送导航目标: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}, yaw={waypoint['底盘yaw']}")
        self.logger.write(f"发送导航目标: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}, yaw={waypoint['底盘yaw']}\n")
        self.logger.flush()

        self.client_base.send_goal(goal)

        start_time = time.time()
        timeout = 600.0  # 可调整
        stuck_start = None
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            state = self.client_base.get_state()
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
                break
            if time.time() - start_time > timeout:
                rospy.logwarn('导航超时，取消目标')
                self.logger.write('导航超时，取消目标\n'); self.logger.flush()
                self.client_base.cancel_goal()
                break
            if abs(self.odom_linear_x) < 0.01 and  abs(self.odom_angular_z) < 0.1:
                if stuck_start is None:
                    stuck_start = time.time()
                elif time.time() - stuck_start > 7.0:
                    rospy.logwarn('导航疑似卡住，超过10s，取消目标')
                    self.logger.write('导航疑似卡住，超过10s，取消目标\n'); self.logger.flush()
                    self.client_base.cancel_goal()
                    break
            else:
                stuck_start = None
            rate.sleep()

        final_state = self.client_base.get_state()
        if final_state == GoalStatus.SUCCEEDED:
            msg = f'项点{waypoint["项点编号"]}导航成功'
        else:
            msg = f'项点{waypoint["项点编号"]}导航失败, 状态码={final_state}'
        rospy.loginfo(msg); self.logger.write(msg + '\n'); self.logger.flush()
        if final_state != GoalStatus.SUCCEEDED:
            raise RuntimeError(msg)

    def send_arm_request(self, waypoint):
        request = self.arm_service_type._request_class()
        if self.execute_type == 'cartesian':
            (request.x, request.y, request.z, request.qx,
             request.qy, request.qz, request.qw) = waypoint['cartesian_target']
        else:
            request.a = waypoint['关节1']
            request.b = waypoint['关节2']
            request.c = waypoint['关节3']
            request.d = waypoint['关节4']
            request.e = waypoint['关节5']
            request.f = waypoint['关节6']
            request.paramValue = ['t=3']  # t 单位为秒, aheadtime, gain
        try:
            response = self.client_arm(request)
            msg = f"Arm response: {response}"
            rospy.loginfo(msg)
            self.logger.write(msg + '\n')
            self.logger.flush()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            raise
        if self.execute_type == 'cartesian':
            # 根据 MoveIt 服务返回的 success/message 判断规划及执行是否成功。
            if not response.success:
                raise RuntimeError(f"项点{waypoint['项点编号']}机械臂运动失败: {response.message}")
            # /move_to_pose 同步返回执行结果，无需沿用 ServoJ 的固定延时。
        else:
            rospy.sleep(5.5)  # 阻塞5s

    def send_detect_pic_request(self, waypoint):
        request = DetectPic._request_class()
        request.save_path_prefix = os.path.join(self.img_dir_prefix, waypoint['项点编号'])
        request.detect_type = waypoint['检测类型']
        context = ';'.join([
            f'train_type_name={self.trainTypeName}',
            f'point_name={waypoint["项点名称"].strip()}',
            f'point_id={waypoint["项点编号"]}',
        ])
        self.detect_context_pub.publish(String(data=context))
        rospy.sleep(0.05)

        try:
            response = self.client_img(request)
            msg = f"point {waypoint['项点编号']} Img response message: {response.message}"
            rospy.loginfo(msg)
            self.logger.write(msg + '\n')
            self.logger.flush()
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def send_http_report_data(self, waypoint, detect_response):
        request = RobotReportData._request_class()
        img_path_prefix = os.path.join(self.img_dir_prefix, waypoint['项点编号'])
        request.raw_file_path = img_path_prefix + '_color.png'
        request.id = self.taskID
        request.point_id = waypoint['项点编号']
        request.analyse_result = detect_response.analyse_result if detect_response else ""
        request.point_name = waypoint['项点名称']  # TODO 根据不同机车车型制作项点清单，从配置文件读取
        request.point_type = waypoint['项点类型']
        request.message = detect_response.message if detect_response else ""
        request.timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        request.robot_id = self.robotId
        request.object_count = detect_response.object_count if detect_response else 0
        request.object_list = detect_response.object_list if detect_response else []
        
        msg = f"http_report_data request: {request}"
        rospy.loginfo(msg)
        self.logger.write(msg + '\n')
        try:
            response = self.client_report_data(request)
            msg = f"http_report_data response: {response}"
            rospy.loginfo(msg)
            self.logger.write(msg + '\n')
            self.logger.flush()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def send_robot_complete(self):
        request = RobotCompleted._request_class()
        request.task_id = self.taskID
        request.total_summary = self.total_summary
        request.count_verify = self.count_verify
        request.installation_position = self.installation_position
        request.physical_measurement = self.physical_measurement
        try:
            response = self.client_complete(request)
            msg = f"complete: {response}"
            rospy.loginfo(msg)
            self.logger.write(msg + '\n')
            self.logger.flush()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    @staticmethod
    def pose_matrix(values, translation_scale=1.0):
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ValueError('位姿必须是六个数值 [x, y, z, rx, ry, rz]')
        values = [float(v) for v in values]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('位姿不能包含 NaN 或无穷大')
        matrix = euler_matrix(*(math.radians(v) for v in values[3:]), axes='sxyz')
        matrix[:3, 3] = [v * translation_scale for v in values[:3]]
        return matrix

    def read_cartesian_transform(self, name):
        values = rospy.get_param('~' + name, None)
        if values is None:
            raise ValueError(f'笛卡尔模式需要先填写 {name}，不能默认两个坐标系相同')
        return self.pose_matrix(values)

    def cartesian_target(self, tcp_pose):
        # T_reference_tip = T_reference_user * T_user_tcp * inverse(T_tip_tcp)
        arm_base_link_target = self.pose_matrix(tcp_pose, 0.001)
        rospy.logerr(f'(arm_base_link_target)')
        target = self.reference_from_user.dot(arm_base_link_target).dot(self.tcp_from_tip)
        quaternion = quaternion_from_matrix(target)
        values = tuple(float(v) for v in target[:3, 3]) + tuple(float(v) for v in quaternion)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('转换后的目标位姿无效')
        return values

    def read_waypoints_from_csv(self):
        if not os.path.exists(self.waypoints_file_path):
            rospy.logerr(f'Waypoint template not found: {self.waypoints_file_path}')
        try:
            with open(self.waypoints_file_path, 'r', newline='', encoding='utf-8-sig') as f:
                reader = csv.reader(f)
                header = next(reader, [])
                tcp_columns = ('tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
                               'tcp_rx_deg', 'tcp_ry_deg', 'tcp_rz_deg')
                if self.execute_type == 'cartesian':
                    if any(header.count(name) != 1 for name in tcp_columns):
                        raise ValueError('笛卡尔模式需要 demonstrate 录制的六列 TCP 位姿')
                    tcp_indices = [header.index(name) for name in tcp_columns]
                for line_number, parts in enumerate(reader, start=2):
                    if not parts:
                        continue
                    if len(parts) < 18:
                        raise ValueError(f'CSV 第 {line_number} 行不足 18 列')
                    detect_type = parts[10].strip().upper()
                    waypoint = {
                        '项点编号': parts[0],
                        '底盘x': float(parts[1]),
                        '底盘y': float(parts[2]),
                        '底盘yaw': float(parts[3]),
                        '关节1': float(parts[4]),
                        '关节2': float(parts[5]),
                        '关节3': float(parts[6]),
                        '关节4': float(parts[7]),
                        '关节5': float(parts[8]),
                        '关节6': float(parts[9]),
                        '检测类型': detect_type, 
                        '项点类型': parts[11],
                        '报活处理班组': parts[12],
                        '架次': parts[13],
                        '方位': parts[14],
                        '左右位': parts[15],
                        '位别': parts[16],
                        '项点名称': parts[17]
                    }
                    if not all(math.isfinite(waypoint[key]) for key in (
                            '底盘x', '底盘y', '底盘yaw',
                            '关节1', '关节2', '关节3', '关节4', '关节5', '关节6')):
                        raise ValueError(f'CSV 第 {line_number} 行含无效底盘或关节数值')
                    if self.execute_type == 'cartesian':
                        if len(parts) <= max(tcp_indices):
                            raise ValueError(f'CSV 第 {line_number} 行缺少 TCP 位姿')
                        waypoint['tcp_pose'] = tuple(float(parts[i]) for i in tcp_indices)
                        waypoint['cartesian_target'] = self.cartesian_target(waypoint['tcp_pose'])
                    self.waypoints.append(waypoint)
                    # only care about YOLO, DINO, OCR, MEASURE
                    if detect_type == 'YOLO':
                        self.count_verify.total_count += 1
                        self.total_summary.total_count += 1
                    elif detect_type in ('DINO', 'OCR'):
                        self.installation_position.total_count += 1
                        self.total_summary.total_count += 1
                    elif detect_type == 'MERE':
                        self.physical_measurement.total_count += 1
                        self.total_summary.total_count += 1
                    else:
                        # skip other types (TEMP, ----)
                        continue
            rospy.loginfo(f'Loaded {len(self.waypoints)} waypoints, {self.total_summary.total_count} items.')
            self.logger.write(f'Loaded {len(self.waypoints)} waypoints, , {self.total_summary.total_count} items.\n')
            self.logger.flush()
        except Exception as e:
            rospy.logerr(f'{str(e)}')
            self.logger.write(f'[error] {str(e)}')
            self.logger.flush()
            self.waypoints = []
            raise
    

    def read_temp_waypoints(self):
        self.temp_waypoints_before = []
        self.temp_waypoints_after = []

        temp_file = None
        if self.robotId == 2:
            waypoints_dir_path = resolve_ros_path(
                rospy.get_param('~waypoints_dir_path', '')
            )
            temp_file = os.path.join(waypoints_dir_path, f'{self.trainTypeName}-robot2-temppoints.csv')
        elif self.robotId == 3:
            waypoints_dir_path = resolve_ros_path(
                rospy.get_param('~waypoints_dir_path', '')
            )
            temp_file = os.path.join(waypoints_dir_path, f'{self.trainTypeName}-robot3-temppoints.csv')
        else:
            return

        if not os.path.exists(temp_file):
            rospy.logwarn("Temp waypoint file not found: %s", temp_file)
            return

        target = self.temp_waypoints_before
        with open(temp_file, 'r', newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            next(reader, None)  # 跳过表头
            for p in reader:
                if not p:
                    target = self.temp_waypoints_after
                    continue
                if len(p) < 10:
                    continue
                target.append({
                    '项点编号': '0',
                    '底盘x': float(p[1]),
                    '底盘y': float(p[2]),
                    '底盘yaw': float(p[3]),
                    '关节1': float(p[4]),
                    '关节2': float(p[5]),
                    '关节3': float(p[6]),
                    '关节4': float(p[7]),
                    '关节5': float(p[8]),
                    '关节6': float(p[9]),
                    '项点名称': 'TEMP'
                })

    def execute_temp_waypoints(self, waypoints):
        for waypoint in waypoints:
            self.send_base_request(waypoint)
            # self.send_arm_request(waypoint)

    def is_base_move_needed(self, waypoint, previous_waypoint):
        current_is_transition = waypoint['项点名称'] == 'TEMP_NO_MOVE_BASE'
        if current_is_transition:
            return False
        if previous_waypoint is None:
            return True
        if previous_waypoint['项点名称'] == 'TEMP_NO_MOVE_BASE':
            return True

        distance = math.hypot(
            waypoint['底盘x'] - previous_waypoint['底盘x'],
            waypoint['底盘y'] - previous_waypoint['底盘y'],
        )
        if distance <= self.min_base_move_distance:
            rospy.loginfo(
                f"项点{waypoint['项点编号']}与前一项点距离{distance:.3f}m，跳过底盘运动"
            )
            return False
        return True

    def start_patrol(self):
        if self.workMode == 'cooperative':
            self.read_temp_waypoints()
            self.execute_temp_waypoints(self.temp_waypoints_before)

        previous_waypoint = None
        for waypoint in self.waypoints:
            # ONLINE
            if self.workMode == 'cooperative':
                if self.robotId == 2:
                # 机器人2：只处理 左侧 或 前侧
                    if waypoint['方位'] not in ['左侧', '前侧']:
                        continue
                elif self.robotId == 3:
                    # 机器人3：只处理 右侧
                    if waypoint['方位'] != '右侧':
                        continue
            if self.is_base_move_needed(waypoint, previous_waypoint):
                self.send_base_request(waypoint)
            self.send_arm_request(waypoint)
            previous_waypoint = waypoint
            if 'TEMP' in waypoint['项点名称']:
                continue  # 跳过临时项点的图像采集和上传
            rospy.loginfo(f"waypoint name: {waypoint['项点名称']}")
            detect_response = self.send_detect_pic_request(waypoint)
            # ONLINE
            if self.mode == 'online':
                self.send_http_report_data(waypoint, detect_response)  # TODO 发送通信请求

            is_abnormal = False if detect_response and detect_response.analyse_result == "ok" else True
            dt = waypoint.get('检测类型')
            if dt == 'YOLO':
                self.count_verify.inspected_count += 1
                if is_abnormal:
                    self.count_verify.abnormal_count += 1
            elif dt in ('DINO', 'OCR'):
                self.installation_position.inspected_count += 1
                if is_abnormal:
                    self.installation_position.abnormal_count += 1
            elif dt == 'MERE':
                self.physical_measurement.inspected_count += 1
                if is_abnormal:
                    self.physical_measurement.abnormal_count += 1
            self.total_summary.inspected_count += 1
            if is_abnormal:
                self.total_summary.abnormal_count += 1
        if self.workMode == 'cooperative':
            self.execute_temp_waypoints(self.temp_waypoints_after)

        req = DisableRobot._request_class()
        response = self.client_arm_disable_robot(req)
        rospy.loginfo(f"机械臂已下使能：{response}")

        rospy.loginfo(f"巡检任务完成，统计结果：{self.total_summary}")
        # ONLINE
        if self.mode == 'online':
            self.send_robot_complete()

    def run(self):
        if self.mode != 'online':
            self.reset_task_state()
            self.init_path_clients()
            self.start_patrol()
            self.logger.close()
            return

        while not rospy.is_shutdown():
            self.spin_until_start()
            if rospy.is_shutdown():
                break

            try:
                self.reset_task_state()
                self.init_path_clients()
                self.start_patrol()
            except Exception as e:
                rospy.logerr(f'巡检任务执行异常: {e}')
                self.logger.write(f'[error] 巡检任务执行异常: {e}\n')
                self.logger.flush()
            finally:
                self.received_robot_start_request = False
                self.task_active = False
                self.img_dir_prefix = ''

        self.logger.close()

def main(args=None):
    patrol = PatrolNode()
    patrol.run()

if __name__ == '__main__':
    main()
