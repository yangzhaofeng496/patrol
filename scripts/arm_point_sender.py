#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import rospy
from tf.transformations import (
    euler_matrix,
    inverse_matrix,
    quaternion_from_matrix,
)

from moma_service.srv import ServoJ
from patrol.srv import SendArmPoint, SendArmPointResponse

try:
    from patrol.path_utils import resolve_ros_path
except ImportError:
    from path_utils import resolve_ros_path


class ArmPointSender:
    """Read one CSV waypoint and send it in joint or Cartesian mode.

    The mode is selected at node startup with the private parameter
    ``~execute_type``. Changing the parameter requires restarting the node.
    """

    TCP_COLUMNS = (
        "tcp_x_mm",
        "tcp_y_mm",
        "tcp_z_mm",
        "tcp_rx_deg",
        "tcp_ry_deg",
        "tcp_rz_deg",
    )

    JOINT_COLUMNS = (
        "关节1",
        "关节2",
        "关节3",
        "关节4",
        "关节5",
        "关节6",
    )

    def __init__(self): 
        rospy.init_node("arm_point_sender")

        self.execute_type = rospy.get_param(
            "~execute_type", "joint"
        ).strip().lower()

        if self.execute_type not in ("joint", "cartesian"):
            raise ValueError(
                "~execute_type 必须是 joint 或 cartesian"
            )

        self.csv_path = resolve_ros_path(
            rospy.get_param(
                "~csv_path",
                "$(find demonstrate)/path/HXD1C_test.csv",
            )
        )
        self.arm_service_name = rospy.get_param(
            "~arm_service",
            "/dobot_v4_bringup/srv/ServoJ"
            if self.execute_type == "joint"
            else "/move_to_pose",
        )
        self.settle_time = float(
            rospy.get_param("~settle_time", 5.5)
        )

        self.reference_from_user = None
        self.tcp_from_tip = None
        self.arm_service_type = ServoJ

        if self.execute_type == "cartesian":
            from moveit_whole_body_ik.srv import MoveToPose

            self.arm_service_type = MoveToPose
            self.reference_from_user = self.read_cartesian_transform(
                "cartesian_reference_from_user"
            )
            self.tcp_from_tip = inverse_matrix(
                self.read_cartesian_transform(
                    "cartesian_tip_from_tcp"
                )
            )

        self.waypoints = self.load_csv()
        rospy.loginfo(
            "正在读取 CSV 文件: %s",
            os.path.abspath(self.csv_path),
        )
        rospy.loginfo(
            "等待机械臂服务 [%s]，模式 [%s]",
            self.arm_service_name,
            self.execute_type,
        )
        rospy.wait_for_service(
            self.arm_service_name,
            timeout=10.0,
        )

        self.arm_client = rospy.ServiceProxy(
            self.arm_service_name,
            self.arm_service_type,
        )

        self.server = rospy.Service(
            "send_arm_point",
            SendArmPoint,
            self.handle_send_arm_point,
        )

        rospy.loginfo(
            "机械臂点位发送节点启动成功：mode=%s, waypoints=%d",
            self.execute_type,
            len(self.waypoints),
        )

    @staticmethod
    def pose_matrix(values, translation_scale=1.0):
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ValueError(
                "位姿必须是 [x, y, z, rx, ry, rz] 六个数值"
            )

        values = [float(value) for value in values]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("位姿不能包含 NaN 或无穷大")

        matrix = euler_matrix(
            *(math.radians(value) for value in values[3:]),
            axes="sxyz",
        )
        matrix[:3, 3] = [
            value * translation_scale
            for value in values[:3]
        ]
        return matrix

    def read_cartesian_transform(self, name):
        values = rospy.get_param("~" + name, None)
        if values is None:
            raise ValueError(
                f"笛卡尔模式必须配置 ~{name}，不能使用 null"
            )
        return self.pose_matrix(values)

    def cartesian_target(self, tcp_pose):
        # T_reference_tip = T_reference_user * T_user_tcp * inverse(T_tip_tcp)
        arm_base_link_target = self.pose_matrix(tcp_pose, 0.001)
        target = self.reference_from_user.dot(arm_base_link_target)
        quaternion = quaternion_from_matrix(target)
        values = tuple(float(v) for v in target[:3, 3]) + tuple(float(v) for v in quaternion)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('转换后的目标位姿无效')
        return values


    def load_csv(self):
        waypoints = {}

        required_columns = ["项点编号"]
        if self.execute_type == "joint":
            required_columns += list(self.JOINT_COLUMNS)
        else:
            required_columns += list(self.TCP_COLUMNS)

        try:
            with open(
                self.csv_path,
                "r",
                newline="",
                encoding="utf-8-sig",
            ) as csv_file:
                reader = csv.DictReader(csv_file)

                if reader.fieldnames is None:
                    raise RuntimeError("CSV 没有表头")

                missing = [
                    column
                    for column in required_columns
                    if column not in reader.fieldnames
                ]
                if missing:
                    raise RuntimeError(
                        "CSV 缺少字段: " + ", ".join(missing)
                    )

                for line_number, row in enumerate(reader, start=2):
                    point_id = row["项点编号"].strip()
                    if not point_id:
                        rospy.logwarn(
                            "CSV 第 %d 行没有项点编号，跳过",
                            line_number,
                        )
                        continue

                    try:
                        point_number = int(point_id)
                    except ValueError:
                        rospy.logwarn(
                            "CSV 第 %d 行项点编号不是整数，跳过: %s",
                            line_number,
                            point_id,
                        )
                        continue

                    # 0 是示教记录中的过渡点，不作为可执行点位。
                    # 必须在重复编号检查之前过滤，否则多个 0 会触发重复错误。
                    if point_number <= 0:
                        rospy.loginfo(
                            "跳过非执行点，第 %d 行项点编号: %s",
                            line_number,
                            point_id,
                        )
                        continue

                    try:
                        if self.execute_type == "joint":
                            joints = [
                                float(row[column])
                                for column in self.JOINT_COLUMNS
                            ]
                            if not all(
                                math.isfinite(value)
                                for value in joints
                            ):
                                raise ValueError(
                                    "关节值包含 NaN 或无穷大"
                                )

                            waypoint = {
                                "point_id": point_id,
                                "joints": joints,
                            }
                        else:
                            tcp_pose = tuple(
                                float(row[column])
                                for column in self.TCP_COLUMNS
                            )
                            if not all(
                                math.isfinite(value)
                                for value in tcp_pose
                            ):
                                raise ValueError(
                                    "TCP 位姿包含 NaN 或无穷大"
                                )

                            waypoint = {
                                "point_id": point_id,
                                "tcp_pose": tcp_pose,
                                "cartesian_target": self.cartesian_target(
                                    tcp_pose
                                ),
                            }

                    except (TypeError, ValueError) as exc:
                        raise RuntimeError(
                            f"CSV 第 {line_number} 行数据无效: {exc}"
                        ) from exc

                    waypoints[point_id] = waypoint

        except OSError as exc:
            raise RuntimeError(
                f"无法打开 CSV 文件: {self.csv_path}: {exc}"
            ) from exc

        if not waypoints:
            raise RuntimeError("CSV 中没有有效机械臂点位")

        return waypoints

    def handle_send_arm_point(self, request):
        point_id = request.point_id.strip()

        try:
            point_number = int(point_id)
        except ValueError:
            return SendArmPointResponse(
                success=False,
                message=f"项点编号必须是整数: {point_id}",
            )

        if point_number <= 0:
            return SendArmPointResponse(
                success=False,
                message=f"只允许执行编号大于 0 的点: {point_id}",
            )

        waypoint = self.waypoints.get(point_id)

        if waypoint is None:
            return SendArmPointResponse(
                success=False,
                message=f"找不到项点编号: {point_id}",
            )

        try:
            arm_request = self.arm_service_type._request_class()

            if self.execute_type == "joint":
                joints = waypoint["joints"]
                arm_request.a = joints[0]
                arm_request.b = joints[1]
                arm_request.c = joints[2]
                arm_request.d = joints[3]
                arm_request.e = joints[4]
                arm_request.f = joints[5]
                arm_request.paramValue = ["t=3"]

            else:
                (
                    arm_request.x,
                    arm_request.y,
                    arm_request.z,
                    arm_request.qx,
                    arm_request.qy,
                    arm_request.qz,
                    arm_request.qw,
                ) = waypoint["cartesian_target"]

            rospy.loginfo(
                "发送项点 %s，模式=%s",
                point_id,
                self.execute_type,
            )

            response = self.arm_client(arm_request)

            if self.execute_type == "cartesian":
                if not response.success:
                    return SendArmPointResponse(
                        success=False,
                        message=(
                            f"笛卡尔运动失败: {response.message}"
                        ),
                    )

            rospy.loginfo("机械臂服务返回: %s", response)
            rospy.sleep(self.settle_time)

            return SendArmPointResponse(
                success=True,
                message=(
                    f"机械臂已发送项点 {point_id}"
                    f"，模式={self.execute_type}"
                ),
            )

        except rospy.ServiceException as exc:
            rospy.logerr("机械臂服务调用失败: %s", exc)
            return SendArmPointResponse(
                success=False,
                message=f"机械臂服务调用失败: {exc}",
            )
        except Exception as exc:
            rospy.logerr("发送项点失败: %s", exc)
            return SendArmPointResponse(
                success=False,
                message=f"发送项点失败: {exc}",
            )


def main():
    try:
        ArmPointSender()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("arm_point_sender 启动失败: %s", exc)


if __name__ == "__main__":
    main()
