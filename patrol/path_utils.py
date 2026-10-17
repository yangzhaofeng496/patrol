"""解析 patrol 配置中的 ROS 包路径表达式。"""

import re

import rospkg


_ROS_FIND_PATTERN = re.compile(r"\$\(find\s+([A-Za-z0-9_][A-Za-z0-9_-]*)\)")


def resolve_ros_path(value):
    """将 ``$(find <package>)`` 解析为 ROS 包的实际路径。"""
    if not isinstance(value, str):
        return value

    def replace(match):
        package_name = match.group(1)
        try:
            return rospkg.RosPack().get_path(package_name)
        except rospkg.ResourceNotFound:
            raise FileNotFoundError(
                "ROS 包不存在，无法解析路径: {}".format(package_name)
            )

    return _ROS_FIND_PATTERN.sub(replace, value)
