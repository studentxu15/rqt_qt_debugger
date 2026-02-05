# rqt_qt_debugger/core/utils.py
import os
from ament_index_python.packages import get_package_share_directory

# 加载QSS样式文件（从ROS2包的resource目录加载，避免绝对路径）
def load_qss():
    try:
        # 获取ROS2包的share目录（setup.py配置后，ROS2能找到）
        package_share_dir = get_package_share_directory("rqt_qt_debugger")
        # QSS文件路径
        qss_path = os.path.join(package_share_dir, "resource", "style.qss")
        # 读取QSS文件
        with open(qss_path, "r", encoding="utf-8") as f:
            return f.read()
    except Exception as e:
        print(f"加载QSS样式失败：{str(e)}，将使用默认样式")
        return None
