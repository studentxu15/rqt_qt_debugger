# widgets/dialog_twist.py
from PyQt6.QtWidgets import QDialog
from rqt_qt_debugger.ui.ui_dialog_pose import Ui_dialog_pose

class DialogPose(QDialog, Ui_dialog_pose):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化Twist弹窗UI（加载6个输入框）
        self.setWindowTitle("Pose位姿输入")
        self.lineEdit.setText("0.0")   # position.x
        self.lineEdit_2.setText("0.0") # position.y
        self.lineEdit_3.setText("0.0") # position.z
        self.lineEdit_4.setText("0.0") # orientation.x
        self.lineEdit_5.setText("0.0") # orientation.y
        self.lineEdit_6.setText("0.0") # orientation.z
        self.lineEdit_7.setText("1.0") # orientation.w

    def _str_to_float(self, text, default=0.0):
        """辅助方法：将输入的字符串转为浮点数，转换失败返回默认值"""
        try:
            return float(text.strip())
        except (ValueError, TypeError):
            return default

    def get_result(self):
        """
        给主窗口返回Pose数据，固定格式：(ROS2消息类型, 速度参数字典)
        适配ROS2 geometry_msgs/msg/Pose: position(x/y/z) + orientation(x/y/z/w)
        """
        # 读取6个输入框的内容，并转为浮点数（转换失败则为0.0）
        position_x = self._str_to_float(self.lineEdit.text())
        position_y = self._str_to_float(self.lineEdit_2.text())
        position_z = self._str_to_float(self.lineEdit_3.text())
        orientation_x = self._str_to_float(self.lineEdit_4.text())
        orientation_y = self._str_to_float(self.lineEdit_5.text())
        orientation_z = self._str_to_float(self.lineEdit_6.text())
        orientation_w = self._str_to_float(self.lineEdit_7.text())
        
        # 封装为Twist标准参数字典，方便主窗口ROS2发布
        pose_data = {
            "position": {"x": position_x, "y": position_y, "z": position_z},
            "orientation": {"x": orientation_x, "y": orientation_y, "z": orientation_z, "w": orientation_w}
        }
        # Twist对应ROS2的geometry_msgs/msg/Twist消息类型
        return "geometry_msgs/msg/Pose", pose_data