# widgets/dialog_twist.py
from PyQt6.QtWidgets import QDialog
# 导入Twist弹窗的UI生成类（按你的实际目录调整，和String保持一致）
from rqt_qt_debugger.ui.ui_dialog_twist import Ui_dialog_twist

class DialogTwist(QDialog, Ui_dialog_twist):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化Twist弹窗UI（加载6个输入框）
        self.setWindowTitle("Twist速度数据输入")  # 自定义弹窗标题，更友好
        # 可选：设置输入框默认值（避免空输入，也可让用户手动填）
        self.lineEdit.setText("0.0")   # 线速度x
        self.lineEdit_2.setText("0.0") # 线速度y
        self.lineEdit_3.setText("0.0") # 线速度z
        self.lineEdit_4.setText("0.0") # 角速度x
        self.lineEdit_5.setText("0.0") # 角速度y
        self.lineEdit_6.setText("0.0") # 角速度z

    def _str_to_float(self, text, default=0.0):
        """辅助方法：将输入的字符串转为浮点数，转换失败返回默认值"""
        try:
            return float(text.strip())
        except (ValueError, TypeError):
            return default

    def get_result(self):
        """
        给主窗口返回Twist数据，固定格式：(ROS2消息类型, 速度参数字典)
        适配ROS2 geometry_msgs/msg/Twist：linear(x/y/z) + angular(x/y/z)
        """
        # 读取6个输入框的内容，并转为浮点数（转换失败则为0.0）
        linear_x = self._str_to_float(self.lineEdit.text())
        linear_y = self._str_to_float(self.lineEdit_2.text())
        linear_z = self._str_to_float(self.lineEdit_3.text())
        angular_x = self._str_to_float(self.lineEdit_4.text())
        angular_y = self._str_to_float(self.lineEdit_5.text())
        angular_z = self._str_to_float(self.lineEdit_6.text())
        
        # 封装为Twist标准参数字典，方便主窗口ROS2发布
        twist_data = {
            "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
            "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
        }
        # Twist对应ROS2的geometry_msgs/msg/Twist消息类型
        return "geometry_msgs/msg/Twist", twist_data