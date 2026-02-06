# widgets/dialog_posestamped.py
from PyQt6.QtWidgets import QDialog
from rqt_qt_debugger.ui.ui_dialog_posestamped import Ui_dialog_posestamped

class DialogPoseStamped(QDialog, Ui_dialog_posestamped):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化PoseStamped弹窗UI
        
        # 设置窗口标题
        self.setWindowTitle("PoseStamped位姿输入")
        
        # 初始化所有输入框的默认值
        # 时间偏移（time_offset）
        self.line_offset.setText("0.0")
        # 坐标系ID（frame_id）
        self.line_frame.setText("base_link")
        # position 相关
        self.lineEdit.setText("0.0")    # position.x
        self.lineEdit_2.setText("0.0")  # position.y
        self.lineEdit_3.setText("0.0")  # position.z
        # orientation 相关
        self.lineEdit_4.setText("0.0")  # orientation.x
        self.lineEdit_5.setText("0.0")  # orientation.y
        self.lineEdit_6.setText("0.0")  # orientation.z
        self.lineEdit_7.setText("1.0")  # orientation.w

    def _str_to_float(self, text, default=0.0):
        """辅助方法：将输入的字符串转为浮点数，转换失败返回默认值"""
        try:
            return float(text.strip())
        except (ValueError, TypeError):
            return default

    def get_result(self):
        """
        给主窗口返回PoseStamped数据，固定格式：(ROS2消息类型, 参数字典)
        适配ROS2 geometry_msgs/msg/PoseStamped消息类型
        PoseStamped包含：header(frame_id) + pose(position/orientation) + 可选的time_offset
        """
        # 读取基础参数
        time_offset = self._str_to_float(self.line_offset.text())
        frame_id = self.line_frame.text().strip() or "base_link"  # 确保frame_id有默认值
        
        # 读取position参数
        position_x = self._str_to_float(self.lineEdit.text())
        position_y = self._str_to_float(self.lineEdit_2.text())
        position_z = self._str_to_float(self.lineEdit_3.text())
        
        # 读取orientation参数
        orientation_x = self._str_to_float(self.lineEdit_4.text())
        orientation_y = self._str_to_float(self.lineEdit_5.text())
        orientation_z = self._str_to_float(self.lineEdit_6.text())
        orientation_w = self._str_to_float(self.lineEdit_7.text())
        
        # 封装为PoseStamped标准参数字典，方便主窗口ROS2发布
        posestamped_data = {
            "header": {
                "frame_id": frame_id,
                "time_offset": time_offset  # 时间偏移，可在发布时转换为ROS2的Time类型
            },
            "pose": {
                "position": {"x": position_x, "y": position_y, "z": position_z},
                "orientation": {"x": orientation_x, "y": orientation_y, "z": orientation_z, "w": orientation_w}
            }
        }
        
        # PoseStamped对应ROS2的geometry_msgs/msg/PoseStamped消息类型
        return "geometry_msgs/msg/PoseStamped", posestamped_data