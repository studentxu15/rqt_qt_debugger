# widgets/dialog_header.py
from PyQt6.QtWidgets import QDialog
from rqt_qt_debugger.ui.ui_dialog_header import Ui_dialog_header

class DialogHeader(QDialog, Ui_dialog_header):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化Header弹窗UI
        
        # 设置窗口标题
        self.setWindowTitle("Header消息输入")
        
        # 初始化所有输入框的默认值
        self.line_offset.setText("0.0")  # 时间偏移（time_offset）
        self.line_frame.setText("base_link")  # 坐标系ID（frame_id）

    def _str_to_float(self, text, default=0.0):
        """辅助方法：将输入的字符串转为浮点数，转换失败返回默认值"""
        try:
            return float(text.strip())
        except (ValueError, TypeError):
            return default

    def get_result(self):
        """
        给主窗口返回Header数据，固定格式：(ROS2消息类型, 参数字典)
        适配ROS2 std_msgs/msg/Header消息类型
        Header包含：frame_id + 可选的time_offset（时间偏移）
        """
        # 读取并处理输入参数
        time_offset = self._str_to_float(self.line_offset.text())
        frame_id = self.line_frame.text().strip() or "base_link"  # 确保frame_id有默认值
        
        # 封装为Header标准参数字典，方便主窗口ROS2发布
        header_data = {
            "frame_id": frame_id,
            "time_offset": time_offset  # 时间偏移，可在发布时转换为ROS2的Time类型
        }
        
        # Header对应ROS2的std_msgs/msg/Header消息类型
        return "std_msgs/msg/Header", header_data