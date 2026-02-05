# widgets/dialog_string.py
from PyQt6.QtWidgets import QDialog
# 导入你生成的String UI类
from rqt_qt_debugger.ui.ui_dialog_string import Ui_dialog_string

class DialogString(QDialog, Ui_dialog_string):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化弹窗UI（和主窗口setupUi逻辑一致）
        self.setWindowTitle("String数据输入")  # 自定义弹窗标题，更友好

    def get_result(self):
        """给主窗口返回输入结果，固定格式：(ROS2消息类型, 输入内容)"""
        # 获取文本框输入，去除首尾空格
        input_content = self.textEdit.toPlainText().strip()
        # String类型对应ROS2的std_msgs/msg/String，固定消息类型标识
        return "std_msgs/msg/String", input_content