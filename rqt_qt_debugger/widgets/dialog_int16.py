# widgets/dialog_int16.py
from PyQt6.QtWidgets import QDialog
# 导入生成的Int16 UI类
from rqt_qt_debugger.ui.ui_dialog_int16 import Ui_dialog_int16

class DialogInt16(QDialog, Ui_dialog_int16):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化Int16弹窗UI
        self.setWindowTitle("Int16整数输入")  # 自定义弹窗标题，更友好
        # 设置默认值，提升用户体验
        self.textEdit.setPlainText("0")

    def _str_to_int16(self, text, default=0):
        """辅助方法：将输入字符串转为Int16范围的整数，转换失败返回默认值"""
        try:
            # 先去除首尾空格
            clean_text = text.strip()
            # 转换为整数
            num = int(clean_text)
            # 确保数值在Int16范围内（-32768 到 32767）
            if num < -32768:
                return -32768
            elif num > 32767:
                return 32767
            return num
        except (ValueError, TypeError):
            # 转换失败返回默认值
            return default

    def get_result(self):
        """给主窗口返回输入结果，固定格式：(ROS2消息类型, 输入内容)"""
        # 获取文本框输入
        input_content = self.textEdit.toPlainText()
        # 转换为合法的Int16整数（自动处理非数字输入）
        int_value = self._str_to_int16(input_content)
        # Int16类型对应ROS2的std_msgs/msg/Int16，固定消息类型标识
        return "std_msgs/msg/Int16", int_value