# widgets/dialog_float32.py
from PyQt6.QtWidgets import QDialog
# 导入生成的Float32 UI类
from rqt_qt_debugger.ui.ui_dialog_float32 import Ui_dialog_float32

class DialogFloat32(QDialog, Ui_dialog_float32):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化Float32弹窗UI
        self.setWindowTitle("Float32浮点数输入")  # 自定义弹窗标题，更友好
        # 设置默认值，提升用户体验（Float32默认0.0更合理）
        self.textEdit.setPlainText("0.0")

    def _str_to_float32(self, text, default=0.0):
        """
        辅助方法：将输入字符串转为Float32范围的浮点数，转换失败返回默认值
        Float32范围：±3.402823466e+38（单精度浮点数标准范围）
        """
        try:
            # 先去除首尾空格
            clean_text = text.strip()
            # 转换为浮点数
            num = float(clean_text)
            
            # 限制Float32数值范围（单精度浮点数极限值）
            float32_min = -3.402823466e+38
            float32_max = 3.402823466e+38
            if num < float32_min:
                return float32_min
            elif num > float32_max:
                return float32_max
            return num
        except (ValueError, TypeError):
            # 转换失败（如输入非数字）返回默认值
            return default

    def get_result(self):
        """给主窗口返回输入结果，固定格式：(ROS2消息类型, 输入内容)"""
        # 获取文本框输入内容
        input_content = self.textEdit.toPlainText()
        # 转换为合法的Float32浮点数（自动处理非数字输入）
        float_value = self._str_to_float32(input_content)
        # Float32类型对应ROS2的std_msgs/msg/Float32，固定消息类型标识
        return "std_msgs/msg/Float32", float_value