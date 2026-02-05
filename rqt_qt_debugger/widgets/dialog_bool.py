# widgets/dialog_bool.py
# 继承Qt6弹窗基类，和DialogString一致
from PyQt6.QtWidgets import QDialog
# 导入自动生成的Bool UI类（和你的ui_dialog_string导入路径一致）
from rqt_qt_debugger.ui.ui_dialog_bool import Ui_dialog_bool

# 继承QDialog（弹窗基类）+ Ui_dialog_bool（自动生成的UI类），和DialogString完全同结构
class DialogBool(QDialog, Ui_dialog_bool):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化可视化设计的UI（和你的String弹窗一致）
        self.setWindowTitle("Bool数据选择")  # 自定义弹窗标题，更友好

        # 初始化选中的布尔值，默认False
        self.selected_bool = False
        # 绑定按钮点击事件：点击True/False记录值并关闭弹窗
        self._bind_buttons()

    def _bind_buttons(self):
        """绑定True/False按钮的点击事件，私有方法"""
        # 点击True按钮：记录值为True，关闭弹窗
        self.btn_true.clicked.connect(self._on_true_clicked)
        # 点击False按钮：记录值为False，关闭弹窗
        self.btn_false.clicked.connect(self._on_false_clicked)

    def _on_true_clicked(self):
        """True按钮点击回调"""
        self.selected_bool = True
        self.accept()  # 触发弹窗关闭，和QDialog的确认逻辑一致

    def _on_false_clicked(self):
        """False按钮点击回调"""
        self.selected_bool = False
        self.accept()  # 触发弹窗关闭

    def get_result(self):
        """给主窗口返回选择结果，和DialogString完全相同的固定格式：(ROS2消息类型, 选择内容)"""
        # Bool类型对应ROS2的std_msgs/msg/Bool，和String的消息类型保持规范
        return "std_msgs/msg/Bool", "True" if self.selected_bool else "False"