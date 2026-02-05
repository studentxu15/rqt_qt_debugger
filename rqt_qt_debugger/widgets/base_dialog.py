# widgets/base_dialog.py
from PyQt6.QtWidgets import QDialog, QPushButton, QHBoxLayout, QVBoxLayout, QWidget
from PyQt6.QtCore import Qt

# 弹窗基础父类：封装公共的确认/取消、模态、数据返回逻辑
class BaseDialog(QDialog):
    def __init__(self, msg_type, parent=None):
        super().__init__(parent)
        self.msg_type = msg_type  # 消息类型（String/Twist）
        self.ros2_msg_type = ""   # 对应ROS2消息类型（如std_msgs/msg/String）
        self.msg_content = ""     # 构造好的消息内容（传给主程序发布）
        self.setModal(True)       # 模态弹窗：必须关闭弹窗才能操作主窗口（核心）
        self.resize(400, 230)     # 默认尺寸，子类可重写
        self._add_confirm_cancel_buttons()  # 统一添加确认/取消按钮

    def _add_confirm_cancel_buttons(self):
        """统一添加确认、取消按钮，水平居中"""
        # 创建按钮
        self.btn_confirm = QPushButton("确认")
        self.btn_cancel = QPushButton("取消")
        # 按钮布局
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_confirm)
        btn_layout.addWidget(self.btn_cancel)
        btn_layout.addStretch()
        # 绑定按钮事件
        self.btn_confirm.clicked.connect(self._on_confirm)
        self.btn_cancel.clicked.connect(self.reject)  # 关闭弹窗，返回Rejected

    def _on_confirm(self):
        """确认按钮：校验参数→通过则保存内容并关闭弹窗"""
        is_valid, result = self._validate_params()
        if not is_valid:
            # 若主窗口有_add_log方法，直接调用主窗口打印错误日志
            if self.parent() and hasattr(self.parent(), "_add_log"):
                self.parent()._add_log(f"❌ {self.msg_type}参数校验失败：{result}")
            return
        self.msg_content = result
        self.accept()  # 关闭弹窗，返回Accepted，通知主程序获取数据

    def _validate_params(self):
        """子类必须实现：参数校验，返回(是否通过, 错误信息/构造好的msg_content)"""
        raise NotImplementedError("子类需重写_validate_params方法")

    def _get_ros2_msg_type(self):
        """子类必须实现：返回对应ROS2消息类型"""
        raise NotImplementedError("子类需重写_get_ros2_msg_type方法")

    def get_result(self):
        """主程序调用：获取(ROS2消息类型, 消息内容)"""
        return self._get_ros2_msg_type(), self.msg_content