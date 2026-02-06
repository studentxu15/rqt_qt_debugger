# widgets/dialog_path.py
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QLineEdit
from PyQt6 import QtCore
from rqt_qt_debugger.ui.ui_dialog_path import Ui_dialog_path

class DialogPath(QDialog, Ui_dialog_path):
    def __init__(self, parent=None):
        super().__init__(parent)  # 初始化Qt弹窗基类
        self.setupUi(self)        # 初始化Path弹窗UI
        
        # 设置窗口标题
        self.setWindowTitle("Path路径点输入")
        
        # 初始化核心：按钮绑定+布局适配+默认值设置
        self._init_ui()
        # 初始化所有输入框的默认值
        self._init_default_values()

    def _init_ui(self):
        """初始化UI相关：按钮绑定+滚动区布局校验"""
        # 绑定加减行按钮
        self.btn_add_row.clicked.connect(self.add_row)
        self.btn_del_row.clicked.connect(self.del_row)
        
        # 校验滚动区布局（防止UI设计时布局丢失，代码兜底）
        if not self.scrollAreaWidgetContents.layout():
            self.scroll_layout = QVBoxLayout(self.scrollAreaWidgetContents)
            self.scroll_layout.setSpacing(10)
            self.scroll_layout.setContentsMargins(10, 10, 10, 10)
        else:
            self.scroll_layout = self.scrollAreaWidgetContents.layout()
        
        # 强制布局贴顶（核心：第一行在滚动区上侧）
        self.scroll_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
        
        # 默认添加第一行，带行号
        self.add_row()

    def _init_default_values(self):
        """初始化所有输入框的默认值"""
        # frame_id输入框默认值
        self.line_frame_id.setText("map")
        
        # 按钮原生逻辑保持（确认/取消）
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

    def _str_to_float(self, text, default=0.0):
        """辅助方法：将输入的字符串转为浮点数，转换失败返回默认值"""
        try:
            return float(text.strip())
        except (ValueError, TypeError):
            return default

    def _create_row(self, row_num):
        """创建单行：行号标签 + 8个输入框（与表头一一对应）"""
        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setSpacing(8)  # 输入框/行号之间的间距
        row_layout.setContentsMargins(0, 0, 0, 0)

        # 1. 行号标签（美化+固定宽度+居中）
        num_label = QLabel(f"第{row_num}行")
        num_label.setFixedWidth(60)
        num_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        num_label.setStyleSheet("""
            background-color: #f8f8f8;
            border: 1px solid #dddddd;
            color: #333333;
            font-weight: 500;
        """)
        row_layout.addWidget(num_label)

        # 2. 8个输入框（固定尺寸，与表头对齐），并设置默认值
        default_values = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0"]
        for idx in range(8):
            line_edit = QLineEdit()
            line_edit.setFixedWidth(90)
            line_edit.setFixedHeight(30)
            # 输入框美化，与行号标签搭配
            line_edit.setStyleSheet("border: 1px solid #dddddd;")
            # 设置每个输入框的默认值
            line_edit.setText(default_values[idx])
            row_layout.addWidget(line_edit)

        return row_widget

    def _update_row_nums(self):
        """更新所有行号（删除行后调用，保证行号连续）"""
        for idx in range(self.scroll_layout.count()):
            # 取每行第一个控件（行号标签）
            row_widget = self.scroll_layout.itemAt(idx).widget()
            num_label = row_widget.layout().itemAt(0).widget()
            num_label.setText(f"第{idx + 1}行")

    def add_row(self):
        """添加行：在最后一行下方新增，自动计算行号"""
        current_count = self.scroll_layout.count()
        new_row = self._create_row(current_count + 1)
        self.scroll_layout.addWidget(new_row)  # 新增行向下排列

    def del_row(self):
        """删除最后一行，至少保留1行，删除后更新行号"""
        current_count = self.scroll_layout.count()
        if current_count > 1:
            # 移除并删除最后一行
            last_item = self.scroll_layout.takeAt(current_count - 1)
            last_item.widget().deleteLater()
            # 更新剩余行的行号
            self._update_row_nums()

    def get_result(self):
        """
        给主窗口返回Path数据，固定格式：(ROS2消息类型, 参数字典)
        适配ROS2 nav_msgs/msg/Path消息类型
        Path包含：header(frame_id/time_offset) + 多个PoseStamped路径点
        """
        # 读取frame_id和时间偏移相关基础参数
        frame_id = self.line_frame_id.text().strip() or "map"
        
        # 遍历所有行，解析8列路径点数据
        path_points = []
        for idx in range(self.scroll_layout.count()):
            row_widget = self.scroll_layout.itemAt(idx).widget()
            row_layout = row_widget.layout()
            
            # 解析每行的8个参数：x,y,z,ox,oy,oz,ow,time_offset
            row_data = {}
            # 跳过行号标签，取第1-8列输入框
            col_mapping = [
                ("time_offset", 1),
                ("x", 2), ("y", 3), ("z", 4),
                ("ox", 5), ("oy", 6), ("oz", 7), ("ow", 8)
            ]
            
            for param_name, col_idx in col_mapping:
                line_edit = row_layout.itemAt(col_idx).widget()
                row_data[param_name] = self._str_to_float(line_edit.text())
            
            # 封装为单个路径点的字典格式
            pose_stamped_data = {
                "header": {
                    "frame_id": frame_id,
                    "time_offset": row_data["time_offset"]
                },
                "pose": {
                    "position": {
                        "x": row_data["x"],
                        "y": row_data["y"],
                        "z": row_data["z"]
                    },
                    "orientation": {
                        "x": row_data["ox"],
                        "y": row_data["oy"],
                        "z": row_data["oz"],
                        "w": row_data["ow"]
                    }
                }
            }
            path_points.append(pose_stamped_data)
        
        # 封装完整的Path参数字典
        path_data = {
            "header": {
                "frame_id": frame_id,
                "time_offset": 0.0  # Path整体的时间偏移，默认0
            },
            "path_points": path_points
        }
        
        # Path对应ROS2的nav_msgs/msg/Path消息类型
        return "nav_msgs/msg/Path", path_data