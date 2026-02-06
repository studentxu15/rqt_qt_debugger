from PyQt6.QtWidgets import QMainWindow, QDialog
from PyQt6.QtCore import Qt, QMetaObject, Q_ARG, QTimer
from datetime import datetime
from pathlib import Path
import os

# 导入自动生成的ui类（你的主界面UI，若有则保留，无则注释不影响核心功能）
from .ui_main import Ui_MainWindow
# 新增：导入弹窗类型映射字典
from rqt_qt_debugger.widgets import DIALOG_MAP
# 导入ROS2核心节点
from rqt_qt_debugger.core.ros2_node import ROS2TopicNode

# 新增：YAML解析、ROS2包路径获取
import yaml
from ament_index_python.packages import get_package_share_directory

# 固定包名（和setup.py一致）
PACKAGE_NAME = "rqt_qt_debugger"
# 接收话题自动刷新间隔（毫秒）
AUTO_REFRESH_INTERVAL = 2000

# 主窗口逻辑类（若有Ui_MainWindow则继承：class MainWindow(QMainWindow, Ui_MainWindow):）
class MainWindow(QMainWindow, Ui_MainWindow):
    @property
    def TIME_FORMAT(self):
        now = datetime.now()
        return f"{now.strftime('%Y-%m-%d %H:%M:%S')}.{now.microsecond // 1000:03d}"

    def __init__(self):
        super().__init__()
        # 若有Designer设计的主界面，保留setupUi(self)
        self.setupUi(self)
        self.setWindowTitle("ROS2 话题发布调试工具")
        # 初始化ROS2单例节点
        self.ros2_node = ROS2TopicNode()
        # 初始化界面基础状态（简化：删除分组框相关）
        self._init_ui()
        # 初始化循环发布、自动刷新
        self._init_cycle_publish()
        self._init_topic_auto_refresh()
        # 绑定所有控件交互事件（简化：删除类型切换事件）
        self._bind_all_events()
        # 新增：缓存循环发布的参数（从弹窗获取后缓存）
        self.cycle_topic_name = ""
        self.cycle_msg_type = ""
        self.cycle_msg_content = ""

    def _init_ui(self):
        """初始化界面：仅保留日志/数据框只读、下拉框加载，删除分组框相关"""
        # 若你的主界面有这两个文本框，保留（核心：日志、接收数据）
        self.te_pub_log.setReadOnly(True)
        self.textEdit.setReadOnly(True)
        # 初始化发布/接收话题下拉框
        self._init_pub_topic_combobox()
        self._init_receive_topic_combobox()
        self.current_receive_topics = self._get_current_receive_topics()
        # 初始化日志提示
        self._add_log("🔧 ROS2 话题发布调试工具已启动（弹窗版）")
        self._add_log("📌 发布：选择话题+类型，点击单次/循环发布，弹窗输入参数")
        self._add_log("📊 监测：选择接收话题，点击监测查看实时数据")
        self._add_log(f"🔄 接收话题自动刷新已开启，间隔{AUTO_REFRESH_INTERVAL/1000}秒")
        self._add_log("💾 保存：点击[保存数据/保存日志]，文件存储至 ~/.ros/debuggertools/")
        self._add_log("🗑️ 清空：点击对应清空按钮，清除接收数据/运行日志")

    def _init_pub_topic_combobox(self):
        """原有逻辑：从YAML加载发布话题，无则用默认"""
        try:
            share_dir = get_package_share_directory(PACKAGE_NAME)
            yaml_path = os.path.join(share_dir, "resource", "config", "topic_list.yaml")
            with open(yaml_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
                topic_list = config.get("topic_names", [])
            if not topic_list:
                topic_list = ["/chatter", "/cmd_vel"]
                self._add_log("⚠️ 发布话题配置为空，加载默认：/chatter、/cmd_vel")
            self.cbb_pub_topic.clear()
            self.cbb_pub_topic.addItems(topic_list)
            self._add_log(f"✅ 发布话题加载完成：共{len(topic_list)}个（YAML配置）")
        except Exception as e:
            default_topics = ["/chatter", "/cmd_vel"]
            self.cbb_pub_topic.clear()
            self.cbb_pub_topic.addItems(default_topics)
            self._add_log(f"❌ 发布话题配置加载失败：{e}，使用默认话题")

    def _init_receive_topic_combobox(self):
        """原有逻辑：从ROS2系统实时获取接收话题"""
        topic_list = self.ros2_node.get_ros2_topics()
        self.cbb_received_topic.clear()
        self.cbb_received_topic.addItems(topic_list)
        self._add_log(f"✅ 接收话题初始加载完成：共{len(topic_list)}个（ROS2系统现有）")

    def _init_cycle_publish(self):
        """原有逻辑：初始化循环发布定时器"""
        self.cycle_publish_running = False
        self.cycle_timer = QTimer(self)
        self.cycle_timer.timeout.connect(self._publish_topic_cycle)

    def _init_topic_auto_refresh(self):
        """原有逻辑：初始化接收话题自动刷新定时器"""
        self.auto_refresh_running = True
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(AUTO_REFRESH_INTERVAL)
        self.refresh_timer.timeout.connect(self._refresh_receive_topic)
        self.refresh_timer.start()

    def _bind_all_events(self):
        """绑定事件：删除类型切换事件，其余保留"""
        self.btn_open_input.clicked.connect(self._open_input_dialog)
        self.btn_publish.clicked.connect(self._publish_topic_once)       # 单次发布
        self.btn_publish_cyc.clicked.connect(self._on_cycle_publish_click) # 循环发布
        self.btn_detect.clicked.connect(self._on_detect_click)           # 监测
        # 保存/清空事件（原有）
        self.btn_debug.clicked.connect(self._save_receive_data)    # 保存数据
        self.btn_debug_2.clicked.connect(self._save_running_log)   # 保存日志
        self.pushButton.clicked.connect(self._clear_receive_data)  # 清空接收数据
        self.pushButton_2.clicked.connect(self._clear_running_log) # 清空运行日志

    # ---------------------- 新增：弹窗调用核心方法 ----------------------
    def _open_input_dialog(self):
        """打开弹窗输入数据，缓存到主窗口，点击Ok自动关闭弹窗"""
        select_type = self.cbb_topic_type.currentText().strip()
        if select_type not in DIALOG_MAP:
            self._add_log(f"❌ 不支持的发布类型：{select_type}，仅支持String")
            return False
        try:
            dialog_cls = DIALOG_MAP[select_type]
            dialog = dialog_cls(self)  # 模态弹窗：弹窗不关无法操作主窗口
            dialog_result = dialog.exec()  # 打开弹窗，等待用户操作并返回结果码
            
            # 【核心修复】用QDialog的标准结果码判断，而非对象属性
            if dialog_result == QDialog.DialogCode.Accepted:  # 点击Ok按钮
                ros2_msg_type, msg_content = dialog.get_result()
                if not msg_content:  # 校验空输入
                    self._add_log(f"❌ {select_type}类型参数输入为空，请重新输入！")
                    return False
                # 缓存数据到主窗口，供发布按钮使用
                self.cycle_msg_type = ros2_msg_type
                self.cycle_msg_content = msg_content
                self._add_log(f"✅ 已输入{select_type}数据并缓存：{msg_content}")
                self._add_log(f"💡 选择话题后，点击「单次发布/循环发布」即可发送")
                return True
            else:  # 点击Cancel按钮/关闭弹窗
                self._add_log(f"🔴 取消{select_type}类型参数输入")
                return False
        except Exception as e:
            self._add_log(f"❌ 打开{select_type}弹窗失败：{str(e)}")
            return False

    # ---------------------- 原有方法：仅修改消息内容获取方式 ----------------------
    def _get_current_receive_topics(self):
        """原有逻辑：获取当前接收话题下拉框内容"""
        return [self.cbb_received_topic.itemText(i) for i in range(self.cbb_received_topic.count())]

    def _refresh_receive_topic(self):
        """原有逻辑：增量刷新接收话题"""
        new_topics = self.ros2_node.get_ros2_topics()
        old_topics = self.current_receive_topics
        if set(new_topics) == set(old_topics):
            return
        added_topics = list(set(new_topics) - set(old_topics))
        removed_topics = list(set(old_topics) - set(new_topics))
        self.cbb_received_topic.clear()
        self.cbb_received_topic.addItems(new_topics)
        self.current_receive_topics = new_topics
        refresh_info = f"🔄 接收话题已刷新 | 总数：{len(new_topics)}"
        if added_topics:
            refresh_info += f" | 新增：{sorted(added_topics)}"
        if removed_topics:
            refresh_info += f" | 删除：{sorted(removed_topics)}"
        self._add_log(refresh_info)

    def _get_save_dir(self):
        """原有逻辑：获取保存目录，无则创建"""
        home_dir = Path.home()
        save_dir = home_dir / ".ros" / "debuggertools"
        save_dir.mkdir(parents=True, exist_ok=True)
        return str(save_dir)

    def _generate_file_name(self, file_suffix):
        """原有逻辑：生成带时间戳的文件名"""
        time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_name = f"debuggertools{time_str}{file_suffix}.log"
        save_dir = self._get_save_dir()
        full_path = os.path.join(save_dir, file_name)
        return full_path

    def _save_receive_data(self):
        """原有逻辑：保存接收数据"""
        data_content = self.textEdit.toPlainText().strip()
        if not data_content:
            self._add_log("❌ 保存数据失败：接收数据框为空！")
            return
        save_path = self._generate_file_name("data")
        try:
            with open(save_path, "w", encoding="utf-8") as f:
                f.write(data_content)
            self._add_log(f"✅ 接收数据保存成功 | 路径：{save_path}")
        except Exception as e:
            self._add_log(f"❌ 接收数据保存失败：{str(e)}")

    def _save_running_log(self):
        """原有逻辑：保存运行日志"""
        log_content = self.te_pub_log.toPlainText().strip()
        if not log_content:
            self._add_log("❌ 保存日志失败：运行日志框为空！")
            return
        save_path = self._generate_file_name("log")
        try:
            with open(save_path, "w", encoding="utf-8") as f:
                f.write(log_content)
            self._add_log(f"✅ 运行日志保存成功 | 路径：{save_path}")
        except Exception as e:
            self._add_log(f"❌ 运行日志保存失败：{str(e)}")

    def _clear_receive_data(self):
        """原有逻辑：清空接收数据"""
        self.textEdit.clear()
        self._add_log("🗑️ 已清空接收数据框")

    def _clear_running_log(self):
        """原有逻辑：清空运行日志"""
        self.te_pub_log.clear()
        self._add_log("🗑️ 已清空运行日志框")

    def _publish_topic_once(self):
        """单次发布：使用弹窗缓存的数据，无数据则提示先输入"""
        # 1. 校验是否选择发布话题
        topic_name = self.cbb_pub_topic.currentText().strip()
        if not topic_name:
            self._add_log(f"❌ 单次发布失败：请先选择发布话题！")
            return
        # 2. 校验是否已通过弹窗输入并缓存数据
        if not self.cycle_msg_type or not self.cycle_msg_content:
            self._add_log(f"❌ 单次发布失败：请先点击「输入数据」按钮填写发布内容！")
            return
        # 3. 使用缓存的数据发布
        publish_res = self.ros2_node.publish_topic(
            topic_name, 
            self.cycle_msg_type, 
            self.cycle_msg_content
        )
        if "成功" in publish_res:
            self._add_log(f"✅ 单次发布成功 | 话题：{topic_name} | 内容：{self.cycle_msg_content}")
        else:
            self._add_log(f"❌ 单次发布失败 | {publish_res}")

    def _on_cycle_publish_click(self):
        """循环发布：使用弹窗缓存的数据，无数据则提示先输入"""
        self.cycle_publish_running = not self.cycle_publish_running
        if self.cycle_publish_running:
            # 1. 校验是否选择发布话题
            topic_name = self.cbb_pub_topic.currentText().strip()
            if not topic_name:
                self._add_log(f"❌ 循环发布启动失败：请先选择发布话题！")
                self.cycle_publish_running = False
                return
            # 2. 校验是否已通过弹窗输入并缓存数据
            if not self.cycle_msg_type or not self.cycle_msg_content:
                self._add_log(f"❌ 循环发布启动失败：请先点击「输入数据」按钮填写发布内容！")
                self.cycle_publish_running = False
                return
            # 3. 校验发布频率
            freq = self._get_float(self.pub_frequence, min_val=0.1)
            if freq <= 0:
                self._add_log(f"❌ 循环发布启动失败：频率需大于0！")
                self.cycle_publish_running = False
                return
            # 4. 缓存话题名（循环发布需要），使用已缓存的消息类型和内容
            self.cycle_topic_name = topic_name
            # 启动定时器循环发布
            self.cycle_timer.setInterval(int(1000 / freq))
            self.cycle_timer.start()
            self.btn_publish_cyc.setText("停止发布")
            self._add_log(f"✅ 循环发布已启动 | 话题：{topic_name} | 频率：{freq}Hz | 内容：{self.cycle_msg_content}")
        else:
            # 停止循环发布
            self.cycle_timer.stop()
            self.btn_publish_cyc.setText("循环发布")
            self._add_log(f"🔴 循环发布已停止 | 最后发布话题：{self.cycle_topic_name}")

    def _publish_topic_cycle(self):
        """原有逻辑：循环发布核心，完全复用"""
        try:
            publish_res = self.ros2_node.publish_topic(
                self.cycle_topic_name,
                self.cycle_msg_type,
                self.cycle_msg_content
            )
            if "失败" in publish_res:
                self._add_log(f"❌ 循环发布失败 | {publish_res}")
                self._on_cycle_publish_click()
        except Exception as e:
            self._add_log(f"❌ 循环发布异常：{e}，已自动停止")
            self._on_cycle_publish_click()

    def _on_detect_click(self):
        """原有逻辑：监测/停止监测接收话题，完全复用"""
        topic_name = self.cbb_received_topic.currentText().strip()
        if not topic_name:
            self._add_log("❌ 监测操作失败：请选择接收话题名！")
            return
        if topic_name in self.ros2_node.subscribers:
            unsub_res = self.ros2_node.unsubscribe_topic(topic_name)
            self.btn_detect.setText("监测")
            self._add_log(f"🔴 监测停止 | {unsub_res}")
            # self.textEdit.clear()
        else:
            sub_res = self.ros2_node.subscribe_topic(topic_name, self._add_receive_data)
            if "成功" in sub_res:
                self.btn_detect.setText("停止监测")
            self._add_log(f"📊 监测操作 | {sub_res}")

    def _get_float(self, le_widget, min_val=None):
        """原有逻辑：获取浮点值，支持最小值校验，完全复用"""
        text = le_widget.text().strip()
        try:
            val = float(text) if text else 0.0
            if min_val is not None and val < min_val:
                self._add_log(f"⚠️ 「{le_widget.objectName()}」值{val} < {min_val}，使用{min_val}")
                return min_val
            return val
        except ValueError:
            self._add_log(f"⚠️ 「{le_widget.objectName()}」输入非数字，使用默认值0.0")
            return 0.0 if min_val is None else min_val

    def _add_log(self, content):
        """原有逻辑：线程安全添加日志，完全复用"""
        current_time = datetime.now().strftime(self.TIME_FORMAT)
        data_content = f"[{current_time}] {content}"
        QMetaObject.invokeMethod(
            self.te_pub_log,
            "append",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(str, data_content)
        )

    def _add_receive_data(self, content):
        """原有逻辑：线程安全添加接收数据，完全复用"""
        current_time = datetime.now().strftime(self.TIME_FORMAT)
        data_content = f"[{current_time}] {content}"
        QMetaObject.invokeMethod(
            self.textEdit,
            "append",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(str, data_content)
        )

    def closeEvent(self, event):
        """原有逻辑：窗口关闭清理资源，完全复用"""
        if self.cycle_publish_running:
            self.cycle_timer.stop()
            self._add_log("🔴 窗口关闭，已自动停止循环发布")
        if self.auto_refresh_running:
            self.refresh_timer.stop()
            self._add_log("🔄 窗口关闭，已停止接收话题自动刷新")
        self.ros2_node.destroy()
        self._add_log("🔌 ROS2连接已断开，工具即将关闭...")
        event.accept()