import rclpy
import sys
from PyQt6.QtWidgets import QApplication
from rqt_qt_debugger.ui.main_window import MainWindow
from rqt_qt_debugger.core.utils import load_qss

def main(args=None):
    rclpy.init(args=args)
    # 初始化PyQt应用
    app = QApplication(sys.argv)
    # 加载QSS样式文件
    qss_style = load_qss()
    if qss_style:
        app.setStyleSheet(qss_style)
    # 创建主窗口并显示
    window = MainWindow()
    window.show()
    # 运行PyQt主循环
    sys.exit(app.exec())

if __name__ == "__main__":
    main()