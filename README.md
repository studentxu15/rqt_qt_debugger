## 设计

### 将.ui 文件转换为.py 文件
```php
pyuic6 resource/ui_file/main.ui -o rqt_qt_debugger/ui/ui_main.py
```
### 模块 4.1：ROS2 节点封装（core/ros2_node.py）

这是核心，把 ROS2 的话题发布 / 订阅逻辑封装成类，所有 ROS2 操作都在这里，界面只需要调用这个类的方法即可，同时处理线程安全和ROS2 spin 阻塞问题

### 模块 4.2：界面逻辑封装（ui/main_window.py）—— 页面切换 + ROS2 调用 + 线程安全

封装界面的按钮点击事件、页面切换、界面更新，重点解决PyQt 的线程安全

### 模块 4.3：程序主入口（main.py）—— 启动界面 + 初始化

这是整个工具的唯一入口，setup.py会配置这个文件为 ROS2 的可执行文件，后期用ros2 run启动
