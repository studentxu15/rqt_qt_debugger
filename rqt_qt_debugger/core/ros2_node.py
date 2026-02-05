import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading
from std_msgs.msg import String, Bool, Int16, Float32, Header
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path

class ROS2TopicNode:
    # 单例模式：全局唯一ROS2节点，避免重复初始化
    _instance = None
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        # 初始化ROS2上下文，仅执行一次
        if not rclpy.ok():
            rclpy.init(args=None)
        # 创建ROS2节点
        self.node = Node("qt_ros2_debugger_node")
        # 发布者字典：key=话题名，value=发布者对象
        self.publishers = {}
        # 订阅者字典：key=话题名，value=订阅者对象
        self.subscribers = {}
        # 订阅回调函数字典：key=话题名，value=界面更新回调
        self.sub_callbacks = {}
        # ROS2单线程执行器（避免阻塞Qt界面）
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # 启动ROS2自旋线程（守护线程，随主进程退出）
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

        # 程序支持的消息类型（后续扩展仅需在这里添加）
        self.supported_msg = {
            "std_msgs/msg/String": String,
            "std_msgs/msg/Bool": Bool,
            "std_msgs/msg/Int16": Int16,
            "std_msgs/msg/Float32": Float32,
            "std_msgs/msg/Header": Header,
            "geometry_msgs/msg/Twist": Twist,
            "geometry_msgs/msg/Pose": Pose,
            "geometry_msgs/msg/PoseStamped": PoseStamped,
            "nav_msgs/msg/Odometry": Odometry,
            "nav_msgs/msg/Path": Path
        }

    def _spin(self):
        """ROS2自旋，处理订阅回调，运行在子线程"""
        self.executor.spin()

    def publish_topic(self, topic_name, msg_type, msg_content):
        """发布话题方法（单次/循环发布通用）"""
        # 校验消息类型是否支持
        if msg_type not in self.supported_msg:
            return f"错误：不支持的消息类型{msg_type}，当前支持{list(self.supported_msg.keys())}"
        # 不存在则创建发布者（队列大小10）
        if topic_name not in self.publishers:
            self.publishers[topic_name] = self.node.create_publisher(
                self.supported_msg[msg_type], topic_name, 10
            )
        # 构造消息并发布
        try:
            msg = self.supported_msg[msg_type]()
            if isinstance(msg_content, dict):
                params = msg_content
            else:
                params = [p.strip() for p in msg_content.split(",") if p.strip()]
            # #################### 基础标量类型（无时间戳） ####################
            if msg_type == "std_msgs/msg/String":
                msg.data = msg_content
            elif msg_type == "std_msgs/msg/Bool":
                msg.data = msg_content.lower() in ["true", "1"]
            elif msg_type == "std_msgs/msg/Int16":
                msg.data = int(msg_content)
            elif msg_type == "std_msgs/msg/Float32":
                msg.data = float(msg_content)
            # #################### 运动控制类型（无时间戳） ####################
            elif msg_type == "geometry_msgs/msg/Twist":
                msg.linear.x = params["linear"].get("x", 0.0)
                msg.linear.y = params["linear"].get("y", 0.0)
                msg.linear.z = params["linear"].get("z", 0.0)
                msg.angular.x = params["angular"].get("x", 0.0)
                msg.angular.y = params["angular"].get("y", 0.0)
                msg.angular.z = params["angular"].get("z", 0.0)
            # #################### 带时间戳类型：Header ####################
            elif msg_type == "std_msgs/msg/Header":
                offset_sec = float(params[0]) if params else 0.0
                msg.stamp = self._get_ros_time_with_offset(offset_sec)
                msg.frame_id = params[1] if len(params)>1 else "base_link"
            # #################### 位姿类型：Pose ####################
            elif msg_type == "geometry_msgs/msg/Pose":
                msg.position.x = params["position"].get("x", 0.0)
                msg.position.y = params["position"].get("y", 0.0)
                msg.position.z = params["position"].get("z", 0.0)
                # 四元数：x,y,z,w（ROS标准，w是实部，缺省w=1.0）
                msg.orientation.x = params["orientation"].get("x", 0.0)
                msg.orientation.y = params["orientation"].get("y", 0.0)
                msg.orientation.z = params["orientation"].get("z", 0.0)
                msg.orientation.w = params["orientation"].get("w", 1.0)
            elif msg_type == "geometry_msgs/msg/PoseStamped":
                offset_sec = float(params[0]) if params else 0.0
                msg.header.stamp = self._get_ros_time_with_offset(offset_sec)
                msg.header.frame_id = params[1] if len(params)>1 else "base_link"
                msg.pose.position.x = float(params[2]) if len(params)>2 else 0.0
                msg.pose.position.y = float(params[3]) if len(params)>3 else 0.0
                msg.pose.position.z = float(params[4]) if len(params)>4 else 0.0
                msg.pose.orientation.x = float(params[5]) if len(params)>5 else 0.0
                msg.pose.orientation.y = float(params[6]) if len(params)>6 else 0.0
                msg.pose.orientation.z = float(params[7]) if len(params)>7 else 0.0
                msg.pose.orientation.w = float(params[8]) if len(params)>8 else 1.0
            elif msg_type == "nav_msgs/msg/Odometry":
                offset_sec = float(params[0]) if params else 0.0
                msg.header.stamp = self._get_ros_time_with_offset(offset_sec)
                msg.header.frame_id = params[1] if len(params)>1 else "odom"
                msg.child_frame_id = params[2] if len(params)>2 else "base_link"
                msg.pose.pose.position.x = float(params[3]) if len(params)>3 else 0.0
                msg.pose.pose.position.y = float(params[4]) if len(params)>4 else 0.0
                msg.pose.pose.position.z = float(params[5]) if len(params)>5 else 0.0
                msg.pose.pose.orientation.x = float(params[6]) if len(params)>6 else 0.0
                msg.pose.pose.orientation.y = float(params[7]) if len(params)>7 else 0.0
                msg.pose.pose.orientation.z = float(params[8]) if len(params)>8 else 0.0
                msg.pose.pose.orientation.w = float(params[9]) if len(params)>9 else 1.0

            # 发布消息
            self.publishers[topic_name].publish(msg)
            return f"成功发布[{topic_name}]"
        except ValueError as e:
            return f"发布失败：参数类型错误，请输入数字！{str(e)}"
        except IndexError as e:
            return f"发布失败：参数数量不足，请按格式输入！{str(e)}"
        except Exception as e:
            return f"发布失败：{str(e)}"
    
    def subscribe_topic(self, topic_name, callback):
        """【核心改造】订阅话题：自动识别类型，仅传话题名+回调"""
        try:
            # 1. 获取ROS2系统中所有现有话题的类型映射
            topic_type_map = self._get_topic_type_map()
            # 2. 校验话题是否存在
            if topic_name not in topic_type_map:
                return f"失败：话题[{topic_name}]不存在，请先启动对应发布节点"
            # 3. 提取话题类型（一个话题可对应多类型，取第一个）
            msg_type = topic_type_map[topic_name][0]
            if not msg_type:
                return f"失败：话题[{topic_name}]无有效消息类型"
            # 4. 校验类型是否支持
            if msg_type not in self.supported_msg:
                return f"失败：话题[{topic_name}]类型[{msg_type}]不支持，仅支持{list(self.supported_msg.keys())}"
            # 5. 重复订阅则先取消
            if topic_name in self.subscribers:
                self.unsubscribe_topic(topic_name)
            # 6. 保存界面回调
            self.sub_callbacks[topic_name] = callback
            # 7. 创建订阅者
            self.subscribers[topic_name] = self.node.create_subscription(
                self.supported_msg[msg_type],
                topic_name,
                lambda msg, t=topic_name: self._sub_callback(t, msg),
                10
            )
            return f"成功订阅[{topic_name}]（自动识别类型：{msg_type}）"
        except Exception as e:
            return f"订阅失败：{str(e)}"

    def _get_topic_type_map(self):
        """工具方法：获取ROS2系统中所有话题的{话题名: [类型列表]}映射"""
        topic_list = self.node.get_topic_names_and_types()
        return {topic: types for topic, types in topic_list}

    def _sub_callback(self, topic_name, msg):
        """ROS2内部订阅回调：格式化消息并转发到界面"""
        if topic_name in self.sub_callbacks and callable(self.sub_callbacks[topic_name]):
            # 格式化String/Twist消息，避免原生对象显示
            if isinstance(msg, String):
                msg_str = msg.data
            elif isinstance(msg, Bool):
                msg_str = f"布尔值：{msg.data}"
            elif isinstance(msg, Int16):
                msg_str = f"整数：{msg.data}"
            elif isinstance(msg, Float32):
                msg_str = f"浮点数：{msg.data:.2f}"
            elif isinstance(msg, Twist):
                msg_str = (f"线速度(x:{msg.linear.x:.2f}, y:{msg.linear.y:.2f}, z:{msg.linear.z:.2f}) | "
                           f"角速度(x:{msg.angular.x:.2f}, y:{msg.angular.y:.2f}, z:{msg.angular.z:.2f})")
            elif isinstance(msg, Header):
                msg_str = f"frame_id: {msg.frame_id}"
            elif isinstance(msg, Pose):
                msg_str = (f"位置(x:{msg.position.x:.2f}, y:{msg.position.y:.2f}, z:{msg.position.z:.2f}) | "
                           f"四元数(x:{msg.orientation.x:.2f}, y:{msg.orientation.y:.2f}, z:{msg.orientation.z:.2f}, w:{msg.orientation.w:.2f})")
            elif isinstance(msg, PoseStamped):
                msg_str = (f"frame_id: {msg.header.frame_id} | "
                           f"位置(x:{msg.pose.position.x:.2f}, y:{msg.pose.position.y:.2f}), z:{msg.pose.position.z:.2f}) | "
                           f"四元数(x:{msg.pose.orientation.x:.2f}, y:{msg.pose.orientation.y:.2f}, z:{msg.pose.orientation.z:.2f}, w:{msg.pose.orientation.w:.2f})")
            elif isinstance(msg, Odometry):
                msg_str = (f"frame_id:{msg.header.frame_id} | child_frame_id: {msg.child_frame_id} | "
                           f"位置(x:{msg.pose.pose.position.x:.2f}, y:{msg.pose.pose.position.y:.2f}), z:{msg.pose.pose.position.z:.2f}) | "
                           f"四元数(x:{msg.pose.pose.orientation.x:.2f}, y:{msg.pose.pose.orientation.y:.2f}, z:{msg.pose.pose.orientation.z:.2f}, w:{msg.pose.pose.orientation.w:.2f})")
            else:
                msg_str = str(msg)
            # 转发到界面回调
            self.sub_callbacks[topic_name](f"[{topic_name}] {msg_str}")
    
    def unsubscribe_topic(self, topic_name):
        """取消订阅话题"""
        if topic_name in self.subscribers:
            self.node.destroy_subscription(self.subscribers[topic_name])
            del self.subscribers[topic_name]
            del self.sub_callbacks[topic_name]
            return f"成功取消订阅[{topic_name}]"
        return f"失败：未订阅[{topic_name}]"

    def get_ros2_topics(self):
        """工具方法：获取ROS2系统中所有现有话题名（供界面下拉框加载）"""
        try:
            topic_type_map = self._get_topic_type_map()
            return sorted(topic_type_map.keys())  # 排序返回，更友好
        except Exception as e:
            self.node.get_logger().error(f"获取ROS2话题失败：{e}")
            return ["/chatter", "/cmd_vel"]  # 兜底默认话题

    def destroy(self):
        """销毁节点，释放所有ROS2资源"""
        if rclpy.ok():
            # 销毁所有发布者/订阅者
            for pub in self.publishers.values():
                self.node.destroy_publisher(pub)
            for sub in self.subscribers.values():
                self.node.destroy_subscription(sub)
            # 销毁节点并关闭ROS2上下文
            self.node.destroy_node()
            rclpy.shutdown()