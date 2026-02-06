# widgets/__init__.py
# 导入所有弹窗逻辑类
from .dialog_string import DialogString
from .dialog_twist import DialogTwist
from .dialog_pose import DialogPose
from .dialog_bool import DialogBool
from .dialog_posestamped import DialogPoseStamped
from .dialog_int16 import DialogInt16
from .dialog_float32 import DialogFloat32
from .dialog_header import DialogHeader
from .dialog_odometry import DialogOdometry

# 弹窗类型映射字典：主程序根据选择的类型，一键实例化对应弹窗
DIALOG_MAP = {
    "String": DialogString,
    "Twist": DialogTwist,
    "Pose": DialogPose,
    "Bool": DialogBool,
    "PoseStamped": DialogPoseStamped,
    "Int16": DialogInt16,
    "Float32": DialogFloat32,
    "Header": DialogHeader,
    "Odometry": DialogOdometry
}