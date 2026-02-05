# widgets/__init__.py
# 导入所有弹窗逻辑类
from .dialog_string import DialogString
from .dialog_twist import DialogTwist

# 弹窗类型映射字典：主程序根据选择的类型，一键实例化对应弹窗
DIALOG_MAP = {
    "String": DialogString,
    "Twist": DialogTwist
}