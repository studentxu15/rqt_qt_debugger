from setuptools import setup, find_packages
import glob

package_name = 'rqt_qt_debugger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        # 1. ament包索引标记（解决package index marker警告）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 2. 核心配置文件package.xml
        ('share/' + package_name, ['package.xml']),
        # 3. launch文件夹（保留）
        (f'share/{package_name}/launch', glob.glob('launch/*.py')),
        
        # 4. 复制resource根目录下的所有文件（如style.qss、索引文件）
        (f'share/{package_name}/resource', glob.glob('resource/*.*')),
        
        # 5. 【关键】复制resource下的子文件夹及内部文件（按需添加，保留目录结构）
        # 示例：复制resource/ui_file文件夹及里面所有文件
        (f'share/{package_name}/resource/ui_file', glob.glob('resource/ui_file/*')),
        # 若有其他子文件夹（如resource/icon、resource/config），按此格式追加即可
        # (f'share/{package_name}/resource/icon', glob.glob('resource/icon/*')),
        (f'share/{package_name}/resource/config', glob.glob('resource/config/*')),
    ],
    # 依赖包
    install_requires=['setuptools', 'rclpy', 'PyQt6', 'ament_index_python', 'pyyaml'],
    zip_safe=True,
    maintainer='Student Xu',
    maintainer_email='xuchunbobo520@gmail.com',
    description='ROS2 Qt6可视化调试工具, 支持String/Twist话题发布.',
    license='Apache-2.0',
    # 替代弃用的tests_require
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'main = rqt_qt_debugger.main:main',
        ],
    },
)