from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'machining_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='catdog',
    maintainer_email='catdog@example.com',
    description='A2 模拟机床上下料：任务状态机 + 机械臂控制(MoveIt) + 工位信号 + 运行记录',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scene_manager = machining_demo.scene_manager:main',
            'arm_controller = machining_demo.arm_controller:main',
            'machine_simulator = machining_demo.machine_simulator:main',
            'task_manager = machining_demo.task_manager:main',
            'result_recorder = machining_demo.result_recorder:main',
            'demo_test = machining_demo.demo_test:main',
        ],
    },
)
