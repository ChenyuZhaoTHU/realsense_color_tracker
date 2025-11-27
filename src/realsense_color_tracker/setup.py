import os
from glob import glob
from setuptools import setup

package_name = 'realsense_color_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # 1. 注册包资源索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 2. 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        
        # 3. 安装 launch 文件夹下的所有 .launch.py 文件 (这是你需要添加的部分)
        # 注意：这一行必须在这个列表内部，用逗号分隔
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Color tracking node',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'tracker = realsense_color_tracker.tracker_node:main',
        ],
    },
)