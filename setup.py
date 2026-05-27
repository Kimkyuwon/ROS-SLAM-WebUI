from setuptools import setup
import os
from glob import glob

package_name = 'ros_slam_webui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.*')),
        (os.path.join('share', package_name, 'web/static'), glob('web/static/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkw',
    maintainer_email='user@todo.todo',
    description='ROS SLAM Web UI - Web-based control interface for autonomous robot navigation, SLAM, localization, and visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    # scripts 방식 사용 (ROS2 표준)
    scripts=[
        'scripts/web_server',
    ],
)
