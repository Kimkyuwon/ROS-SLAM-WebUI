from setuptools import setup
import os
from glob import glob

package_name = 'ros_slam_webui'


def _collect_vendor_data_files():
    """Install web/static/vendor recursively (Three.js tree, Plotly, ROSLIB)."""
    vendor_root = os.path.join('web', 'static', 'vendor')
    if not os.path.isdir(vendor_root):
        return []

    data_files = []
    for root, _dirs, files in os.walk(vendor_root):
        if not files:
            continue
        rel = os.path.relpath(root, 'web')
        dest = os.path.join('share', package_name, 'web', rel)
        src_files = [os.path.join(root, f) for f in files]
        data_files.append((dest, src_files))
    return data_files


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
        *_collect_vendor_data_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkw',
    maintainer_email='user@todo.todo',
    description='ROS SLAM Web UI - Web-based control interface for autonomous robot navigation, SLAM, localization, and visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=[
        'scripts/web_server',
    ],
)
