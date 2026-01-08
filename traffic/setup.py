from setuptools import find_packages, setup
import os           # <--- 이 줄이 반드시 있어야 합니다!
from glob import glob

package_name = 'traffic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dcu',
    maintainer_email='dcu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        # 'traffic_light=traffic.traffic_light:main',
        'traffic_node_exe = traffic.traffic_node_exe:main',
        ],
    },
)
