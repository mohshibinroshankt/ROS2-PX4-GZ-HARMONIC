from setuptools import find_packages, setup
from glob import glob

package_name = 'rrt_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),  
        ('share/' + package_name, glob('config/*.yaml')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shibin',
    maintainer_email='shibin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
    'test': ['pytest'],
},
    entry_points={
        'console_scripts': [
            'my_node = rrt_package.my_node:main',
            'my_node2 = rrt_package.my_node2:main',
            'mynode3 = rrt_package.mynode3:main',
            'lidar_data_node = rrt_package.lidar_data_node:main',
            'my_node4 = rrt_package.my_node4:main',
            'ob_av = rrt_package.ob_av:main',
            'test = rrt_package.test:main',
            'lidar_proc = rrt_package.lidar_proc:main',
            'simple_offboard = rrt_package.simple_offboard:main',
        ],
    },
)
