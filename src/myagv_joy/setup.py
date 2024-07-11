# from setuptools import find_packages, setup
# import os
# from glob import glob

# package_name = 'myagv_joy'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/launch', ['myagv_joy/launch/myagv_joy.launch.py']),
#         #(os.path.join('share', package_name), glob('launch/*.launch.py')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='agv1',
#     maintainer_email='agv1@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             "teleop_twist_joy = myagv_joy.teleop_twist_joy:main",
#         ],
#     },
# )

from setuptools import setup

package_name = 'myagv_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/myagv_joy.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agv1',
    maintainer_email='agv1@todo.todo@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_twist_joy_node = myagv_joy.teleop_twist_joy:main"
        ],
    },
)
