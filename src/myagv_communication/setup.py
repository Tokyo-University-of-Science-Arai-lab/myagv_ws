from setuptools import find_packages, setup

package_name = 'myagv_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/myagv_communication.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agv2',
    maintainer_email='agv2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial = myagv_communication.serial:main",
            'pd_control_node = myagv_communication.pd_control_node:main',
            "diff_drive_controller_node = myagv_communication.diff_drive_controller:main",
            "move_control_node = myagv_communication.move_control_node:main",
            "odometry_node = myagv_communication.odometry:main",
            "testpd_node = myagv_communication.testpd:main",
            "message_node = myagv_communication.message:main",
            "move_forward_node = myagv_communication.move_forward_node:main",
            'test_node = myagv_communication.test_node:main',
            "trans_command_node = myagv_communication.trans_command_node:main",
            "distance_node = myagv_communication.distance:main",
            "second_move_forward_node = myagv_communication.second_move_forward_node:main",
            "second_pd_control_node = myagv_communication.second_pd_control_node:main",
        ],
    },
)
