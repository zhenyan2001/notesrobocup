import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dynamic_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotino',
    maintainer_email='robotino@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf = dynamic_tf.static_tf:main',
            'dyn_tf_gripper_y_ori = dynamic_tf.dyn_tf_gripper_y_ori:main',
            'dyn_tf_gripper_x_ori = dynamic_tf.dyn_tf_gripper_x_ori:main',
            'dyn_tf_gripper_z_ori = dynamic_tf.dyn_tf_gripper_z_ori:main',
            'dyn_tf_base_cam = dynamic_tf.dyn_tf_base_cam:main',
            'static_tf_back_laser=dynamic_tf.static_tf_back_laser:main',
            'gripper_action_server=dynamic_tf.gripper_action_server:main',
            'gripper_action_client=dynamic_tf.gripper_action_client:main',
            'static_tf_cam_tag=dynamic_tf.static_tf_cam_tag:main',
            'static_tf_end_effector_home=dynamic_tf.static_tf_end_effector_home:main',
            'static_tf_front_laser=dynamic_tf.static_tf_front_laser:main',
            'static_tf_gripper_origin=dynamic_tf.static_tf_gripper_origin:main',
            'static_tf_plate_top=dynamic_tf.static_tf_plate_top:main',
            'static_tf_base_laser=dynamic_tf.static_tf_base_laser:main',
            'test=dynamic_tf.test:main',
        ],
    },
)
