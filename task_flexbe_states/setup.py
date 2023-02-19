from setuptools import setup

package_name = 'task_flexbe_states'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_action_ros1_state = task_flexbe_states.example_action_ros1_state',
            'example_ros1_state = task_flexbe_states.example_ros1_state',
            'example_action_state = task_flexbe_states.example_action_state',
            'example_state = task_flexbe_states.example_state',
            'moveit_load_mesh = task_flexbe_states.tmp.moveit_load_mesh',
            'coordinate_transform_state = task_flexbe_states.coordinate_transform_state',
            'get_DIO_state = task_flexbe_states.get_DIO_state',
            'gqcnn_grasp_plan_state = task_flexbe_states.gqcnn_grasp_plan_state',
            'grasped_and_stop = task_flexbe_states.grasped_and_stop',
            'moveit_execute_traj_state = task_flexbe_states.moveit_execute_traj_state',
            'moveit_joint_plan_state = task_flexbe_states.moveit_joint_plan_state',
            'moveit_pose_plan_state = task_flexbe_states.moveit_pose_plan_state',
            'set_DIO_state = task_flexbe_states.set_DIO_state',
            'set_place_pose_random_state = task_flexbe_states.set_place_pose_random_state',
        ],
    },
)
