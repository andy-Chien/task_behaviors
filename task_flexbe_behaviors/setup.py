from setuptools import setup

package_name = 'task_flexbe_behaviors'

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
            'example_behavior_ros1_sm = task_flexbe_behaviors.example_behavior_ros1_sm',
            'example_behavior_sm = task_flexbe_behaviors.example_behavior_sm',
            'go_to_initial_pose_sm = task_flexbe_behaviors.go_to_initial_pose_sm',
            'gqcnn_bin_picking_using_moveit_sm = task_flexbe_behaviors.gqcnn_bin_picking_using_moveit_sm',
            'gqcnn_picking_task_sm = task_flexbe_behaviors.gqcnn_picking_task_sm',
            'grasp_plan_sm = task_flexbe_behaviors.grasp_plan_sm',
            'move_to_pick_sm = task_flexbe_behaviors.move_to_pick_sm',
            'move_to_place_sm = task_flexbe_behaviors.move_to_place_sm',
            'single_arm_random_sampled_planning_sm = task_flexbe_behaviors.single_arm_random_sampled_planning_sm'
            'multi_robot_roadmap_sampler_sm = task_flexbe_behaviors.multi_robot_roadmap_sampler_sm',
            'robot_roadmap_sampler_sm = task_flexbe_behaviors.robot_roadmap_sampler_sm',
        ],
    },
)