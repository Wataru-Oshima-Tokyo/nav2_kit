from setuptools import find_packages, setup
import glob
package_name = 'goal_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wataru',
    maintainer_email='wataru.bb.tokyo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_saver_node = goal_handler.goal_saver:main',
            'goal_handler_node = goal_handler.goal_publisher:main',
            'imu_pose_publisher = goal_handler.imu_pose_publisher:main',
            'demo_recoveries = goal_handler.demo_recoveries:main',
            'logging_test = goal_handler.logging_test:main'
        ],
    },
)
