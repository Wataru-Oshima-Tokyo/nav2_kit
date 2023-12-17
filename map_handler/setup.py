from setuptools import find_packages, setup
import glob
from os.path import isdir
package_name = 'map_handler'
library = 'map_handler/library'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, library],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
         ('share/' + package_name + '/param', glob.glob('param/*')),
         ('share/' + package_name + '/maps', [f for f in glob.glob('maps/**/*', recursive=True) if not isdir(f)]),
         ('share/' + package_name + '/pcd', glob.glob('pcd/*')),
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
             'map_position_change_node = map_handler.load_map:main',
             'marker_localization = map_handler.marker_to_odom:main',
             'init_pose_setter = map_handler.initial_pose_setter:main',
             'scan_to_costmap = map_handler.scan_to_costmap:main',
        ],
    },
)
