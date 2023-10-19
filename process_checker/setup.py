import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'process_checker'
library = 'process_checker/library'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, library],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*.launch.py')),
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
             'process_handler = process_checker.process_handler:main',
        ],
    },
)
