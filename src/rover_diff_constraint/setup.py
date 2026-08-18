from setuptools import setup
import os
from glob import glob

package_name = 'rover_diff_constraint'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zeynep Altundal',
    maintainer_email='zeynep@surover.local',
    description='Software differential bar (anti-phase constraint) for rocker diff joints',
    license='MIT',
    entry_points={
        'console_scripts': [
            'diff_constraint_node = rover_diff_constraint.diff_constraint_node:main',
        ],
    },
)
