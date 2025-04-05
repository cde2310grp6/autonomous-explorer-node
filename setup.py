from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ensure launch files are found under custom_explorer package
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unknown',
    maintainer_email='unknown@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = custom_explorer.explorer:main',
            'missionControl = custom_explorer.mission_control:main',
        ],
    },
)
