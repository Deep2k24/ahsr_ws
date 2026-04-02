import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ahsr_web_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deep',
    maintainer_email='your_email@todo.todo',
    description='Web GUI and Voice Control for Autonomous Hospital Service Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'techno_voice_node = ahsr_web_gui.techno_voice_node:main',
            'web_gui_node = ahsr_web_gui.web_gui_node:main',
        ],
    },
)
