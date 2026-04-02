import os
from glob import glob

from setuptools import setup

package_name = 'shopmate_bot_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'vosk'],
    zip_safe=True,
    maintainer='admin1',
    maintainer_email='deepsonawane.robotics@gmail.com',
    description='Offline voice for Techno (Vosk + espeak-ng)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'techno_voice_node = shopmate_bot_voice.techno_voice_node:main',
        ],
    },
)
