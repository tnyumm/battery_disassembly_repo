import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'battery_disassembly_framework'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Exactly like this:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tnyumm',
    maintainer_email='tnyumm@todo.todo',
    description='Battery Disassembly Automation Framework',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator_node = battery_disassembly_framework.orchestrator_node:main',
        ],
    },
)