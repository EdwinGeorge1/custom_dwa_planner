from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_dwa_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files (if you have any YAML configs)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='tgaedwin@gmail.com',
    description='Custom DWA Planner in Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 👇 Add your Python node here
            'plannar = custom_dwa_planner.dwa_node:main',
        ],
    },
)
