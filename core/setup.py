import os
from glob import glob
from setuptools import setup

package_name = 'core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.[yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cristina',
    maintainer_email='cristina@udc.es',
    description='Intermediate layer to manage nodes.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = core.commander_node:main',
            'execution_node = core.execution_node:main',
            'ltm = core.ltm:main'
        ],
    },
)
