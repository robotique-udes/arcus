import os
from glob import glob
from setuptools import setup

package_name = 'param_saver_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ARCUS',
    maintainer='ARCUS',
    description='Parameter saving utility',
    license='MIT',
    entry_points={
        'console_scripts': [
            'param_saver_node = param_saver_node.param_saver_node:main'
        ],
    },
)