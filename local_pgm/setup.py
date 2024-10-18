from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'local_pgm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksj',
    maintainer_email='mmj5859@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'tf_broadcaster = local_pgm.tf_broadcaster:main',
            # 'gps_receiver = local_pgm.gps_receiver:main',
            'in = local_pgm.in:main',
        ],
    },
)
