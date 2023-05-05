from setuptools import setup
import os
from setuptools import setup
from glob import glob


package_name = 'imu_odometry'
INS = "imu_odometry/INS"
INSTOOLS = "imu_odometry/INS/tools"
INSDETECTORS = "imu_odometry/INS/detectors"


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, INS, INSTOOLS, INSDETECTORS],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chathushka-sutd',
    maintainer_email='chathushka.ranasinghe41@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'exec_py = imu_odometry.main:main',
        ],
    },
)

