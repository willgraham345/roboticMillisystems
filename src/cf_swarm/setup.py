import os
from glob import glob
from setuptools import setup
from ament_index_python.packages import get_package_share_directory


package_name = 'cf_swarm'
package_share = get_package_share_directory('crazyflie_ros2_slam')


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_share), glob('/launch/slam_toolbox_mapping_simulation.py'))
    ],
    zip_safe=True,
    maintainer='will',
    maintainer_email='willgraham345@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cf_swarm_launch = cf_swarm.cf_swarm_launch:main",
            
        ],
    },
    
)
