from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'controller_pro'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name , 'launch') , glob("launch/*")), 
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pich',
    maintainer_email='monninpich6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "trajectory_control = controller_pro.trajectory_control:main",
            "data_bridge = controller_pro.data_bridge:main",
            "visual_rviz2 = controller_pro.visual_rviz2:main",
            "trajectory_mavros =controller_pro.trajectory_mavros:main",
            "test1 =controller_pro.test1:main",
        ],
    },
)
