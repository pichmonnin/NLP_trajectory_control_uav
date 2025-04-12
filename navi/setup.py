from setuptools import find_packages, setup
import os 
from glob import glob


package_name = 'navi'

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
    install_requires=['setuptools','px4_msgs', 'gekko'],
    zip_safe=True,
    maintainer='pich',
    maintainer_email='pich@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "control = navi.control:main",
            "Tra_con = navi.Tra_con:main",
            "Con_Tra =navi.Con_Tra:main",
            "Joy_control = navi.Joy_control:main",
            "altitude_filter = navi.altitude_filter:main",
            "depth_converter = navi.depth_converter:main",
            "navi_filter_control = navi.navi_filter_control:main",
            "camera_bridge = navi.camera_bridge:main"
        ],
    },
)
