from setuptools import find_packages, setup

package_name = 'ros2_bag_to_image'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sepehr',
    maintainer_email='sepehr.saryazdi@gmail.com',
    description='Simple ROS2 bag image extraction package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_bag_to_image = ros2_bag_to_image.ros2_bag_to_image:main',
        ],
    },
)
