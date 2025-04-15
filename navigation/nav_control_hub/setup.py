from setuptools import find_packages, setup

package_name = 'nav_control_hub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigator.launch.py']),  # Added line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bao Guo',
    maintainer_email='baoguo0816@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial = serial_task.serial_node:main',
            'control = main_control.control_node:main',
            'explore = main_control.yolo_explore:main',
            'socket = main_control.socket_client:main',
            'fakeSemantic = main_control.fake_semantic_map:main'
        ],
    },
)
