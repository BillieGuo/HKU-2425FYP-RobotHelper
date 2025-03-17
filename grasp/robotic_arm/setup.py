from setuptools import find_packages, setup

package_name = 'robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zeng Shaoyu',
    maintainer_email='zshaoyu@connect.hku.hk',
    description='Package on board for robotic arm manipulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_request_node = robotic_arm.grasp_request_node:main',
            'grasp_response_node = robotic_arm.grasp_response_node:main',
            'eye_in_hand_calibration = robotic_arm.eye_in_hand_calibration:main',
        ],
    },
)
