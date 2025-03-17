from setuptools import find_packages, setup

package_name = 'grasp_execute'

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
    description='Handle the grasp request and response',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_request_node = grasp_execute.grasp_request_node:main'
        ],
    },
)
