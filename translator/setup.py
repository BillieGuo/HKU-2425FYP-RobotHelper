from setuptools import find_packages, setup

package_name = 'translator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/translator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fyp2',
    maintainer_email='baoguo0816@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master = translator.master:main',
            'socket = translator.socket_server:main',
        ],
    },
)
