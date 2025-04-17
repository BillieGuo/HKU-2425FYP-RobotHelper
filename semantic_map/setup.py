from setuptools import find_packages, setup

package_name = 'semantic_map'

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
    maintainer='fyp2',
    maintainer_email='baoguo0816@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_socket_send = semantic_map.image_socket_send:main',
            'image_socket_send_test = semantic_map.image_socket_send_test:main',
            'listen_print_tf = semantic_map.listen_print_tf:main',
            'query_semantic_map = semantic_map.query_semantic_map:main',
            'query_client = semantic_map.query_client:main',
        ],
    },
)
