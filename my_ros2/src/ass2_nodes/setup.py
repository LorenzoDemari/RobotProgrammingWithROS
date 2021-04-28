from setuptools import setup

package_name = 'ass2_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'service=ass2_nodes.service_ros2:main',
        	'teleop=ass2_nodes.my_user_interface_ros2:main',
        	'servicebup=ass2_nodes.servicebup_ros2:main'
        ],
    },
)
