from setuptools import setup

package_name = 'range_sensors'

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
    maintainer='krissso',
    maintainer_email='krissso@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'sensor_pub = range_sensors.range_sensors:main',
		'sensor_filter = range_sensors.filter:main',
		'range_to_laser = range_sensors.range_to_laser:main',
		'laser_to_pointcloud = range_sensors.laserTopointcloud:main',
        ],
    },
)
