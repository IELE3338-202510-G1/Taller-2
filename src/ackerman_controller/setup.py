from setuptools import find_packages, setup

package_name = 'ackerman_controller'

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
    maintainer='rob',
    maintainer_email='rob@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ackerman_teleop = ackerman_controller.ackerman_teleop:main",
            "arduino_reader = ackerman_controller.arduino_reader:main",
            "arduino_writer = ackerman_controller.arduino_writer:main"
        ],
    },
)
