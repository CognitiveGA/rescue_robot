from setuptools import find_packages, setup

package_name = 'rescue_actuator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/actuator_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shady',
    maintainer_email='shadyrafat60@gmail.com',
    description='Actuator subsystem controlling left and right motors with a singleton controller.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator_node = rescue_actuator.actuator_node:main',
        ],
    },
)
