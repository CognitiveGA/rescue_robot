from setuptools import find_packages, setup

package_name = 'rescue_core_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/core_system_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shady',
    maintainer_email='shadyrafat60@gmail.com',
    description='Core system for mapping and autonomous navigation in a rescue robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'core_system_node = rescue_core_system.core_system_node:main',
        ],
    },
)
