from setuptools import find_packages, setup

package_name = 'rescue_search_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/search_tasks_launch.py', 'launch/test_comm_to_tasks_launch.py', 'launch/test_perception_to_triage_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shady',
    maintainer_email='shadyrafat60@gmail.com',
    description='Search and Rescue decision-making subsystem for TIAGo robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_and_rescue_node = rescue_search_tasks.search_and_rescue_node:main',
        ],
    },
)
