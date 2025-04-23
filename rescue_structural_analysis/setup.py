from setuptools import find_packages, setup

package_name = 'rescue_structural_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/structural_analysis_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shady',
    maintainer_email='shadyrafat60@gmail.com',
    description='Structural Analysis subsystem for TIAGo Search & Rescue Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'structural_analysis_node = rescue_structural_analysis.structural_analysis_node:main',
        ],
    },
)
