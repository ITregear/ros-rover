from setuptools import find_packages, setup

package_name = 'launch_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivantregear@gmail.com',
    description='Launch file for running all rover nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={},
)
