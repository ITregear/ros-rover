from setuptools import find_packages, setup

package_name = 'foxglove_bridge_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/foxglove_bridge_launch']),
        ('share/' + package_name, ['package.xml']),
        ('share/foxglove_bridge_launch/launch', ['launch/foxglove_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivantregear@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
