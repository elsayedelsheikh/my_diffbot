from setuptools import find_packages, setup

package_name = 'merlin_bluetooth'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sayed',
    maintainer_email='elsayed.elsheikh97@gmail.com',
    description='BLE provisioning link between the Merlin app and the robot.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['bluetooth_node = merlin_bluetooth.node:main']},
)
