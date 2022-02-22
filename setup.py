from glob import glob
from setuptools import setup, find_packages

package_name = 'INSIA_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/{}'.format(package_name), glob('launch/*.launch.py')),
        ('share/conf/', glob('conf/*')),
        ('share/test/', glob('test/*'))
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy', 'networkx'],
    zip_safe=True,
    maintainer='Alfredo Valle Barrio',
    maintainer_email='alfredo.valle@upm.es',
    description='Paquete de control universal INSIA en Ros2 Foxy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can = INSIA_control.DriverNodes.CAN_Node:main',
            'maxon = INSIA_control.DriverNodes.Maxon_Node:main',
            'gears_arduino = INSIA_control.DriverNodes.Gears_Arduino_Node:main',
            'canadac = INSIA_control.DriverNodes.CANADAC_Node:main',
            'vehicledecoder = INSIA_control.DriverNodes.Vehicle_Node:main',
            'test_can = test.test_CAN_node:main',
        ],
    },
)
