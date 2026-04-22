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
    install_requires=['setuptools', 'PyYAML', 'numpy', 'networkx', 'simple_pid', 'python-can==4.3.1'],
    zip_safe=True,
    maintainer='Alfredo Valle Barrio',
    maintainer_email='alfredo.valle@upm.es',
    description='Paquete de control universal INSIA en Ros2 Foxy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_mutt = INSIA_control.DevicesControlNodes.MUTT.Telemetry_Basic:main',
            'canadac_mutt = INSIA_control.DriverNodes.CANADAC_MUTT:main',
            'device_mutt = INSIA_control.DevicesControlNodes.MUTT.MUTT_Device_Node:main',
            'control_mutt = INSIA_control.HLControl.MUTTControl:main',
            'decision_mutt = INSIA_control.DevicesControlNodes.MUTT.decision_MUTT:main',
        ],
    },
)
