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
            'can = INSIA_control.DriverNodes.CAN_Node:main',
            'maxon = INSIA_control.DriverNodes.Maxon_Node:main',
            'faulhaber = INSIA_control.DriverNodes.FAULHABER_Node:main',
            'gears_arduino = INSIA_control.DriverNodes.Gears_Arduino_Node:main',
            'canadac = INSIA_control.DriverNodes.CANADAC_Node:main',
            'ascod_placa_azul = INSIA_control.DriverNodes.ASCOD_Placa_Azul:main',
            'vehicledecoder_base = INSIA_control.DriverNodes.Vehicle_Node_Base:main',
            'io_card = INSIA_control.DriverNodes.IO_Card:main',
            'test_can = test.test_CAN_node:main',
            'can_ascod = INSIA_control.DriverNodes.CAN_NODE_ASCOD:main',
            'maxon_ascod = INSIA_control.DriverNodes.MAXON_NODE_ASCOD:main',

            'brake_imiev = INSIA_control.DevicesControlNodes.Imiev.BrakeNode:main',
            'throttle_imiev = INSIA_control.DevicesControlNodes.Imiev.ThrottleNode:main',
            'throttle_imiev_new = INSIA_control.DevicesControlNodes.Imiev.ThrottleNode_new:main',
            'steering_imiev = INSIA_control.DevicesControlNodes.Imiev.SteeringNode:main',
            'telemetry_imiev = INSIA_control.DevicesControlNodes.Imiev.TelemetryNode:main',

            'brake_lagarto = INSIA_control.DevicesControlNodes.Lagarto.BrakeNode:main',
            'throttle_lagarto = INSIA_control.DevicesControlNodes.Lagarto.ThrottleNode:main',
            'steering_lagarto = INSIA_control.DevicesControlNodes.Lagarto.SteeringNode:main',
            'gears_lagarto = INSIA_control.DevicesControlNodes.Lagarto.GearsNode:main',
            'telemetry_lagarto = INSIA_control.DevicesControlNodes.Lagarto.Telemetry_Basic:main',

            'telemetry_maqueta = INSIA_control.DevicesControlNodes.Maqueta.TelemetryNode:main',
            'throttle_maqueta = INSIA_control.DevicesControlNodes.Maqueta.ThrottleNode:main',
            'steering_maqueta = INSIA_control.DevicesControlNodes.Maqueta.SteeringNode:main',
            'brake_maqueta = INSIA_control.DevicesControlNodes.Maqueta.BrakeNode:main',

            'telemetry_gdels = INSIA_control.DevicesControlNodes.GDELS.TelemetryNode:main',
            'throttle_gdels = INSIA_control.DevicesControlNodes.GDELS.ThrottleNode:main',
            'steering_gdels = INSIA_control.DevicesControlNodes.GDELS.SteeringNode:main',
            'brake_gdels = INSIA_control.DevicesControlNodes.GDELS.BrakeNode:main',

            'dumpbox_lagarto = INSIA_control.DevicesControlNodes.Lagarto.DumpBoxNode:main',
            'brake_emt = INSIA_control.DevicesControlNodes.EMT.BrakeNode:main',
            'throttle_emt = INSIA_control.DevicesControlNodes.EMT.ThrottleNode:main',
            'steering_emt = INSIA_control.DevicesControlNodes.EMT.SteeringNode:main',
            'gears_emt = INSIA_control.DevicesControlNodes.EMT.GearsNode:main',
            'telemetry_emt = INSIA_control.DevicesControlNodes.EMT.Telemetry_Basic:main',

            'brake_ascod = INSIA_control.DevicesControlNodes.ASCOD.BrakeNode:main',
            'throttle_ascod = INSIA_control.DevicesControlNodes.ASCOD.ThrottleNode:main',
            'steering_ascod = INSIA_control.DevicesControlNodes.ASCOD.SteeringNode:main',
            'gears_ascod = INSIA_control.DevicesControlNodes.ASCOD.GearsNode:main',
            'telemetry_ascod = INSIA_control.DevicesControlNodes.ASCOD.Telemetry_Basic:main',


            'joy_transformer = INSIA_control.DevicesControlNodes.JKU.JoyTransformerNode:main',
            'joy_transformer_pet = INSIA_control.DevicesControlNodes.JKU.JoyTransformerPetNode:main',

            'accel_jku_car = INSIA_control.DevicesControlNodes.JKU.Car.AccelNode:main',
            'steering_jku_car = INSIA_control.DevicesControlNodes.JKU.Car.SteeringNode:main',
            'telemetry_jku_car = INSIA_control.DevicesControlNodes.JKU.Car.Telemetry:main',

            'speed_jku_robot = INSIA_control.DevicesControlNodes.JKU.Robot.SpeedNode:main',
            'steering_jku_robot = INSIA_control.DevicesControlNodes.JKU.Robot.SteeringNode:main',
            'telemetry_jku_robot = INSIA_control.DevicesControlNodes.JKU.Robot.Telemetry:main',

            'longitudinal_control = INSIA_control.HLControl.LongitudinalControl:main',
            'longitudinal_control_PID = INSIA_control.HLControl.LongitudinalControlPID:main',
            'longitudinal_control_simple = INSIA_control.HLControl.LongitudinalControlSimple:main',
            'longitudinal_control_simple_ascod = INSIA_control.HLControl.LongitudinalControlSimpleAscod:main',

            'longitudinal_control_IA = INSIA_control.HLControl.LongitudinalControlIA:main',
            'lateral_control_PID = INSIA_control.HLControl.LateralControlPID:main',
            'lateral_control_PID_simple = INSIA_control.HLControl.LateralControlPIDSimple:main',
            'lateral_control = INSIA_control.HLControl.LateralControl:main',
            'gears_control = INSIA_control.HLControl.HLGearsControl:main',

            'decision = INSIA_control.Decision.Decision_Node:main',
            'decision_low = INSIA_control.Decision.decision_low:main',
            'decision_simulador = INSIA_control.Decision.decision_simulador:main',

            'pathplanning_basic = INSIA_control.PathPlanning.PathPlanning_Basic:main',

            'driverCAN = INSIA_control.DriverCAN.DriverCAN:main',
            'driverCANServicio = INSIA_control.DriverCAN.DriverCANServicio:main',
            'telemetry_imiev_srv = INSIA_control.DevicesControlNodes.Imiev.TelemetryNodeSRV:main',

        ],
    },
)
