from enum import IntEnum

from src.utils.utils import make_can_msg

QC_FACTOR = 1  # 625000 / 360
DIGITAL_OUTPUT_3 = False
DIGITAL_OUTPUT_4 = False


class EPOSCommand(IntEnum):
    SHUTDOWN = 0x06
    SWITCH_ON = 0x07
    SWITCH_ON_AND_ENABLE = 0x0F
    DISABLE_VOLTAGE = 0x00
    QUICK_STOP = 0x02
    DISABLE_OPERATION = 0x07
    ENABLE_OPERATION = 0x0F
    FAULT_RESET = 0x80


status_epos = {
    0: 'Not ready to switch on',
    64: 'Switch on disabled',
    33: 'Ready to switch on',
    35: 'Switched on',
    39: 'Operation enabled',
    7: 'Quick stop active',
    15: 'Fault reaction active',
    8: 'Fault'
}
mode_epos = {
    'PPM': 1,
    'PVM': 3,
    'HMM': 6,
    'CSP': 8,
    'CSV': 9,
    'CST': 10
}

def enable(node: int):
    return [
        make_can_msg(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE)]
    # return [
    #     make_can_msg(node=node, index=0x6040, data=EPOSCommand.SHUTDOWN),
    #     make_can_msg(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON),
    #     make_can_msg(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE),
    #     make_can_msg(node=node, index=0x6060, data=0x01)]


def read_status(node: int):
    return [make_can_msg(node=node, index=0x6041, write=False)]


def read_io(node: int):
    return [make_can_msg(node=node, index=0x2078, sub_index=1, write=False)]


def read_position(node: int):
    return [make_can_msg(node=node, index=0x6064, write=False)]


def disable(node: int):
    return [
        make_can_msg(node=node, index=0x6040, data=EPOSCommand.DISABLE_OPERATION),
    ]


def fault_reset(node: int):
    return [make_can_msg(node=node, index=0x6040, data=EPOSCommand.FAULT_RESET),
            make_can_msg(node, 0x6040, 0, EPOSCommand.SHUTDOWN),
            make_can_msg(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE), ]


def set_angle_value(node: int, angle: int, absolute: bool = False, motor_type: str = 'MCD60'):
    qc_to_rotate = int(QC_FACTOR * angle)
    set_angle = []
    if absolute:
        set_angle += [make_can_msg(node=node, index=0x607A, data=qc_to_rotate)]
        if motor_type == 'EPOS4':
            set_angle += [make_can_msg(node=node, index=0x6040, data=0x002F)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x003F)]
    else:
        set_angle += [make_can_msg(node=node, index=0x607A, data=qc_to_rotate)]
        if motor_type == 'EPOS4':
            set_angle += [make_can_msg(node=node, index=0x6040, data=0x006F)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x007F)]
    return set_angle


def init_device(node: int, mode: str = 'PPM', rpm: int = 5000):
    if not 1 < rpm < 50000:
        raise ValueError('RPM out of range')
    return [
        make_can_msg(node, 0x6040, 0, 0x0080),
        make_can_msg(node, 0x6060, 0, mode_epos.get(mode)),  # operation mode=profile position
        make_can_msg(node, 0x6081, 0, 1, rpm),  # rpm speed 1-25000 = 10_000 rpm
        make_can_msg(node, 0x6040, 0, EPOSCommand.SHUTDOWN),  # ????
        make_can_msg(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE),
        # make_can_msg(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE),
        # make_can_msg(node, 0x2078, 2, 0x3000)  # DO configuration
    ]
