from enum import IntEnum

from src.utils.utils import make_can_msg

QC_FACTOR = 1  # 625000 / 360
DIGITAL_OUTPUT_3 = False
DIGITAL_OUTPUT_4 = False


class EPOSCommand(IntEnum):
    SHUTDOWN = 0b00000110
    SWITCH_ON = 0b00000111
    SWITCH_ON_AND_ENABLE = 0b00001111
    DISABLE_VOLTAGE = 0b00000000
    QUICK_STOP = 0b00000010
    DISABLE_OPERATION = 0b00000111
    ENABLE_OPERATION = 0b00001111
    FAULT_RESET = 0b10000000


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


def set_angle_value(node: int, angle: int, absolute=False, epos4=False):
    qc_to_rotate = int(QC_FACTOR * angle)
    set_angle = []
    if absolute:
        set_angle += [make_can_msg(node=node, index=0x607A, data=qc_to_rotate)]
        if epos4:
            set_angle += [make_can_msg(node=node, index=0x6040, data=0x002F)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x003F)]
    else:
        set_angle += [make_can_msg(node=node, index=0x607A, data=qc_to_rotate)]
        if epos4:
            set_angle += [make_can_msg(node=node, index=0x6040, data=0x006F)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x007F)]
    return set_angle


def enable_digital_4(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_4 = True

    if DIGITAL_OUTPUT_3:
        return [make_can_msg(node, 0x2078, 1, 0x3000)]
    else:
        return [make_can_msg(node, 0x2078, 1, 0x1000)]


def disable_digital_4(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_4 = False

    if DIGITAL_OUTPUT_3:
        return [make_can_msg(node, 0x2078, 1, 0x2000)]
    else:
        return [make_can_msg(node, 0x2078, 1, 0x0000)]


def enable_digital_3(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_3 = True

    if DIGITAL_OUTPUT_4:
        return [make_can_msg(node, 0x2078, 1, 0x3000)]
    else:
        return [make_can_msg(node, 0x2078, 1, 0x2000)]


def disable_digital_3(node: int):
    global DIGITAL_OUTPUT_4, DIGITAL_OUTPUT_3
    DIGITAL_OUTPUT_3 = False

    if DIGITAL_OUTPUT_4:
        return [make_can_msg(node, 0x2078, 1, 0x1000)]
    else:
        return [make_can_msg(node, 0x2078, 1, 0x0000)]


def init_device(node: int, rpm=0x1388):
    return [
        make_can_msg(node, 0x6040, 0, 0x0080),
        # make_can_msg(node, 0x6060, 0, 0x08),  # operation mode=Cyclic Synchronous Position Mode
        make_can_msg(node, 0x6060, 0, 0x01),  # operation mode=profile position
        make_can_msg(node, 0x6081, 0, rpm),  # rpm speed 1-25000 = 10_000 rpm
        make_can_msg(node, 0x6040, 0, EPOSCommand.SHUTDOWN),  # ????
        make_can_msg(node=node, index=0x6040, data=EPOSCommand.SWITCH_ON_AND_ENABLE),
        # make_can_msg(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE),
        # make_can_msg(node, 0x2078, 2, 0x3000)  # DO configuration
    ]
