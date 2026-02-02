from enum import IntEnum, Enum
from typing import Optional

from INSIA_control.utils.utils import make_can_msg
import networkx as nx


def set_angle_value(node: int, angle: int, absolute: bool = False):
    set_angle = []
    set_angle += [make_can_msg(node=node, index=0x607A, data=angle)]
    set_angle += [make_can_msg(node=node, index=0x6040, data=0x000F)]
    if absolute:
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x003F)]
    else:
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x007F)]
    return set_angle


def init_device(node: int):
    return [
        make_can_msg(node, 0x6040, 0, 0),
        make_can_msg(node, 0x6040, 0, 6),
        make_can_msg(node, 0x6040, 0, 0xF),
        make_can_msg(node, 0x6060, 0, 0x01),
    ]