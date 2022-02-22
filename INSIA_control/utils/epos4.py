from enum import IntEnum, Enum
from typing import Optional

from INSIA_control.utils.utils import make_can_msg
import networkx as nx


class EPOSCommand(IntEnum):
    SHUTDOWN = 0x06
    SWITCH_ON = 0x07
    SWITCH_ON_AND_ENABLE = 0x0F
    DISABLE_VOLTAGE = 0x00
    QUICK_STOP = 0x02
    DISABLE_OPERATION = 0x07
    ENABLE_OPERATION = 0x0F
    FAULT_RESET = 0x80


class EPOSStatus(str, Enum):
    Not_ready_to_switch_on = 'Not ready to switch on'
    Switch_on_disabled = 'Switch on disabled'
    Ready_to_switch_on = 'Ready to switch on'
    Switched_on = 'Switched on'
    Operation_enabled = 'Operation enabled'
    Quick_stop_active = 'Quick stop active'
    Fault_reaction_active = 'Fault reaction active'
    Fault = 'Fault'


status_epos = {
    0x00: EPOSStatus.Not_ready_to_switch_on,
    0x40: EPOSStatus.Switch_on_disabled,
    0x21: EPOSStatus.Ready_to_switch_on,
    0x23: EPOSStatus.Switched_on,
    0x27: EPOSStatus.Operation_enabled,
    0x07: EPOSStatus.Quick_stop_active,
    0x0F: EPOSStatus.Fault_reaction_active,
    0x08: EPOSStatus.Fault
}
mode_epos = {
    'PPM': 1,
    'PVM': 3,
    'HMM': 6,
    'CSP': 8,
    'CSV': 9,
    'CST': 10
}

transitions = {
    1: EPOSCommand.DISABLE_VOLTAGE,
    2: EPOSCommand.SHUTDOWN,
    3: EPOSCommand.SWITCH_ON,
    4: EPOSCommand.SWITCH_ON_AND_ENABLE,
    5: EPOSCommand.DISABLE_OPERATION,
    6: EPOSCommand.SHUTDOWN,
    7: EPOSCommand.DISABLE_VOLTAGE,
    8: EPOSCommand.SHUTDOWN,
    9: EPOSCommand.DISABLE_VOLTAGE,
    10: EPOSCommand.DISABLE_VOLTAGE,
    11: EPOSCommand.QUICK_STOP,
    12: EPOSCommand.DISABLE_VOLTAGE,
    14: EPOSCommand.FAULT_RESET,
    15: EPOSCommand.FAULT_RESET,
}

control_word_dict = {
    'Switch on disabled': EPOSCommand.DISABLE_VOLTAGE,
    'Ready to switch on': EPOSCommand.SHUTDOWN,
    'Switched on': EPOSCommand.SWITCH_ON,
    'Operation enabled': EPOSCommand.SWITCH_ON_AND_ENABLE,
    'Quick stop active': EPOSCommand.QUICK_STOP,
}


def set_state(node: int, target_state, graph: nx.classes.digraph = None, status_word: int = None):
    if target_state in status_epos.values():
        if graph is not None and status_word is not None:
            if get_status(status_word) != target_state:
                msgs = []
                for transition in get_transitions(graph=graph, start=get_status(status_word), end=target_state):
                    msgs.append(make_can_msg(node=node, index=0x6040, data=transitions.get(transition)))
                return msgs
            else:
                return None
        else:
            msgs = []
            for transition in get_transitions(graph=graph, start=EPOSStatus.Fault, end=target_state):
                msgs.append(make_can_msg(node=node, index=0x6040, data=transitions.get(transition)))
            return msgs
    else:
        raise ValueError(f'Target state not valid: {target_state}')


def configuration(node: int, graph: nx.classes.digraph, status_word: int):
    msgs = []
    for transition in get_transitions(graph=graph, start=get_status(status_word), end='Ready to switch on'):
        msgs.append(make_can_msg(node=node, index=0x6040, data=transitions.get(transition)))
    return msgs


def set_digital(node: int, outputs: dict = None):
    if outputs is None:
        outputs = {1: False, 2: False}
    if not outputs.keys() >= {1, 2}:
        raise ValueError(f'Digital outputs keys missing {outputs.keys()} , required 1,2')
    out = (2 if outputs.get(2) else 0) + (1 if outputs.get(1) else 0)
    return [make_can_msg(node=node, index=0x3150, sub_index=0x02, data=out)]


def set_analog(node: int, analog_out: int, voltage: float):
    print(f'Set analog {analog_out} v {int(voltage * 1000)}')
    return [make_can_msg(node=node, index=0x3182, sub_index=analog_out, data=int(voltage * 1000))]


def read_status(node: int):
    return [make_can_msg(node=node, index=0x6041, write=False)]


def read_io(node: int):
    return [make_can_msg(node=node, index=0x2078, sub_index=1, write=False)]


def read_position(node: int):
    return [make_can_msg(node=node, index=0x6064, write=False)]


def get_status(status_word: int) -> Optional[str]:
    if status_word is not None:
        return status_epos.get(status_word & 0x006F)
    else:
        return None


def get_status_from_dict(dictionary: dict) -> str:
    return get_status(dictionary.get('Statusword'))


def get_status_hex(status_word: int) -> int:
    return status_word & 0x006F


def reset_position(node: int, position: int = 0, prev_mode: str = 'PPM', status_word=0x23):
    control_word = control_word_dict.get(get_status(status_word))
    if control_word is None:
        raise ValueError(f'Error getting control word from status: {hex(status_word)}')

    return [make_can_msg(node, 0x6060, 0, mode_epos.get('HMM')),  # operation mode=Homing mode
            make_can_msg(node, 0x6098, 0, 37),  # homing method = Actual position
            make_can_msg(node, 0x30B0, 0, position),  # set position 0
            make_can_msg(node, 0x6040, 0, control_word & 0xEF),  # set position 0
            make_can_msg(node, 0x6040, 0, (control_word & 0xEF) + 0x10),  # set position 0
            make_can_msg(node, 0x6060, 0, mode_epos.get(prev_mode)),  # operation mode=prev_mode default=PPM
            ]


def fault_reset(node: int):
    return [make_can_msg(node=node, index=0x6040, data=EPOSCommand.FAULT_RESET),
            make_can_msg(node, 0x6040, 0, EPOSCommand.SHUTDOWN),
            make_can_msg(node, 0x6040, 0, EPOSCommand.SWITCH_ON_AND_ENABLE), ]


def set_angle_value(node: int, angle: int, absolute: bool = False):
    set_angle = []
    if absolute:
        set_angle += [make_can_msg(node=node, index=0x607A, data=angle)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x002F)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x003F)]
    else:
        set_angle += [make_can_msg(node=node, index=0x607A, data=angle)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x006F)]
        set_angle += [make_can_msg(node=node, index=0x6040, data=0x007F)]
    return set_angle


def set_torque(node: int, torque: int):
    return [make_can_msg(node=node, index=0x6071, sub_index=0, data=int(torque))]


def init_device(node: int, mode: str = 'PPM', rpm: int = 5000):
    if not 1 < rpm < 50000:
        raise ValueError('RPM out of range')
    return [
        make_can_msg(node, 0x6040, 0, EPOSCommand.FAULT_RESET),
        make_can_msg(node, 0x6060, 0, mode_epos.get(mode)),  # operation mode
        make_can_msg(node, 0x6081, 0, rpm),  # rpm speed 1-25000 = 10_000 rpm
    ]


def get_transitions(graph, start, end):
    edge_labels = nx.get_edge_attributes(graph, 'transition')
    path = nx.shortest_path(graph)
    path_edges = [edge_labels.get(x, edge_labels.get((x[1], x[0]))) for x in
                  zip(path[start][end], path[start][end][1:])]
    return path_edges
