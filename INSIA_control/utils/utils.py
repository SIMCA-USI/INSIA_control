from insia_msg.msg import CAN
from std_msgs.msg import Header
from rclpy.node import Node
import numpy as np
import struct
import can

# def make_can_frame(node, index, sub_index=0, data=0, write=True):
#     frame = bytearray(2)
#     protocol_payload = bytearray([6])
#     node_payload = bytearray([node])
#
#     if write:
#         mode_payload = bytearray([0x22])
#     else:
#         mode_payload = bytearray([0x40])
#
#     index_payload = bytearray([index & 0xFF, (index >> 8) & 0xFF])
#     sub_index_payload = bytearray([sub_index])
#     data_payload = bytearray([data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF])
#     payload = frame + protocol_payload + node_payload + mode_payload + index_payload + sub_index_payload + data_payload
#     payload.append(0x08)
#
#     return payload

import traceback


def decoder_can(msg: bytearray, extended=False):
    try:
        if extended:
            cobid, specifier = struct.unpack('>IB8x', msg)
            index, sub_index = struct.unpack('<5xHB5x', msg)
        else:
            cobid, specifier = struct.unpack('>2xHB8x', msg)
            index, sub_index = struct.unpack('<5xHB5x', msg)
        data_raw = bytearray(msg[8:-1])
    except Exception as e:
        print(f'Error decoding CAN {msg} {e} {len(msg)}')
        print(f'{traceback.format_exc()}')
        data_raw, cobid, specifier, index, sub_index = 0, 0, 0, 0, 0
    return msg, data_raw, cobid, specifier, index, sub_index

def decoder_libreria(msg: can.Message):
    try:
        data_raw= np.zeros(13, dtype=np.uint8)
        cobid = msg.arbitration_id
        specifier=0
        index=0
        sub_index=0
        data= np.zeros(4, dtype=np.uint8)

def decoder_libreria(msg: can.Message):
    try:
        data_raw = np.zeros(13, dtype=np.uint8)
        cobid = msg.arbitration_id
        specifier = 0
        index = 0
        sub_index = 0
        data = np.zeros(4, dtype=np.uint8)


        if len(msg.data) == 8:
            specifier, index, sub_index = struct.unpack('<BHB4x', msg.data)
            data = msg.data[4:]
            if msg.is_extended_id:
                cobid_values = [(cobid >> 24) & 0xFF, (cobid >> 16) & 0xFF, (cobid >> 8) & 0xFF, cobid & 0xFF]
                data_raw[:4] = cobid_values
                data_raw[4:12] = list(msg.data)
                data_raw[12] = 0
            else:
                cobid_values = [(cobid >> 8) & 0xFF, cobid & 0xFF]
                data_raw[:2] = 0
                data_raw[2:4] = cobid_values
                data_raw[4:12] = list(msg.data)
                data_raw[12] = 0
        else:
            if msg.is_extended_id:
                data_raw[:4] = msg.arbitration_id
            else:
                data_raw[:2] = 0
                data_raw[2:4] = msg.arbitration_id
            data_raw[4:len(msg.data)+4] = msg.data

    except Exception as e:
        pass
        # print(f'Error decoding CAN {msg} {e} {len(msg)}')
        # print(f'{traceback.format_exc()}')
        # msg, data_raw, cobid, specifier, index, sub_index, data = 0, 0, 0, 0, 0, 0, 0

    return msg, data_raw, cobid, specifier, index, sub_index, data


def make_can_frame(node, index, sub_index=0, data=0, write=True) -> bytearray:
    if write:
        return bytearray(struct.pack('<2xBBBHBiB', 6, node, 0x22, index, sub_index, data, 0x08))
    else:
        return bytearray(struct.pack('<2xBBBHBiB', 6, node, 0x40, index, sub_index, data, 0x08))


def make_can_msg(node, index=0x0000, sub_index=0, data=0, write=True, clock=None) -> CAN:
    msg = make_can_frame(node, index, sub_index, data, write)
    _, data_raw, cobid, specifier, index, sub_index = decoder_can(msg)
    return CAN(
        header=Header() if clock is None else Header(stamp=clock),
        # (stamp=Node('can_utils').get_clock().now().to_msg()),
        is_extended=False,
        cobid=cobid,
        specifier=specifier,
        index=index,
        sub_index=sub_index,
        data=data_raw,
        msg_raw=msg
    )


def convert_types(ros2_type: str, data):
    if 'int' in ros2_type:
        ros2_type = 'int'
    function = ros2_types.get(ros2_type)
    if function is not None:
        return function(data)
    else:
        return data


ros2_types = {
    'string': str,
    'int': int,
    'double': float
}
