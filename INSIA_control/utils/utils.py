from insia_msg.msg import CAN
from std_msgs.msg import Header
from rclpy.node import Node

import struct


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
        print(f'Error decoding CAN {msg}')
        data_raw, cobid, specifier, index, sub_index = 0, 0, 0, 0, 0
    return msg, data_raw, cobid, specifier, index, sub_index


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


def convert_types(ros2_type, data):
    function = ros2_types.get(ros2_type)
    if function is not None:
        return function(data)
    else:
        return data


ros2_types = {
    'string': str,
    'int32': int,
    'double': float
}
