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


def decoder_can(msg, extended=False):
    cobid_1, cobid_2, specifier, index, sub_index, data = struct.unpack('<2xBBBHBi1x', msg)
    cobid = struct.unpack('>H', bytearray([6, 5]))[0]
    return msg, cobid, specifier, index, sub_index, data


def make_can_frame(node, index, sub_index=0, data=0, write=True):
    if write:
        return bytearray(struct.pack('<2xBBBHBiB', 6, node, 0x22, index, sub_index, data, 0x08))
    else:
        return bytearray(struct.pack('<2xBBBHBiB', 6, node, 0x40, index, sub_index, data, 0x08))


def make_can_msg(node, index, sub_index=0, data=0, write=True):
    msg = make_can_frame(node, index, sub_index, data, write)
    _, cobid, specifier, index, sub_index, data = decoder_can(msg)
    return CAN(
        header=Header(stamp=Node.get_clock().now().to_msg()),
        is_extended=False,
        cobid=cobid,
        specifier=specifier,
        index=index,
        sub_index=sub_index,
        data=data,
        data_raw=msg
    )
