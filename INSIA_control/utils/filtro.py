import os
import struct
import yaml

dic_byte_order = {
    'intel': '<',
    'little_endian': '<',
    'motorola': '>',
    'big_endian': '>'
}

dic_format = {
    (True, 8): 'b',
    (False, 8): 'B',
    (True, 16): 'h',
    (False, 16): 'H',
    (True, 32): 'i',
    (False, 32): 'I'
}


class Decoder:
    def __init__(self, dictionary, cobid=0):
        self.dic_parameters = {}
        with open(os.getenv('ROS_WS') + '/src/INSIA_control/INSIA_control/diccionarios/' + dictionary) as f:
            imiev_parameters = yaml.load(f, Loader=yaml.FullLoader)
        for i in imiev_parameters.keys():  # cobid
            for j in imiev_parameters[i].keys():  # index
                if j == 0xFFFF:
                    self.dic_parameters.update({f'{i + cobid}': Filter(imiev_parameters[i][j][0xFF])})
                else:
                    for k in imiev_parameters[i][j].keys():
                        if k == 0xFF:
                            self.dic_parameters.update({f'{i + cobid}:{j}': Filter(imiev_parameters[i][j][k])})
                        else:
                            self.dic_parameters.update({f'{i + cobid}:{j}:{k}': Filter(imiev_parameters[i][j][k])})

    def decode(self, msg):
        if f'{msg.cobid}:{msg.index}:{msg.sub_index}' in self.dic_parameters.keys():
            return self.dic_parameters.get(f'{msg.cobid}:{msg.index}:{msg.sub_index}').decoder(msg)
        elif f'{msg.cobid}:{msg.index}' in self.dic_parameters.keys():
            return self.dic_parameters.get(f'{msg.cobid}:{msg.index}').decoder(msg)
        elif f'{msg.cobid}' in self.dic_parameters.keys():
            return self.dic_parameters.get(f'{msg.cobid}').decoder(msg)
        else:
            raise ValueError(f'msg no declarado: {msg.cobid}:{msg.index}:{msg.sub_index}')


class Filter:
    def __init__(self, parameters):
        parameters = parameters
        self.name = parameters['name']
        self.inicio = parameters['inicio']
        self.longitud = parameters['longitud']
        self.factor = parameters['factor']
        self.offset = parameters['offset']
        self.signed = parameters['signed']
        self.type = parameters['type']
        self.can_open = parameters['can_open']
        if 'mask' in parameters:
            self.mask = parameters['mask']
        else:
            self.mask=None

    def decoder(self, data):
        if self.can_open:
            data = data.data
        else:
            data = data.msg_raw[4:]
        deco = dic_byte_order[self.type] + dic_format[(self.signed, self.longitud)]
        value = struct.unpack(deco, data[self.inicio:int(self.inicio + self.longitud / 8)])[
                    0]
        try:
            if self.mask is not None:
                value = value & self.mask
        except Exception as e:
            print(f'Error aplying mask {e}')
        value =  value * self.factor + self.offset
        return self.name, value


def main(args=None):
    try:
        Decoder(dictionary='../epos4_dictionary.yaml')
    except KeyboardInterrupt:
        print('CAN: Keyboard interrupt')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
