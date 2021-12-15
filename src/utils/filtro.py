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
    def __init__(self, dictionary):
        self.dic_parameters = {}
        with open(os.getenv('ROS_WS') + '/src/INSIA_control/src/diccionarios/' + dictionary) as f:
            imiev_parameters = yaml.load(f, Loader=yaml.FullLoader)
        for i in imiev_parameters.keys(): #cobid
            for j in imiev_parameters[i].keys():#index
                if j == 0xFFFF:
                    self.dic_parameters.update({f'{i}:': Filter(imiev_parameters[i][j][0xFF])})
                else:
                    for k in imiev_parameters[i][j].keys():
                        if k == 0xFF:
                            self.dic_parameters.update({f'{i}:{j}': Filter(imiev_parameters[i][j][k])})
                        else:
                            self.dic_parameters.update({f'{i}:{j}:{k}': Filter(imiev_parameters[i][j][k])})

    def decode(self, msg):
        if f'{msg.cobid}:{msg.index}:{msg.sub_index}' in self.dic_parameters.keys():
            return self.dic_parameters.get(f'{msg.cobid}:{msg.index}:{msg.sub_index}').decoder(msg.data)
        elif f'{msg.cobid}:{msg.index}' in self.dic_parameters.keys():
            return self.dic_parameters.get(f'{msg.cobid}:{msg.index}').decoder(msg.data)
        elif f'{msg.cobid}:' in self.dic_parameters.keys():
            return self.dic_parameters.get(f'{msg.cobid}:').decoder(msg.msg_raw)
        else:
            raise ValueError(f'msg no declarado: {msg.cobid}:{msg.index}:{msg.sub_index}')


class Filter:
    def __init__(self, parameters):
        parameters = parameters
        self.name = parameters['name']
        self.inicio = parameters['inicio']
        if self.inicio < 0:
            self.inicio = self.inicio + 8
        self.longitud = parameters['longitud']
        self.factor = parameters['factor']
        self.offset = parameters['offset']
        self.signed = parameters['signed']
        self.type = parameters['type']

    def decoder(self, data):
        deco = dic_byte_order[self.type] + dic_format[(self.signed, self.longitud)]
        value = struct.unpack(deco, data[self.inicio:int(self.inicio + self.longitud / 8)])[
                    0] * self.factor + self.offset
        return self.name, value


def main(args=None):
    try:
        Decoder()
    except KeyboardInterrupt:
        print('CAN: Keyboard interrupt')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
