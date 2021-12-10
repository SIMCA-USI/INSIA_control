import struct
import yaml


dic_byte_order = {
    'intel': '<',
    'little_endian': '<',
    'motorola': '>',
    'big_endian': '>'
}

dic_format = {
    (True, 1): 'b',
    (False, 1): 'B',
    (True, 2): 'h',
    (False, 2): 'H',
    (True, 4): 'i',
    (False, 4): 'I'
}


class Decoder:
    def __init__(self):
        dic_parameters = {}
        data = b'\x00\x10\x10\x02'
        with open('/home/victor/Escritorio/ASCOD/INSIA_control/src/diccionarios/imiev_dictionary.yaml') as f:
            imiev_parameters = yaml.load(f, Loader=yaml.FullLoader)
        for i in imiev_parameters.keys():
            for j in imiev_parameters[i].keys():
                dic_parameters.update({f'{hex(i)}:{hex(j)}': Filter(imiev_parameters[i][j])})

        dic_parameters.get('0x306:0x0').decoder(data)


class Filter:
    def __init__(self, parameters):
        parameters = parameters
        self.name = parameters['name']
        self.inicio = parameters['inicio']
        self.longitud = parameters['longitud'] / 8
        self.factor = parameters['factor']
        self.offset = parameters['offset']
        self.signed = parameters['signed']
        self.type = parameters['type']

    def decoder(self, data):
        deco = dic_byte_order[self.type]+dic_format[(self.signed, self.longitud)]
        value = struct.unpack(deco, data)[0] * self.factor + self.offset
        print(value)


def main(args=None):
    try:
        Decoder()
    except KeyboardInterrupt:
        print('CAN: Keyboard interrupt')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
