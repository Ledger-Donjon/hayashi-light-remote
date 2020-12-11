import serial
import serial.tools.list_ports

class NoDongleError(Exception):
    pass

class MultipleDongleError(Exception):
    pass

def assert_connected(f):
    def g(self, *args, **kwargs):
        if self.ser is None:
            raise RuntimeError("not connected")
        return f(self, *args, **kwargs)
    return g

class HyshLR:
    def __init__(self, dev=None):
        """
        Connect to dongle, turn off the lamp by default and set intensity to
        25%.
        """
        self.ser = None
        self.__lamp = None
        self.__intensity_cache = None
        self.connect(dev)

    def connect(self, dev=None):
        """
        Connect to the dongle over serial.

        :param dev: Serial port device path. For instance "/dev/ttyUSB0" on
            linux, "COM0" on Windows. If None, tries to automatically find the
            board by scanning USB description strings.
        """
        if self.ser is not None:
            raise RuntimeError("already connected")
        if dev is None:
            possible_ports = []
            for port in serial.tools.list_ports.comports():
                if port.product == "hayashi-light-remote":
                    possible_ports.append(port)
            if len(possible_ports) > 1:
                raise MultipleDongleError()
            elif len(possible_ports) == 1:
                dev = possible_ports[0].device
            else:
                raise NoDongleError()
        self.ser = serial.Serial(dev, 9600)
        self.lamp = False
        self.intensity = 0.25

    @assert_connected
    def disconnect(self):
        """ Disconnect from the serial port. """
        self.__assert_connected()

    @property
    @assert_connected
    def lamp(self):
        """ Lamp state: True to turn On, False to turn Off. """
        return self.__lamp

    @lamp.setter
    @assert_connected
    def lamp(self, value: bool):
        if type(value) is not bool:
            raise ValueError("expected a bool")
        if value != self.__lamp:
            frame = bytearray(b"\x02")
            frame.append(int(value))
            self.ser.write(frame)
            res = self.ser.read(1)
            if res != b"\x02":
                raise RuntimeError("invalid response from dongle")
            self.__lamp = value

    @property
    def intensity(self):
        """ Lamp intensity, from 0 to 1. """
        return self.__intensity_cache

    @intensity.setter
    def intensity(self, value: float):
        if (value < 0) or (value > 1):
            raise ValueError("intensity value out of range")
        frame = bytearray(b"\x03")
        value_code = int(value * 0b111111111111) << 2
        print('CODE', value_code)
        frame += value_code.to_bytes(2, 'big')
        self.ser.write(frame)
        res = self.ser.read(1)
        if res != b"\x03":
            raise RuntimeError("invalid response from dongle")
        self.__intensity_cache = value
