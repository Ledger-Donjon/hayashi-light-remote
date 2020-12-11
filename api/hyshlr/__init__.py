import serial
import serial.tools.list_ports

def assert_connected(f):
    def g(self, *args, **kwargs):
        if self.ser is None:
            raise RuntimeError("not connected")
        return f(self, *args, **kwargs)
    return g

class HyshLR:
    def __init__(self, dev=None):
        self.ser = None
        self.__lamp = None
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
                raise RuntimeError("multiple dongle found")
            elif len(possible_ports) == 1:
                dev = possible_ports[0].device
            else:
                raise RuntimeError("no dongle found");
        self.ser = serial.Serial(dev, 9600)
        self.lamp = False

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
    def power(self):
        """ Lamp power, from 0 to 1. """
        return self.__power_cache

    @power.setter
    def power(self, value: float):
        if (value < 0) or (value > 1):
            raise ValueError("power value out of range")
        frame = bytearray(b"\x03")
        value_code = int(value * 0b111111111111) << 2
        print('CODE', value_code)
        frame += value_code.to_bytes(2, 'big')
        self.ser.write(frame)
        res = self.ser.read(1)
        if res != b"\x03":
            raise RuntimeError("invalid response from dongle")
        self.__power_cache = value
