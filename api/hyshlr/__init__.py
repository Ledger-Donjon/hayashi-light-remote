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
        self.__lamp_cache = None
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

    @assert_connected
    def disconnect(self):
        """ Disconnect from the serial port. """
        self.ser.close()
        self.ser = None
        self.__lamp_cache = None
        self.__intensity_cache = None

    @property
    @assert_connected
    def lamp(self):
        """ Lamp state: True to turn On, False to turn Off. """
        if self.__lamp_cache is None:
            # Query from device
            self.ser.write(b"\x04")
            res = self.ser.read(2)
            assert res[0] == 0x04
            assert res[1] in (0, 1)
            self.__lamp_cache = res[1]
        return self.__lamp_cache

    @lamp.setter
    @assert_connected
    def lamp(self, value: bool):
        if type(value) is not bool:
            raise ValueError("expected a bool")
        if value != self.__lamp_cache:
            frame = bytearray(b"\x02")
            frame.append(int(value))
            self.ser.write(frame)
            res = self.ser.read(1)
            if res != b"\x02":
                raise RuntimeError("invalid response from dongle")
            self.__lamp_cache = bool(value)

    @property
    def intensity(self):
        """ Lamp intensity, from 0 to 1. """
        if self.__intensity_cache is None:
            # Query from device
            self.ser.write(b"\x05")
            res = self.ser.read(3)
            assert res[0] == 0x05
            value = int.from_bytes(res[1:], "big", signed=False)
            value = (value >> 2) / 0b111111111111
            assert (value >= 0) and (value <= 1)
            self.__intensity_cache = value
        return self.__intensity_cache

    @intensity.setter
    def intensity(self, value: float):
        if (value < 0) or (value > 1):
            raise ValueError("intensity value out of range")
        frame = bytearray(b"\x03")
        value_code = int(value * 0b111111111111) << 2
        frame += value_code.to_bytes(2, 'big')
        self.ser.write(frame)
        res = self.ser.read(1)
        if res != b"\x03":
            raise RuntimeError("invalid response from dongle")
        self.__intensity_cache = value

    @property
    def burnout(self) -> bool:
        self.ser.write(b"\x06")
        res = self.ser.read(2)
        assert res[0] == 0x06
        assert res[1] in range(2)
        return bool(res[1])
