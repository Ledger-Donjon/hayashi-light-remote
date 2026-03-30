from __future__ import annotations

import serial
from serial.tools.list_ports import comports
from serial.tools.list_ports_common import ListPortInfo
from .errors import NoDongleError, MultipleDongleError


class HyshLR:
    def __init__(self, dev: str | None = None) -> None:
        """
        Connect to dongle.
        """
        self.ser: serial.Serial | None = None
        self.__lamp_cache: bool | None = None
        self.__intensity_cache: float | None = None
        self.connect(dev)

    def connect(self, dev: str | None = None) -> None:
        """
        Connect to the dongle over serial.

        :param dev: Serial port device path. For instance "/dev/ttyUSB0" on
            linux, "COM0" on Windows. If None, tries to automatically find the
            board by scanning USB description strings.
        """
        if self.ser is not None:
            raise RuntimeError("already connected")
        if dev is None:
            possible_ports: list[ListPortInfo] = []
            for port in comports():
                print(port.device, port.product)
                if port.product == "hayashi-light-remote":
                    possible_ports.append(port)
            if len(possible_ports) > 1:
                raise MultipleDongleError()
            elif len(possible_ports) == 1:
                dev = possible_ports[0].device
            else:
                raise NoDongleError()
        self.ser = serial.Serial(dev, 9600)

    def disconnect(self):
        """Disconnect from the serial port."""
        if self.ser is not None:
            self.ser.close()
            self.ser = None
        self.__lamp_cache = None
        self.__intensity_cache = None

    @property
    def lamp(self) -> bool:
        """Lamp state: True to turn On, False to turn Off."""
        if self.ser is None:
            raise RuntimeError("not connected")
        if self.__lamp_cache is None:
            # Query from device
            self.ser.write(b"\x04")
            res = self.ser.read(2)
            assert len(res) == 2
            assert res[0] == 0x04
            assert res[1] in (0, 1)
            self.__lamp_cache = bool(res[1])
        return self.__lamp_cache

    @lamp.setter
    def lamp(self, value: bool):
        if self.ser is None:
            raise RuntimeError("not connected")
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
    def intensity(self) -> float:
        """Lamp intensity, from 0 to 1."""
        if self.ser is None:
            raise RuntimeError("not connected")
        if self.__intensity_cache is None:
            # Query from device
            self.ser.write(b"\x05")
            res = self.ser.read(3)
            assert res[0] == 0x05
            value_int = int.from_bytes(res[1:], "big", signed=False)
            value = (value_int >> 2) / 0b111111111111
            assert (value >= 0) and (value <= 1)
            self.__intensity_cache = value
        return self.__intensity_cache

    @intensity.setter
    def intensity(self, value: float):
        if self.ser is None:
            raise RuntimeError("not connected")
        if (value < 0) or (value > 1):
            raise ValueError("intensity value out of range")
        frame = bytearray(b"\x03")
        value_code = int(value * 0b111111111111) << 2
        frame += value_code.to_bytes(2, "big")
        self.ser.write(frame)
        res = self.ser.read(1)
        if res != b"\x03":
            raise RuntimeError("invalid response from dongle")
        self.__intensity_cache = value

    @property
    def burnout(self) -> bool:
        """Burnout state: True if the lamp is burned out, False otherwise."""
        if self.ser is None:
            raise RuntimeError("not connected")
        self.ser.write(b"\x06")
        res = self.ser.read(2)
        assert res[0] == 0x06
        assert res[1] in range(2)
        return bool(res[1])
