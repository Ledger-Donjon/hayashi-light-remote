import pytest
from hyshlr import HyshLR, NoDongleError, MultipleDongleError

import types


class DummySerial:
    def __init__(self):
        self.is_closed = False
        self.writes = []
        self.read_data = []
        # Two caches for lamp query and intensity
        self.lamp_value = 0
        self.intensity_value = 0

    def write(self, data):
        self.writes.append(bytes(data))

    def read(self, n):
        # Simulate responses depending on the last command written
        # Common commands:
        #   \x04 - query lamp
        #   \x05 - query intensity
        #   \x02 - set lamp
        #   \x03 - set intensity
        #   \x06 - query burnout
        if not self.writes:
            return b"\x00" * n
        last_cmd = self.writes[-1][:1]
        if last_cmd == b"\x04":
            return bytes([0x04, self.lamp_value])
        if last_cmd == b"\x05":
            # Simulate 12-bit value encoded in 2 bytes (with topmost bits 0)
            value = int(self.intensity_value * 0b111111111111) << 2
            b1 = (value >> 8) & 0xFF
            b2 = value & 0xFF
            return bytes([0x05, b1, b2])
        if last_cmd == b"\x02":
            return b"\x02"
        if last_cmd == b"\x03":
            return b"\x03"
        if last_cmd == b"\x06":
            return bytes([0x06, 0])
        return b"\x00" * n

    def close(self):
        self.is_closed = True


@pytest.fixture
def hyshlr_with_dummy_serial(monkeypatch):
    h = HyshLR.__new__(HyshLR)
    dummy = DummySerial()
    h.ser = dummy
    h._HyshLR__lamp_cache = None
    h._HyshLR__intensity_cache = None
    return h, dummy


def test_lamp_on_off(hyshlr_with_dummy_serial):
    h, dummy = hyshlr_with_dummy_serial

    # Simulate lamp is off at beginning
    dummy.lamp_value = 0
    assert h.lamp == 0
    # Set lamp to True
    h.lamp = True
    assert dummy.writes[-1][:1] == b"\x02"
    assert h.lamp is True
    # Set lamp to False
    dummy.lamp_value = 1  # if the query is done again, simulate it
    h.lamp = False
    assert dummy.writes[-1][:1] == b"\x02"
    assert h.lamp is False


def test_lamp_setter_raises_on_non_bool(hyshlr_with_dummy_serial):
    h, _ = hyshlr_with_dummy_serial
    with pytest.raises(ValueError):
        h.lamp = "on"


def test_intensity_setter_and_getter(hyshlr_with_dummy_serial):
    h, dummy = hyshlr_with_dummy_serial

    dummy.intensity_value = 0.25
    assert abs(h.intensity - 0.25) < 0.01

    h.intensity = 0.75
    assert dummy.writes[-1][:1] == b"\x03"
    assert abs(h._HyshLR__intensity_cache - 0.75) < 0.01


def test_intensity_setter_out_of_range(hyshlr_with_dummy_serial):
    h, _ = hyshlr_with_dummy_serial
    with pytest.raises(ValueError):
        h.intensity = -0.1
    with pytest.raises(ValueError):
        h.intensity = 1.1


def test_disconnect(hyshlr_with_dummy_serial):
    h, dummy = hyshlr_with_dummy_serial
    h.disconnect()
    assert dummy.is_closed
    assert h.ser is None


def test_burnout_property(hyshlr_with_dummy_serial):
    h, dummy = hyshlr_with_dummy_serial
    assert h.burnout is False

    # Simulate burnout
    def read_burnout(n):
        return bytes([0x06, 1])

    dummy.read = read_burnout
    assert h.burnout is True


def test_connect_raises_already_connected(monkeypatch):
    h = HyshLR.__new__(HyshLR)
    h.ser = object()
    with pytest.raises(RuntimeError):
        h.connect("/dev/notreal")


def test_connect_no_dongle(monkeypatch):
    import hyshlr.hyslr

    # Patch serial.tools.list_ports.comports to empty
    monkeypatch.setattr(hyshlr.hyslr.serial.tools.list_ports, "comports", lambda: [])
    h = HyshLR.__new__(HyshLR)
    h.ser = None
    with pytest.raises(NoDongleError):
        h.connect(dev=None)


def test_connect_multiple_dongle(monkeypatch):
    import types, hyshlr.hyslr

    class Port:
        def __init__(self, device, product):
            self.device = device
            self.product = product

    monkeypatch.setattr(
        hyshlr.hyslr.serial.tools.list_ports,
        "comports",
        lambda: [
            Port("/dev/a", "hayashi-light-remote"),
            Port("/dev/b", "hayashi-light-remote"),
        ],
    )
    h = HyshLR.__new__(HyshLR)
    h.ser = None
    with pytest.raises(MultipleDongleError):
        h.connect(dev=None)
