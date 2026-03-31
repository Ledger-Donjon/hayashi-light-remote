import pytest
import time
from typing import Generator
from hyshlr.hyslr import HyshLR, NoDongleError, MultipleDongleError


@pytest.fixture(scope="module")
def hyshlr_real() -> Generator[HyshLR, None, None]:
    """
    Fixture to connect to a real Hayashi Light Remote dongle.

    Skips tests if no dongle is found or if multiple dongles are detected.
    """
    try:
        h = HyshLR()
        yield h
        h.disconnect()
    except NoDongleError:
        pytest.skip("No Hayashi Light Remote dongle found for real device tests.")
    except MultipleDongleError:
        pytest.skip(
            "Multiple Hayashi Light Remote dongles found; manual selection required."
        )


def test_real_connect(hyshlr_real: HyshLR):
    """Test that the real dongle connects and has sensible initial values."""
    h = hyshlr_real
    assert h.ser is not None


def test_lamp_state(hyshlr_real: HyshLR):
    """Test setting the lamp state on the real device."""
    h = hyshlr_real
    orig = h.lamp
    for led_value in [not orig, orig]:
        h.lamp = led_value
        time.sleep(1)
        assert h.lamp == led_value


def test_intensity(hyshlr_real: HyshLR):
    """Test setting the intensity on the real device."""
    h = hyshlr_real
    orig = h.intensity
    for intensity_value in range(0, 101, 10):
        h.intensity = intensity_value / 100
        time.sleep(0.5)
        assert abs(h.intensity - intensity_value / 100) < 0.01

    h.intensity = orig
    time.sleep(0.5)
    assert abs(h.intensity - orig) < 0.01
