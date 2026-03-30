import pytest
import time

from hyshlr.hyslr import HyshLR, NoDongleError, MultipleDongleError


@pytest.fixture(scope="module")
def hyshlr_real():
    """
    Fixture to connect to a real Hayashi Light Remote dongle.

    Skips tests if no dongle is found or if multiple dongles are detected.
    """
    try:
        h = HyshLR()
        h.connect()
        yield h
        h.disconnect()
    except NoDongleError:
        pytest.skip("No Hayashi Light Remote dongle found for real device tests.")
    except MultipleDongleError:
        pytest.skip(
            "Multiple Hayashi Light Remote dongles found; manual selection required."
        )


def test_real_connect(hyshlr_real):
    """Test that the real dongle connects and has sensible initial values."""
    h = hyshlr_real
    assert h.ser is not None
    assert hasattr(h, "firmware_version")


def test_real_get_state(hyshlr_real):
    """Test getting state from the real dongle."""
    state = hyshlr_real.state
    assert isinstance(state, dict)
    assert "relay" in state
    assert "led" in state


def test_real_toggle_relay(hyshlr_real):
    """Test toggling relay on the real dongle."""
    h = hyshlr_real
    orig_state = h.relay
    # Toggle relay
    h.relay = not orig_state
    time.sleep(0.1)  # give hardware a moment
    assert h.relay == (not orig_state)
    # Restore original state
    h.relay = orig_state
    time.sleep(0.1)
    assert h.relay == orig_state


def test_real_set_led(hyshlr_real):
    """Test setting the LED on the real device."""
    h = hyshlr_real
    orig = h.led
    for led_value in [not orig, orig]:
        h.led = led_value
        time.sleep(0.1)
        assert h.led == led_value


def test_real_firmware_version(hyshlr_real):
    """Test that firmware version can be read and matches expected pattern."""
    v = hyshlr_real.firmware_version
    assert isinstance(v, str)
    assert len(v) > 0
