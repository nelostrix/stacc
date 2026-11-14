"""Control-boundary regression tests. All device and SSH calls are mocked."""
import math
import shlex
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from stacc.hal.gpio import GPIO
from stacc.hal.can import CANBus, CANFrame, UnitreeMotorCAN
from stacc.hal.boards import FirmwareUploader
from stacc.hal.deploy import Deployer
from stacc.control.limiter import SCurveProfile


def test_real_gpio_requires_an_implemented_provider():
    for backend in (None, "gpiod"):
        with patch.object(GPIO, "_detect_backend", return_value=backend):
            with pytest.raises(RuntimeError, match="No supported real GPIO"):
                GPIO()
    gpio = GPIO(simulation=True)
    gpio.simulation = False
    with pytest.raises(RuntimeError):
        gpio.write(18, 1)
    assert gpio._pin_states == {}


@pytest.mark.parametrize("backend,module", [("rpigpio", "RPi.GPIO"), ("jetson", "Jetson.GPIO")])
def test_gpio_dispatches_io_and_pwm_to_selected_provider(backend, module):
    driver = MagicMock()
    driver.input.return_value = 1
    with patch.object(GPIO, "_detect_backend", return_value=backend), patch(
        "stacc.hal.gpio.importlib.import_module", return_value=driver
    ) as load:
        gpio = GPIO()
        gpio.setup(18, GPIO.OUTPUT)
        gpio.write(18, 1)
        assert gpio.read(18) == 1
        pwm = gpio.pwm_start(18, 1000, 25)
        gpio.cleanup()
    assert all(call.args == (module,) for call in load.call_args_list)
    driver.output.assert_called_once_with(18, 1)
    driver.PWM.assert_called_once_with(18, 1000)
    pwm.start.assert_called_once_with(25)
    driver.cleanup.assert_called_once()


def test_explicit_gpio_simulation_and_failed_write_do_not_fake_real_io():
    gpio = GPIO(simulation=True)
    gpio.setup(18, GPIO.OUTPUT)
    gpio.write(18, 1)
    assert gpio.read(18) == 1
    assert gpio.pwm_start(18) is None
    driver = MagicMock()
    driver.output.side_effect = OSError("device lost")
    gpio.simulation = False
    gpio._backend = "jetson"
    with patch("stacc.hal.gpio.importlib.import_module", return_value=driver):
        with pytest.raises(OSError):
            gpio.write(18, 0)
    assert gpio._pin_states[18] == 1


def test_real_can_bind_failure_is_not_virtual_success():
    with patch("socket.socket", side_effect=OSError("no interface")):
        with pytest.raises(ConnectionError, match="is_virtual=True"):
            CANBus()


def test_can_rejects_lossy_or_unverified_packets():
    bus = CANBus(is_virtual=True)
    with pytest.raises(ValueError, match="payload"):
        CANFrame(1, b"x" * 9)
    with pytest.raises(ValueError, match="arbitration"):
        CANFrame(0x800, b"")
    with pytest.raises(NotImplementedError, match="model-specific"):
        UnitreeMotorCAN(1, bus).send_hybrid_control(0, 0, 1, 1, 2)
    assert bus.recv() is None
    frame = CANFrame(1, b"abc")
    assert bus.send(frame)
    assert bus.recv() is frame


def test_firmware_metacharacters_are_literal_subprocess_arguments():
    uploader = FirmwareUploader("arduino_uno", port="/dev/serial port; literal")
    path = "/tmp/firmware name; $(literal).hex"
    args = uploader.get_upload_args(path)
    assert args[-1] == f"flash:w:{path}:i"
    assert shlex.split(uploader.get_upload_command(path)) == args
    with patch("subprocess.run", return_value=SimpleNamespace(returncode=0)) as run:
        assert uploader.upload(path)
    assert run.call_args.args == (args,)
    assert run.call_args.kwargs["shell"] is False


def test_manual_upload_and_missing_esp_layout_cannot_report_success():
    uploader = FirmwareUploader("raspberry_pi_pico")
    with patch("subprocess.run") as run:
        assert not uploader.upload("missing.uf2")
        run.assert_not_called()
    esp = FirmwareUploader("esp32_s3")
    with pytest.raises(ValueError, match="flash_address"):
        esp.get_upload_args("combined.bin")
    esp = FirmwareUploader("esp32_s3", flash_address=0)
    assert esp.get_upload_args("combined.bin")[2] == "esp32s3"


def test_ssh_uses_trusted_hosts_and_cleans_up_rejected_connection():
    client = MagicMock()
    paramiko = SimpleNamespace(SSHClient=lambda: client, RejectPolicy=type("RejectPolicy", (), {}))
    with patch.dict("sys.modules", {"paramiko": paramiko}):
        deployer = Deployer("robot.invalid", known_hosts="/tmp/enrolled-hosts")
        assert deployer.connect()
        client.load_system_host_keys.assert_called_once()
        client.load_host_keys.assert_called_once_with("/tmp/enrolled-hosts")
        assert isinstance(client.set_missing_host_key_policy.call_args.args[0], paramiko.RejectPolicy)
        client.connect.side_effect = OSError("host key mismatch")
        assert not deployer.connect()
        assert deployer._ssh is None
        with pytest.raises(ConnectionError):
            with Deployer("robot.invalid"):
                pytest.fail("body must not execute on rejected SSH connection")


@pytest.mark.parametrize("distance,vmax,amax,jmax", [
    (10, .01, 1, 10), (-10, .2, 1, 10), (.00001, 1, 1, 10),
    (1, 10, 1, 10), (10, 1, 2, 10), (-.1, .03, .2, .7),
])
def test_scurve_bounds_monotonicity_continuity_and_endpoint(distance, vmax, amax, jmax):
    profile = SCurveProfile(vmax, amax, jmax)
    profile.reset(2)
    profile.set_target(2 + distance)
    dt = profile._total_time / 2000
    previous = profile.position
    previous_acceleration = 0
    previous_velocity = 0
    for _ in range(2002):
        position = profile.compute(dt)
        assert abs(profile.velocity) <= vmax + 1e-10
        assert abs(profile.acceleration) <= amax + 1e-10
        assert abs(profile.acceleration - previous_acceleration) <= jmax * dt + 1e-10
        assert abs(profile.velocity - previous_velocity) <= amax * dt + 1e-10
        assert (position - previous) * distance >= -1e-10
        assert abs(position - previous) <= vmax * dt + 1e-9
        previous, previous_acceleration, previous_velocity = position, profile.acceleration, profile.velocity
    assert profile.is_idle
    assert profile.position == pytest.approx(2 + distance)
    assert profile.velocity == profile.acceleration == 0


def test_scurve_rejects_invalid_limits_and_moving_retarget():
    for value in (0, -1, math.nan, math.inf):
        with pytest.raises(ValueError):
            SCurveProfile(max_velocity=value)
    profile = SCurveProfile()
    profile.set_target(0)
    assert profile.is_idle
    profile.set_target(1)
    profile.compute(.01)
    with pytest.raises(RuntimeError, match="retargeting"):
        profile.set_target(2)
    with pytest.raises(ValueError):
        profile.compute(math.nan)
    profile.compute(100)
    profile.set_target(-1)
    profile.compute(100)
    assert profile.position == -1


def test_board_selection_never_downgrades_specific_or_unknown_models():
    from stacc.hal.boards import get_board
    assert get_board("esp32_s3").name == "ESP32-S3"
    assert get_board("ESP32-S3").name == "ESP32-S3"
    assert get_board("ESP32-Unknown") is None
    assert get_board("") is None
    assert get_board("Arduino") is not None  # explicit catalog platform alias
