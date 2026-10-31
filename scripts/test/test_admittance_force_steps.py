"""Offline tests: no ROS nodes, services, sleeps, or motor commands are used."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "admittance_force_steps.py"
SPEC = importlib.util.spec_from_file_location("admittance_force_steps", SCRIPT_PATH)
steps = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(steps)


@pytest.mark.parametrize("axis", ("fy", "fz"))
def test_defaults_and_millinewton_to_newton_conversion(axis):
    args = steps.build_parser().parse_args(["--axis", axis])
    assert args.axis == axis
    assert args.interval == 3.0
    assert args.sign == 1
    assert args.node == "/robot_control"
    assert args.timeout == 3.0
    assert not args.dry_run
    assert steps.validate_args(args) == pytest.approx(
        (0.0, 0.2, 0.3, 0.4, 0.5, 0.4, 0.3, 0.2, 0.0)
    )


def test_negative_force_direction_is_selected_explicitly():
    args = steps.build_parser().parse_args(["--axis", "fz", "--sign", "-1"])
    assert steps.validate_args(args) == pytest.approx(
        (0.0, -0.2, -0.3, -0.4, -0.5, -0.4, -0.3, -0.2, 0.0)
    )


@pytest.mark.parametrize(
    "argv",
    (
        [],
        ["--axis", "fx"],
        ["--axis", "y"],
        ["--axis", "fy", "--sign", "0"],
        ["--axis", "fy", "--sign", "2"],
    ),
)
def test_cli_rejects_missing_axis_or_invalid_axis_and_sign(argv):
    with pytest.raises(SystemExit):
        steps.build_parser().parse_args(argv)


@pytest.mark.parametrize("option", ("--interval", "--timeout"))
@pytest.mark.parametrize("value", ("0", "-1", "nan", "inf", "-inf"))
def test_nonpositive_or_nonfinite_timing_is_rejected(option, value):
    args = steps.build_parser().parse_args(["--axis", "fy", f"{option}={value}"])
    with pytest.raises(ValueError):
        steps.validate_args(args)


def test_custom_positive_interval_and_timeout_are_accepted():
    args = steps.build_parser().parse_args(
        ["--axis", "fy", "--interval", "4.5", "--timeout", "1.5", "--dry-run"]
    )
    assert args.interval == 4.5
    assert args.timeout == 1.5
    assert args.dry_run
    assert len(steps.validate_args(args)) == 9


def test_sequence_writes_and_holds_each_target_in_order():
    events = []
    values = (0.0, 0.2, 0.3, 0.4, 0.5, 0.4, 0.3, 0.2, 0.0)
    steps.run_sequence(
        values,
        3.0,
        lambda value: events.append(("write", value)),
        lambda seconds: events.append(("wait", seconds)),
        emit=lambda _message: None,
    )

    expected = []
    for value in values:
        expected.extend((("write", value), ("wait", 3.0)))
    assert events == expected
    assert sum(value for kind, value in events if kind == "wait") == 27.0


def test_sequence_preserves_negative_targets():
    writes = []
    steps.run_sequence(
        (0.0, -0.2, -0.5, 0.0),
        0.1,
        writes.append,
        lambda _seconds: None,
        emit=lambda _message: None,
    )
    assert writes == [0.0, -0.2, -0.5, 0.0]


def test_interrupted_hold_does_not_send_later_targets():
    writes = []

    def interrupted_wait(_seconds):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        steps.run_sequence(
            (0.0, 0.2, 0.3),
            3.0,
            writes.append,
            interrupted_wait,
            emit=lambda _message: None,
        )
    assert writes == [0.0]


def test_failed_write_does_not_hold_or_send_later_targets():
    events = []

    def rejected_write(value):
        events.append(("write", value))
        raise RuntimeError("request rejected")

    with pytest.raises(RuntimeError, match="request rejected"):
        steps.run_sequence(
            (0.0, 0.2, 0.3),
            3.0,
            rejected_write,
            lambda seconds: events.append(("wait", seconds)),
            emit=lambda _message: None,
        )
    assert events == [("write", 0.0)]


def install_fake_ros(monkeypatch):
    """Install only fake modules; no ROS implementation is imported or initialized."""
    events = []
    ros = ModuleType("rclpy")
    ros.init = lambda **kwargs: events.append(("init", kwargs))
    ros.ok = lambda: True
    ros.shutdown = lambda: events.append(("shutdown",))
    signals = ModuleType("rclpy.signals")
    signals.SignalHandlerOptions = SimpleNamespace(NO="no_ros_signals")
    parameter = ModuleType("rclpy.parameter")

    class Parameter:
        def __init__(self, name, value):
            self.name = name
            self.value = value

        def to_parameter_msg(self):
            return SimpleNamespace(name=self.name, value=self.value)

    parameter.Parameter = Parameter
    srv = ModuleType("rcl_interfaces.srv")
    srv.GetParameters = SimpleNamespace(Request=SimpleNamespace)
    srv.SetParametersAtomically = SimpleNamespace(Request=SimpleNamespace)
    msg = ModuleType("rcl_interfaces.msg")
    msg.ParameterType = SimpleNamespace(PARAMETER_DOUBLE=3)
    for name, module in {
        "rclpy": ros,
        "rclpy.signals": signals,
        "rclpy.parameter": parameter,
        "rcl_interfaces": ModuleType("rcl_interfaces"),
        "rcl_interfaces.srv": srv,
        "rcl_interfaces.msg": msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(steps.signal, "signal", lambda *_args: None)
    return events


@pytest.mark.parametrize("axis_index", (0, 1))
def test_parameter_write_contains_only_selected_force_target(monkeypatch, axis_index):
    install_fake_ros(monkeypatch)
    adapter = steps.ParameterTarget.__new__(steps.ParameterTarget)
    adapter.name = steps.TARGET_NAMES[axis_index]
    adapter.writer = object()
    requests = []

    def call(client, request):
        assert client is adapter.writer
        requests.append(request)
        return SimpleNamespace(result=SimpleNamespace(successful=True))

    adapter._call = call
    adapter.write(-0.3)
    assert len(requests) == 1
    assert len(requests[0].parameters) == 1
    assert requests[0].parameters[0].name == steps.TARGET_NAMES[axis_index]
    assert requests[0].parameters[0].value == -0.3


def test_rejected_target_is_not_treated_as_success(monkeypatch):
    install_fake_ros(monkeypatch)
    adapter = steps.ParameterTarget.__new__(steps.ParameterTarget)
    adapter.name = steps.TARGET_NAMES[0]
    adapter.writer = object()
    adapter._call = lambda *_args: SimpleNamespace(
        result=SimpleNamespace(successful=False, reason="force norm exceeded")
    )
    with pytest.raises(RuntimeError, match="force norm exceeded"):
        adapter.write(0.5)


@pytest.mark.parametrize("axis_index", (0, 1))
def test_preflight_includes_unchanged_other_axis_in_force_limit(monkeypatch, axis_index):
    install_fake_ros(monkeypatch)
    adapter = steps.ParameterTarget.__new__(steps.ParameterTarget)
    adapter.axis_index = axis_index
    adapter.name = steps.TARGET_NAMES[axis_index]
    adapter.reader = object()
    # The selected axis starts at zero; the other axis must not be zeroed implicitly.
    targets = [0.0, 0.4] if axis_index == 0 else [0.4, 0.0]
    settings = targets + [0.6]
    requests = []

    def call(client, request):
        assert client is adapter.reader
        requests.append(request)
        return SimpleNamespace(
            values=[SimpleNamespace(type=3, double_value=value) for value in settings]
        )

    adapter._call = call
    with pytest.raises(RuntimeError, match="unchanged other-axis"):
        adapter.preflight((0.0, 0.5))  # sqrt(0.5**2 + 0.4**2) exceeds 0.6 N.
    assert requests[0].names == list(steps.TARGET_NAMES) + ["admittance/max_force_N"]
    adapter.preflight((0.0, 0.3))  # Vector norm is 0.5 N; no setting is written.


def test_dry_run_never_initializes_ros_or_constructs_parameter_client(monkeypatch, capsys):
    ros_events = install_fake_ros(monkeypatch)

    def forbidden_adapter(_args):
        pytest.fail("dry-run must not construct a parameter client")

    monkeypatch.setattr(steps, "ParameterTarget", forbidden_adapter)
    assert steps.main(["--axis", "fz", "--sign", "-1", "--dry-run"]) == 0
    assert ros_events == []
    assert "-500" in capsys.readouterr().out


@pytest.mark.parametrize("failure", ("interrupt", "first_write_timeout", "preflight"))
def test_main_failure_cleanup_and_lifecycle_are_explicit(monkeypatch, failure):
    ros_events = install_fake_ros(monkeypatch)
    actions = []

    class FakeTarget:
        def __init__(self, args):
            assert args.axis == "fz"
            self.write_count = 0

        def preflight(self, _values):
            if failure == "preflight":
                raise RuntimeError("bad settings")

        def write(self, value):
            self.write_count += 1
            actions.append(("fz", value))
            if failure == "first_write_timeout" and self.write_count == 1:
                raise RuntimeError("timeout: target state unknown")

        def close(self):
            actions.append(("close",))

    def wait(_seconds):
        if failure == "interrupt":
            raise KeyboardInterrupt
        pytest.fail("no hold should follow a preflight or write failure")

    monkeypatch.setattr(steps, "ParameterTarget", FakeTarget)
    monkeypatch.setattr(steps.time, "sleep", wait)
    result = steps.main(["--axis", "fz"])
    assert result == (130 if failure == "interrupt" else 1)
    if failure == "preflight":
        assert actions == [("close",)]  # Never overwrite targets on read-only failure.
    else:
        assert actions == [("fz", 0.0), ("fz", 0.0), ("close",)]
    assert ros_events[0][0] == "init"
    assert ros_events[-1] == ("shutdown",)


def test_successful_main_finishes_at_zero_without_extra_writes(monkeypatch):
    install_fake_ros(monkeypatch)
    actions = []

    class FakeTarget:
        def __init__(self, args):
            assert args.axis == "fy"

        def preflight(self, _values):
            pass

        def write(self, value):
            actions.append(("fy", value))

        def close(self):
            actions.append(("close",))

    monkeypatch.setattr(steps, "ParameterTarget", FakeTarget)
    monkeypatch.setattr(steps.time, "sleep", lambda value: actions.append(("wait", value)))
    assert steps.main(["--axis", "fy", "--sign", "-1"]) == 0
    assert [value for kind, value in actions[:-1] if kind == "fy"] == pytest.approx(
        (0.0, -0.2, -0.3, -0.4, -0.5, -0.4, -0.3, -0.2, 0.0)
    )
    assert actions[-2] == ("wait", 3.0)
    assert actions[-1] == ("close",)


@pytest.mark.parametrize(
    "failure, message",
    (
        ("unavailable", "Service unavailable"),
        ("timeout", "target state unknown"),
        ("exception", "fake DDS failure"),
        ("empty", "Empty parameter service response"),
    ),
)
def test_service_failures_are_reported(failure, message):
    adapter = steps.ParameterTarget.__new__(steps.ParameterTarget)
    adapter.timeout = 0.5
    adapter.node = object()
    adapter.ros = SimpleNamespace(spin_until_future_complete=lambda *_args, **_kwargs: None)
    future = SimpleNamespace(
        done=lambda: failure != "timeout",
        exception=lambda: RuntimeError("fake DDS failure") if failure == "exception" else None,
        result=lambda: None,
    )
    client = SimpleNamespace(
        srv_name="/fake/set_parameters_atomically",
        wait_for_service=lambda **_kwargs: failure != "unavailable",
        call_async=lambda _request: future,
    )
    with pytest.raises(RuntimeError, match=message):
        adapter._call(client, object())


def test_failed_zero_cleanup_is_explicit_and_does_not_send_more_steps(monkeypatch, capsys):
    ros_events = install_fake_ros(monkeypatch)
    actions = []

    class FakeTarget:
        def __init__(self, _args):
            pass

        def preflight(self, _values):
            pass

        def write(self, value):
            actions.append(value)
            raise RuntimeError("no acknowledgement")

        def close(self):
            pass

    monkeypatch.setattr(steps, "ParameterTarget", FakeTarget)
    assert steps.main(["--axis", "fy"]) == 1
    assert actions == [0.0, 0.0]
    assert "ZERO RESET FAILED" in capsys.readouterr().err
    assert ros_events[-1] == ("shutdown",)
