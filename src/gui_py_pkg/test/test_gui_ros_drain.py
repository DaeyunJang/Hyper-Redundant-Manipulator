"""Qt drains ROS callbacks in bounded batches, without starting real nodes."""

from types import SimpleNamespace
from unittest.mock import Mock

from gui_py_pkg import gui_node


def test_fast_callbacks_are_batched_with_a_count_limit(monkeypatch):
    clock = {'now': 1.0}
    monkeypatch.setattr(gui_node.time, 'monotonic', lambda: clock['now'])
    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    spin = Mock()
    monkeypatch.setattr(gui_node.rclpy, 'spin_once', spin)
    window = SimpleNamespace(node=object())

    gui_node.MyGUI.node_spin_once(window)

    assert spin.call_count == 16
    assert all(call.args == (window.node,) and call.kwargs == {'timeout_sec': 0.0}
               for call in spin.call_args_list)


def test_batch_stops_starting_callbacks_after_time_budget(monkeypatch):
    clock = {'now': 1.0}
    monkeypatch.setattr(gui_node.time, 'monotonic', lambda: clock['now'])
    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)

    def execute_callback(*args, **kwargs):
        clock['now'] += 0.00075

    spin = Mock(side_effect=execute_callback)
    monkeypatch.setattr(gui_node.rclpy, 'spin_once', spin)

    gui_node.MyGUI.node_spin_once(SimpleNamespace(node=object()))

    # The current callback completes; a fourth cannot start past the 2 ms budget.
    assert spin.call_count == 3


def test_stopped_context_does_not_spin(monkeypatch):
    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: False)
    spin = Mock()
    monkeypatch.setattr(gui_node.rclpy, 'spin_once', spin)

    gui_node.MyGUI.node_spin_once(SimpleNamespace(node=object()))

    spin.assert_not_called()


def test_context_shutdown_interrupts_batch(monkeypatch):
    monkeypatch.setattr(gui_node.time, 'monotonic', lambda: 1.0)
    monkeypatch.setattr(gui_node.rclpy, 'ok', Mock(side_effect=[True, True, False]))
    spin = Mock()
    monkeypatch.setattr(gui_node.rclpy, 'spin_once', spin)

    gui_node.MyGUI.node_spin_once(SimpleNamespace(node=object()))

    assert spin.call_count == 2


def test_burst_is_drained_without_publishing_or_changing_source(monkeypatch):
    monkeypatch.setattr(gui_node.time, 'monotonic', lambda: 1.0)
    monkeypatch.setattr(gui_node.rclpy, 'ok', lambda: True)
    node = Mock()
    queued = ['ft', 'loadcell', 'motor', 'prediction'] * 3
    received = []

    def receive_one(*args, **kwargs):
        if queued:
            received.append(queued.pop(0))

    monkeypatch.setattr(gui_node.rclpy, 'spin_once', receive_one)

    gui_node.MyGUI.node_spin_once(SimpleNamespace(node=node))

    assert not queued
    assert len(received) == 12
    assert not node.mock_calls
