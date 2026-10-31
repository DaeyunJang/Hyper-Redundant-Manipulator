"""Entry-point shutdown is safe under roslaunch/terminal duplicate interrupts."""

from unittest.mock import Mock

import pytest

from estimation_pkg import force_id_estimation as inference


@pytest.mark.parametrize('interrupt', [KeyboardInterrupt, inference.ExternalShutdownException])
@pytest.mark.parametrize('cleanup_interrupt', [False, True])
def test_shutdown_after_interrupt(monkeypatch, interrupt, cleanup_interrupt):
    node = Mock()
    if cleanup_interrupt:
        node.destroy_node.side_effect = KeyboardInterrupt
    init = Mock()
    shutdown = Mock()
    monkeypatch.setattr(inference.rclpy, 'init', init)
    monkeypatch.setattr(inference.rclpy, 'spin', Mock(side_effect=interrupt))
    monkeypatch.setattr(inference.rclpy, 'try_shutdown', shutdown)
    monkeypatch.setattr(inference, 'ForceIdEstimationNode', Mock(return_value=node))

    inference.main(args=[])

    init.assert_called_once_with(args=[])
    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()


def test_duplicate_interrupt_during_context_shutdown(monkeypatch):
    node = Mock()
    shutdown = Mock(side_effect=KeyboardInterrupt)
    monkeypatch.setattr(inference.rclpy, 'init', Mock())
    monkeypatch.setattr(inference.rclpy, 'spin', Mock(side_effect=KeyboardInterrupt))
    monkeypatch.setattr(inference.rclpy, 'try_shutdown', shutdown)
    monkeypatch.setattr(inference, 'ForceIdEstimationNode', Mock(return_value=node))

    inference.main()

    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()
