"""Operator preview and speed controls without a ROS executor or display."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fr3_bolt_inspection_cell.preview import PreviewState


def test_camera_switch_clears_previous_image_until_new_camera_has_a_frame():
    preview = PreviewState()
    frame = NS(encoding='rgb8')
    assert preview.select('head_camera', (frame, 10), 10) == (True, frame, '')
    changed, image, text = preview.select('left_d435i', None, 10.1)
    assert changed and image is None and 'left_d435i' in text and '等待' in text
    assert not preview.select('left_d435i', None, 10.2)[0]
    # Switching back must repaint even when it is the same cached frame.
    assert preview.select('head_camera', (frame, 10), 10.3) == (True, frame, '')


def test_camera_disconnect_clears_old_frame_and_new_frames_restore_preview():
    preview = PreviewState()
    frame = NS(encoding='bgr8')
    preview.select('waist_camera', (frame, 10), 10)
    assert not preview.select('waist_camera', (frame, 10), 11)[0]
    changed, image, text = preview.select('waist_camera', (frame, 10), 12.1)
    assert changed and image is None and '过期' in text
    assert not preview.select('waist_camera', (frame, 10), 13)[0]
    fresh_frame = NS(encoding='bgr8')
    assert preview.select('waist_camera', (fresh_frame, 13.1), 13.1) == (True, fresh_frame, '')


def test_unsupported_camera_encoding_replaces_previous_image_with_error():
    preview = PreviewState()
    preview.select('head_camera', (NS(encoding='rgb8'), 10), 10)
    changed, image, text = preview.select('left_d435i', (NS(encoding='16UC1'), 10), 10)
    assert changed and image is None and '16UC1' in text


class Value:
    def __init__(self, value):
        self.value = value

    def set(self, value):
        self.value = value

    def get(self):
        return self.value


@pytest.fixture
def panel():
    pytest.importorskip('tkinter')
    from fr3_bolt_inspection_cell.panel_ui import InspectionPanel
    ui = InspectionPanel.__new__(InspectionPanel)
    ui.root, ui.set_speed = MagicMock(), MagicMock()
    ui.speed_timer = None
    ui.speed_initialized = ui.speed_available = False
    ui.speed_value, ui.speed_label = Value(1.), Value('')
    for name in ('speed_slider', 'handover', 'start', 'retry', 'randomize'):
        setattr(ui, name, MagicMock())
    ui.controls({'scan_speed_scale': 1.})
    return ui


def test_speed_disconnect_cancels_pending_publish_and_restores_backend_value(panel):
    panel.speed_value.set(1.8)
    panel.speed_changed('1.8')
    scheduled_callback = panel.root.after.call_args.args[1]
    timer = panel.speed_timer
    panel.controls({})
    panel.root.after_cancel.assert_called_once_with(timer)
    assert panel.speed_timer is None
    panel.speed_slider.configure.assert_called_with(state='disabled')
    # Even an already-dispatched callback must not publish after disconnection.
    scheduled_callback()
    panel.set_speed.assert_not_called()
    panel.controls({'scan_speed_scale': .75})
    assert panel.speed_value.get() == .75
    panel.set_speed.assert_not_called()


def test_speed_debounce_sends_latest_value_only_while_connected(panel):
    panel.speed_value.set(1.234)
    panel.speed_changed('1.234')
    timer = panel.speed_timer
    panel.speed_value.set(1.567)
    panel.speed_changed('1.567')
    panel.root.after_cancel.assert_called_once_with(timer)
    panel.root.after.call_args.args[1]()
    panel.set_speed.assert_called_once_with(1.57)
    assert panel.speed_timer is None
