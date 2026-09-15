import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import math
import time
import pytest
import yaml
from PyQt5 import QtWidgets
from fr3_control_panel.core import opening_to_joint, pose_values, save_points, load_points
from fr3_control_panel.validation import validate_config
from fr3_control_panel.app import Panel


@pytest.fixture(scope='module')
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def config():
    return yaml.safe_load((Path(__file__).parents[1]/'config/panel.yaml').read_text())


def test_opening_direction():
    assert opening_to_joint(100, .1) == 0
    assert opening_to_joint(0, .1) == .1
    assert opening_to_joint(50, .1) == .05
    with pytest.raises(ValueError):
        opening_to_joint(-1, .1)


@pytest.mark.parametrize('values', [[0]*5, [0]*5+[float('nan')], [0]*5+[True]])
def test_invalid_pose(values):
    with pytest.raises(ValueError):
        pose_values(values)


def test_points_frame_guard(tmp_path):
    path = tmp_path/'p.json'
    points = {'抓取': [.3, .1, .4, 180, 0, 90]}
    save_points(path, 'base_link', 'gripper_tcp', points)
    assert load_points(path, 'base_link', 'gripper_tcp') == points
    with pytest.raises(ValueError):
        load_points(path, 'world', 'gripper_tcp')


def test_config():
    cfg = config()
    validate_config(cfg)
    cfg['velocity'] = 2
    with pytest.raises(ValueError):
        validate_config(cfg)


class FakeBackend:
    mock = True
    payload = False
    fault = None
    connected = True
    execution_enabled = True
    grasp_pending = False

    def __init__(self, fail=None):
        self.calls = []
        self.fail = fail

    def begin(self):
        pass

    def snapshot(self):
        stamp = time.monotonic()
        return {n: (0, stamp) for n in config()['joints']+['gripper_joint']}, [0]*6, stamp

    def capture(self):
        return [.3, .1, .4, 180, 0, 90]

    def gripper(self, opening, grasp=False):
        self.calls.append(('gripper', opening))
        if grasp and self.fail == 'grasp':
            raise RuntimeError('empty grasp')

    def motion(self, values, execute=False):
        self.calls.append(('motion', values))
        if self.fail == 'plan':
            raise RuntimeError('collision')

    def attach(self):
        self.calls.append(('attach',))
        self.payload = True

    def cancel(self):
        pass


def finish(app, panel):
    deadline = time.monotonic()+3
    while panel.busy and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(.01)
    assert not panel.busy


@pytest.mark.parametrize('failure,expected', [
    ('plan', ['gripper', 'motion']),
    ('grasp', ['gripper', 'motion', 'gripper']),
    (None, ['gripper', 'motion', 'gripper', 'attach', 'motion', 'motion'])])
def test_sequence_fail_closed(app, failure, expected):
    backend = FakeBackend(failure)
    panel = Panel(config(), backend)
    panel.points = {'抓取': [0]*6, '展示': [1]*6, '途经1': [.5]*6}
    panel.pick()
    finish(app, panel)
    if failure is None:
        assert [x[0] for x in backend.calls] == ['gripper', 'motion', 'gripper']
        assert panel.grasp_ready  # No automatic transport based on a stalled gripper.
        panel.confirm_grasp()
        finish(app, panel)
    assert [x[0] for x in backend.calls] == expected
    if failure is None:
        assert backend.calls[-2][1] == [.5]*6
        assert backend.calls[-1][1] == [1]*6
    panel.close()


def test_capture_and_preview(app):
    panel = Panel(config(), FakeBackend())
    panel.capture('抓取')
    assert panel.points['抓取'] == [.3, .1, .4, 180, 0, 90]
    panel.close()
    demo = Panel(config())
    assert all(not button.isEnabled() for button in demo.command_buttons)
    demo.close()


def test_motion_requires_explicit_arming_but_capture_does_not(app):
    panel = Panel(config(), FakeBackend())
    assert all(not b.isEnabled() for b in panel.motion_buttons)
    assert next(b for b in panel.command_buttons if b.text() == '采集抓取位姿').isEnabled()
    panel.arm_motion.setChecked(True)
    assert next(b for b in panel.motion_buttons if b.text() == '规划并到达').isEnabled()
    panel.backend.execution_enabled = False
    panel.refresh()
    assert all(not b.isEnabled() for b in panel.motion_buttons)
    panel.close()


def test_reorder_points_and_confirmation_keeps_original_snapshot(app):
    backend = FakeBackend()
    panel = Panel(config(), backend)
    panel.points = {'抓取': [0]*6, '展示': [1]*6, '途经1': [.4]*6, '途经2': [.6]*6}
    panel.update_points()
    panel.list.setCurrentRow(3)
    panel.reorder_selected(-1)
    assert list(panel.points)[2:] == ['途经2', '途经1']
    panel.pick()
    finish(app, panel)
    panel.points['展示'] = [9]*6
    panel.confirm_grasp()
    finish(app, panel)
    assert [call[1] for call in backend.calls if call[0] == 'motion'] == [[0]*6, [.6]*6, [.4]*6, [1]*6]
    panel.close()


def test_cancel_before_confirmation_blocks_transport(app):
    panel = Panel(config(), FakeBackend())
    panel.points = {'抓取': [0]*6, '展示': [1]*6}
    panel.pick()
    finish(app, panel)
    panel.cancel()
    with pytest.raises(RuntimeError):
        panel.confirm_grasp()
    assert not panel.backend.payload
    panel.close()


def test_disconnect_does_not_clear_busy_until_worker_exits(app):
    import threading
    backend = FakeBackend()
    backend.disconnect = lambda: setattr(backend, 'connected', False)
    panel = Panel(config(), backend)
    gate = threading.Event()
    panel.run('slow action', lambda: gate.wait(2))
    panel.disconnect_devices()
    assert panel.busy and panel.disconnecting
    assert not panel.connect_button.isEnabled()
    gate.set()
    finish(app, panel)
    assert not backend.connected
    panel.close()
