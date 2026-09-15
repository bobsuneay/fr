from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import pytest
from fr3_control_panel.session import BringupSession, launch_command


def options(tmp_path, mode='real'):
    real = tmp_path/'real config.yaml'
    cell = tmp_path/'cell.yaml'
    real.touch()
    cell.touch()
    return dict(mode=mode, real_config=str(real), cell=str(cell), allow_execution=False)


def test_ip_is_passed_as_an_argument_not_shell_code(tmp_path):
    cfg = options(tmp_path)
    cmd = launch_command(cfg, '192.168.58.2', '/dev/ttyACM0', True)
    assert 'robot_ip:=192.168.58.2' in cmd
    assert 'enable_execution:=false' in cmd
    assert 'real_config:='+str(Path(cfg['real_config']).resolve()) in cmd
    assert not any('dual' in part or 'gazebo' in part for part in cmd)


@pytest.mark.parametrize('ip', ['192.168.58.999', '127.0.0.1; touch x', ''])
def test_invalid_ip_rejected(tmp_path, ip):
    with pytest.raises(ValueError):
        launch_command(options(tmp_path), ip, '/dev/ttyACM0', True)


def test_real_requires_confirmation_and_existing_config(tmp_path):
    cfg = options(tmp_path)
    with pytest.raises(ValueError):
        launch_command(cfg, '192.168.58.2', '/dev/ttyACM0', False)
    cfg['cell'] = str(tmp_path/'absent.yaml')
    with pytest.raises(ValueError):
        launch_command(cfg, '192.168.58.2', '/dev/ttyACM0', True)


def test_no_second_launch_while_process_is_owned(tmp_path):
    session = BringupSession(options(tmp_path))
    session.process = object()
    with pytest.raises(RuntimeError):
        session.start('192.168.58.2', '/dev/ttyACM0', True)


def test_cleanup_signals_owned_group_and_refuses_live_descendants(monkeypatch, tmp_path):
    from fr3_control_panel import session as module
    obj = BringupSession(options(tmp_path))
    obj.process = MagicMock(pid=43210)
    kill = MagicMock()
    monkeypatch.setattr(module.os, 'killpg', kill, raising=False)
    with pytest.raises(RuntimeError, match='子进程'):
        obj.stop()
    assert obj.process is not None
    assert kill.call_args_list[-1].args == (43210, 0)
    kill.side_effect = [None, ProcessLookupError()]
    obj.stop()
    assert obj.process is None
