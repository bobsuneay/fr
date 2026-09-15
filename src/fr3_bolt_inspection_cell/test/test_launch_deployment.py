"""Expand launch models with stand-in launch actions; no ROS runtime claim."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import pytest
import yaml

SHARE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SHARE), str(SHARE.parent/'fr3_dual_bolt_cell')]


@pytest.mark.parametrize('mode', ['mock', 'gazebo', 'real'])
def test_launch_builds_consistent_models_for_each_backend(monkeypatch, tmp_path, mode):
    for name in ('ament_index_python.packages', 'launch', 'launch.actions', 'launch.conditions',
                 'launch.event_handlers', 'launch.events', 'launch.launch_description_sources',
                 'launch.substitutions', 'launch_ros.actions', 'launch_ros.parameter_descriptions'):
        monkeypatch.setitem(sys.modules, name, MagicMock())
    import fr3_dual_bolt_cell.hardware_lease as leasing
    lease = MagicMock()
    monkeypatch.setattr(leasing, 'HardwareLease', lease)
    real = yaml.safe_load((SHARE/'config/real_feedback.example.yaml').read_text())
    real['calibrated'] = True
    for side in ('left', 'right'):
        real[side].update(contact_min=10, contact_max=100)
    real_file = tmp_path/'real.yaml'
    real_file.write_text(yaml.safe_dump(real))
    hw = yaml.safe_load((SHARE.parent/'fr3_dual_bolt_cell/config/hardware.example.yaml').read_text())
    hw['commissioned'] = True
    for side, ip in [('left', '192.168.1.2'), ('right', '192.168.1.3')]:
        hw[side].update(robot_ip=ip, serial_port='/dev/'+side)
    hw_file = tmp_path/'hw.yaml'
    hw_file.write_text(yaml.safe_dump(hw))
    args = dict(mode=mode, enable_execution='false', rviz='false', gui='false', panel='false',
                scene=str(SHARE/'config/scene.yaml'), arms=str(SHARE/'config/arms.yaml'),
                inspection=str(SHARE/'config/inspection.yaml'), real_feedback=str(real_file), hardware=str(hw_file))
    spec = importlib.util.spec_from_file_location('_inspection_launch', SHARE/'launch/bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.LaunchConfiguration = lambda name: NS(perform=lambda context: args[name])
    module.get_package_share_directory = lambda name: str(SHARE.parent/name)
    module.get_packages_with_prefixes = lambda: {'fairino_hardware_v3_9_7': str(SHARE.parent/'fairino_hardware_v3_9_7')}
    original_exists = Path.exists
    monkeypatch.setattr(Path, 'exists', lambda p: True if str(p).replace('\\', '/').startswith('/dev/') else original_exists(p))
    assert module.start(None)
    tasks = [call for call in module.Node.call_args_list if call.kwargs.get('executable') == 'inspection_task']
    assert len(tasks) == 1
    assert tasks[0].kwargs['parameters'][0]['mode'] == mode
    assert not tasks[0].kwargs['parameters'][0]['enable_execution']
    if mode == 'real':
        lease.assert_called_once()
        assert not any(c.kwargs.get('package') == 'gazebo_ros' for c in module.Node.call_args_list)
