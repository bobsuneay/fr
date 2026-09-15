from copy import deepcopy
from pathlib import Path
import pytest
import yaml
from fr3_real_bringup.configuration import validate_real


def commissioned():
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config/real.example.yaml').read_text())
    cfg.update(firmware_version='3.9.7', driver_package='fairino_hardware_v3_9_7')
    cfg['checks'] = {key: True for key in cfg['checks']}
    return cfg


def test_validated_single_arm_parameters():
    cfg = commissioned()
    assert validate_real(cfg, True, '192.168.58.2')['update_rate'] == 125


@pytest.mark.parametrize('key,value', [('update_rate', 100), ('firmware_version', '3.8.0'),
                                     ('driver_package', 'fairino_hardware_other'),
                                     ('driver_configured_ip', '192.168.58.3')])
def test_mismatched_adapter_parameters_fail(key, value):
    cfg = commissioned()
    cfg[key] = value
    with pytest.raises(ValueError):
        validate_real(cfg, True, '192.168.58.2')


def test_gui_ip_must_match_commissioned_robot_and_defaults_are_not_authorized():
    cfg = commissioned()
    with pytest.raises(ValueError):
        validate_real(cfg, True, '192.168.58.3')
    with pytest.raises(ValueError):
        validate_real(cfg, False, '192.168.58.2')
    raw = yaml.safe_load((Path(__file__).parents[1]/'config/real.example.yaml').read_text())
    with pytest.raises(ValueError):
        validate_real(raw, True, '192.168.58.2')


@pytest.mark.parametrize('key,value', [('position_open_register', 0), ('target_force_percent', 0),
                                     ('baud_rate', True), ('slave_address', 248)])
def test_invalid_gripper_settings_fail(key, value):
    cfg = commissioned()
    cfg['gripper'][key] = value
    with pytest.raises(ValueError):
        validate_real(cfg, True, '192.168.58.2')
