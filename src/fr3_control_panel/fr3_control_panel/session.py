"""Own exactly one single-arm bringup process; never send SDK motion commands."""
import ipaddress
import os
from pathlib import Path
import signal
import subprocess
import tempfile


def launch_command(options, robot_ip, serial_port, confirmed):
    mode = options['mode']
    if mode not in ('mock', 'real'):
        raise ValueError('mode must be mock or real')
    ipaddress.IPv4Address(robot_ip)
    if mode == 'real':
        if not confirmed:
            raise ValueError('请先确认现场启动条件；连接驱动可能发送保持指令')
        for key in ('real_config', 'cell'):
            if not Path(options[key]).expanduser().is_file():
                raise ValueError('配置文件不存在：'+options[key])
        if not serial_port.startswith('/dev/') or '\n' in serial_port:
            raise ValueError('夹爪串口应为 /dev/...')
    command = ['ros2', 'launch', 'fr3_real_bringup', 'bringup.launch.py',
               'mode:='+mode, 'robot_ip:='+robot_ip, 'serial_port:='+serial_port,
               'confirm_real:='+str(mode == 'real').lower(),
               'enable_execution:='+str(options['allow_execution']).lower(),
               'rviz:=true']
    for key in ('real_config', 'cell'):
        if options.get(key):
            command.append(key+':='+str(Path(options[key]).expanduser().resolve()))
    return command


class BringupSession:
    def __init__(self, options):
        self.options = options
        self.process = None
        self.log_path = None

    def start(self, robot_ip, serial_port, confirmed):
        if self.process is not None:
            raise RuntimeError('本界面已有后端，请先断开连接')
        command = launch_command(self.options, robot_ip, serial_port, confirmed)
        if os.name != 'posix':
            raise RuntimeError('设备后端需要 Ubuntu / ROS 2；Windows 仅支持 --demo')
        # A file cannot deadlock like an undrained stdout PIPE. No shell expansion.
        with tempfile.NamedTemporaryFile(prefix='fr3-single-arm-', suffix='.log', delete=False) as log:
            self.log_path = log.name
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                            stdout=log, stderr=subprocess.STDOUT,
                                            start_new_session=True)

    def check(self):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('单臂后端已退出。日志：'+str(self.log_path))

    def stop(self):
        if self.process is None:
            return
        process = self.process
        # Signal the owned process group, including children if launch died first.
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            raise RuntimeError('后端尚未退出；不强杀硬件驱动，请现场确认。日志：'+str(self.log_path))
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            self.process = None
        else:
            raise RuntimeError('仍有后端子进程；禁止重复连接。日志：'+str(self.log_path))
