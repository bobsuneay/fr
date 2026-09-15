"""Same host/IP/serial lock naming as the dual-arm adapter, without importing it."""
import atexit
import hashlib
import os
from pathlib import Path
import tempfile


class HardwareLease:
    def __init__(self, robot_ip, serial_port):
        import fcntl
        self.files = []
        try:
            for target in sorted([robot_ip, str(Path(serial_port).resolve())]):
                name = hashlib.sha256(target.encode()).hexdigest()
                path = Path(tempfile.gettempdir())/('fr3-hardware-'+name+'.lock')
                fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                file = os.fdopen(fd, 'a+')
                self.files.append(file)
                fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Release after launch has shut down children, not during OnShutdown.
            atexit.register(self.close)
        except Exception:
            self.close()
            raise RuntimeError('机械臂 IP / 串口已被本机工程占用，或无法获取设备锁')

    def close(self):
        for file in self.files:
            file.close()
        self.files.clear()
