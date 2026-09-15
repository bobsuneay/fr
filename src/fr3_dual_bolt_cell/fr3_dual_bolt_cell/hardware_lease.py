"""Linux same-host mutual exclusion for project-owned hardware launches."""
import hashlib
import os
from pathlib import Path
import tempfile
import atexit


class HardwareLease:
    def __init__(self, config):
        import fcntl
        self.files = []
        targets = [config[s]['robot_ip'] for s in ('left', 'right')]
        targets += [str(Path(config[s]['serial_port']).resolve()) for s in ('left', 'right')]
        try:
            for target in sorted(targets):
                name = hashlib.sha256(target.encode()).hexdigest()
                path = Path(tempfile.gettempdir())/('fr3-hardware-'+name+'.lock')
                fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                file = os.fdopen(fd, 'a+')
                self.files.append(file)
                fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Keep the lease until the launch process exits, after its child processes.
            # OnShutdown fires before controller_manager has necessarily disconnected.
            atexit.register(self.close)
        except Exception:
            self.close()
            raise RuntimeError('Hardware already claimed or lock unavailable; stop the other project launch')

    def close(self):
        for file in self.files:
            file.close()
        self.files.clear()

    def __del__(self):
        self.close()
