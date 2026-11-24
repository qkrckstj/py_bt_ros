"""One factory Nav2 stack per user and ROS domain on this machine."""
import fcntl
import os
from pathlib import Path
import tempfile


def acquire_factory_session(domain=None, directory=None):
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0') if domain is None else domain)
    path = Path(directory or tempfile.gettempdir()) / f'limo-factory-{os.getuid()}-{domain}.lock'
    handle = path.open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.seek(0)
        pid = handle.read().strip()
        handle.close()
        raise RuntimeError(
            f'Factory navigation already running in ROS_DOMAIN_ID={domain} (launch PID {pid}). '
            'Close its BT/RViz window or stop its launch with Ctrl+C before starting another.') from None
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid()))
    handle.flush()
    # Keep the inode: unlinking it would allow other launches to lock a new file.
    return handle
