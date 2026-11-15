#!/usr/bin/env python3
"""Run the native viewer in the existing ROS Docker container."""
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
MOUNT = Path('/home/user/Clothoid-A')
arguments = []
for value in sys.argv[1:]:
    path = Path(value)
    if path.exists() or path.is_absolute():
        try:
            value = str(MOUNT/path.resolve().relative_to(ROOT))
        except ValueError:
            raise SystemExit('백과 저장할 화면은 저장소의 Docker 공유 경로 안에 두세요.')
    arguments.append(value)
subprocess.run(['docker', 'start', 'clothoid-dev'], check=True, stdout=subprocess.DEVNULL)
command = ('source /opt/ros/humble/setup.bash && cd /home/user/Clothoid-A && '
           'source ros2_ws/install/local_setup.bash && '
           'OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 QT_X11_NO_MITSHM=1 '+
           shlex.join(['python3', 'tools/bump_viewer/viewer.py', *arguments]))
raise SystemExit(subprocess.call(['docker', 'exec', 'clothoid-dev', 'bash', '-lc', command]))
