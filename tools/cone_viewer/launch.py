#!/usr/bin/env python3
"""Start the native viewer inside the existing ROS Docker container."""
from pathlib import Path
import shlex
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[2]
CONTAINER=Path('/home/user/Clothoid-A')
args=sys.argv[1:];mapped=[]
for value in args:
    path=Path(value)
    if path.exists() and (path.is_dir() or path.suffix=='.db3'):
        try:value=str(CONTAINER/path.resolve().relative_to(ROOT))
        except ValueError:raise SystemExit('백은 저장소 안에 두세요. Docker 공유 경로가 필요합니다.')
    mapped.append(value)
subprocess.run(['docker','start','clothoid-dev'],check=True,stdout=subprocess.DEVNULL)
command=['python3','tools/cone_viewer/viewer.py',*mapped]
shell='source /opt/ros/humble/setup.bash && cd /home/user/Clothoid-A && source ros2_ws/install/setup.bash && PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 QT_X11_NO_MITSHM=1 '+shlex.join(command)
# Use the container's existing DISPLAY/XAUTHORITY; no access-control changes.
raise SystemExit(subprocess.call(['docker','exec','clothoid-dev','bash','-lc',shell]))
