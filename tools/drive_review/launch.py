#!/usr/bin/env python3
"""Always reanalyse with current source in Docker, then serve locally."""
import argparse
from datetime import datetime
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import shlex
import subprocess
import webbrowser

ROOT=Path(__file__).resolve().parents[2]
CONTAINER=Path('/home/user/Clothoid-A')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bags',nargs='*',type=Path,help='repo 안 bag 폴더; 생략하면 최근 parking_* 3개')
    p.add_argument('--port',type=int,default=8766)
    p.add_argument('--hz',type=float,default=10)
    a=p.parse_args()
    bags=a.bags or sorted((ROOT/'bag').glob('parking_*/metadata.yaml'),key=lambda f:f.stat().st_mtime)[-3:]
    bags=[b.parent if b.name=='metadata.yaml' else b for b in bags]
    if not bags:p.error('parking bag 없음: 경로를 지정하세요')
    try: paths=[str(CONTAINER/b.resolve().relative_to(ROOT)) for b in bags]
    except ValueError:p.error('bag은 Docker에 공유된 저장소 안에 있어야 합니다')
    for b in bags:
        if not (b/'metadata.yaml').is_file():p.error(f'bag metadata 없음: {b}')
    out=ROOT/'ros2_ws/log'/('drive_review_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    # Bind before expensive analysis so an occupied port never wastes a full export.
    server=ThreadingHTTPServer(('127.0.0.1',a.port),partial(SimpleHTTPRequestHandler,directory=str(out)))
    try:
        subprocess.run(['docker','start','clothoid-dev'],check=True)
        command=['python3','tools/drive_review/export.py',*paths,'--output',str(CONTAINER/out.relative_to(ROOT)),'--hz',str(a.hz)]
        shell='source /opt/ros/humble/setup.bash && source /home/user/Clothoid-A/ros2_ws/install/setup.bash && cd /home/user/Clothoid-A && OPENBLAS_NUM_THREADS=1 '+shlex.join(command)
        print('현재 소스 코드로 다시 분석합니다. 기존 결과를 재사용하지 않습니다.',flush=True)
        subprocess.run(['docker','exec','-u','user','clothoid-dev','bash','-lc',shell],check=True)
        url=f'http://127.0.0.1:{a.port}'
        print(f'리뷰 화면: {url}\n결과: {out}\n종료: Ctrl+C',flush=True)
        webbrowser.open(url)
        server.serve_forever()
    except KeyboardInterrupt: pass
    finally:server.server_close()

if __name__=='__main__':main()
