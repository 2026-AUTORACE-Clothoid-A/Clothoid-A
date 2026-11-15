"""python3 -m pytest tools/carmon -q"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from carmon_lib import (cpu_split, name_process, parse_ctxt, parse_load_line,  # noqa: E402
                        parse_pid_stat, parse_proc_stat, parse_tegrastats, percentile)

ORIN = ('11-15-2026 10:00:00 RAM 4100/7620MB (lfb 2x4MB) SWAP 150/3810MB (cached 0MB) '
        'CPU [62%@1510,70%@1510,55%@1267,80%@806,off,off] EMC_FREQ 15%@2133 GR3D_FREQ 45%@[624,624] '
        'VIC_FREQ 0%@115 APE 174 CV0@-256C CPU@60.1C SOC2@57.3C GPU@57.5C tj@61.2C '
        'VDD_IN 9500mW/9400mW VDD_CPU_GPU_CV 3200mW/3100mW')
OLD = 'RAM 2000/3964MB (lfb 1x4MB) CPU [10%@1479,20%@1479] EMC_FREQ 0% GR3D_FREQ 7%@921 PLL@40C CPU@42.5C'


def test_tegrastats_orin():
    r = parse_tegrastats(ORIN)
    assert (r['ram_used_mb'], r['ram_total_mb'], r['swap_used_mb']) == (4100, 7620, 150)
    assert r['cores'][:4] == [(62.0, 1510), (70.0, 1510), (55.0, 1267), (80.0, 806)]
    assert r['cores'][4] == (None, None)
    assert (r['gpu_pct'], r['gpu_mhz'], r['emc_pct'], r['power_mw']) == (45, 624, 15, 9500)
    assert r['temps']['tj'] == 61.2 and 'cv0' not in r['temps']   # -256C = off


def test_tegrastats_old_format():
    r = parse_tegrastats(OLD)
    assert (r['gpu_pct'], r['gpu_mhz']) == (7, 921)
    assert r['temps']['cpu'] == 42.5


def test_cpu_split():
    a = parse_proc_stat('cpu 100 0 50 800 50 0 0 0\ncpu0 1 0 1 1 0 0 0 0\nctxt 10\nprocs_running 3\n')
    b = parse_proc_stat('cpu 160 0 70 900 50 0 20 0\ncpu0 2 0 1 2 0 0 0 0\nctxt 110\nprocs_running 5\n')
    s = cpu_split(a['cpu'], b['cpu'])
    assert round(s['busy']) == 50 and round(s['user']) == 30 and round(s['sys']) == 10
    assert round(s['softirq']) == 10
    assert b['procs_running'] == 5 and b['ctxt'] - a['ctxt'] == 100 and b['cores'][0][0] == 0


def test_pid_stat_with_spaces_in_comm():
    fields = ['S'] + ['0'] * 10 + ['250', '30'] + ['0'] * 4 + ['7'] + ['0'] * 3 + ['1234'] + ['0'] * 20
    st = parse_pid_stat('4242 (python3 lane x) ' + ' '.join(fields))
    assert st == {'utime': 250, 'stime': 30, 'threads': 7, 'rss': 1234}
    assert parse_pid_stat('garbage') is None


def test_ctxt():
    assert parse_ctxt('Name:\tx\nvoluntary_ctxt_switches:\t12\nnonvoluntary_ctxt_switches:\t3\n') == (12, 3)


def test_names():
    lane = ['/usr/bin/python3', '/ws/install/clothoid_lane/lib/clothoid_lane/lane_main', '--ros-args',
            '-r', '__node:=lane_main', '--params-file', 'x.yaml']
    assert name_process(lane, 'lane_main') == ('lane_main', True)
    assert name_process(['/usr/bin/python3', '/usr/bin/ros2', 'bag', 'record', '-o', 'x'], 'ros2') == \
        ('ros2 bag record', True)
    assert name_process(['/usr/lib/xorg/Xorg', ':0'], 'Xorg') == ('Xorg', False)
    assert name_process(['python3', '-u', 'carmon.py', 'out'], 'python3') == ('carmon.py', False)
    assert name_process([], 'kworker/0:1') == ('kworker/0:1', False)
    assert name_process(['python3', '/t/harness.py', 'a.b:C', 'lane_main', '70', '--ros-args'], 'python3') \
        == ('lane_main', True)
    assert name_process(['/opt/app/app --type=renderer --lang=en-US'], 'app') == ('app', False)


def test_load_line():
    line = ('[static_avoid-15] [INFO] [1794667390.18] [static_avoid]: load 5s (idle): image n16 avg 9ms '
            'max 18ms | scan n50 avg 1ms max 6ms | odom gap max 33ms')
    p = parse_load_line(line)
    assert p['node'] == 'static_avoid' and p['state'] == 'idle'
    assert p['items']['image'] == {'n': 16, 'avg_ms': 9, 'max_ms': 18} and 'odom' not in p['items']


def test_percentile():
    assert percentile([1, 2, 3, 4, 5], 50) == 3
    assert percentile([], 50) != percentile([], 50)   # nan


def test_read_survives_bad_sysfs(tmp_path, monkeypatch):
    import builtins
    import carmon
    real_open = builtins.open

    class Broken:                       # what a sensor-off thermal zone does on the Jetson
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def read(self): raise TypeError("can't concat NoneType to bytes")

    monkeypatch.setattr(builtins, 'open', lambda p, *a, **k: Broken() if str(p).startswith('/sys/bad') else real_open(p, *a, **k))
    assert carmon.read('/sys/bad/temp', 'x') == 'x'
    good = tmp_path / 'temp'
    good.write_text('41000\n')
    assert carmon.read(str(good)).strip() == '41000'
