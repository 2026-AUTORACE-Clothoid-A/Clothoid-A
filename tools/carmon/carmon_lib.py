"""ROS-free parsing for carmon: tegrastats lines, /proc/stat, /proc/<pid>/stat, process names.

Everything here takes text and returns plain values so it can be tested on the PC with saved samples.
"""
import os
import re
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------- tegrastats
# Orin (JetPack 5/6):
#   RAM 4100/7620MB (lfb 2x4MB) SWAP 150/3810MB (cached 0MB) CPU [62%@1510,off,...] EMC_FREQ 15%@2133
#   GR3D_FREQ 45%@[624,624] ... CPU@60.1C GPU@57.5C tj@61.2C ... VDD_IN 9500mW/9400mW ...
# older: GR3D_FREQ 45%@624, temperatures as 'CPU@60.1C'
_RAM = re.compile(r'RAM (\d+)/(\d+)MB')
_SWAP = re.compile(r'SWAP (\d+)/(\d+)MB')
_CPU = re.compile(r'CPU \[([^\]]*)\]')
_GR3D = re.compile(r'GR3D_FREQ (\d+)%(?:@\[?([\d,]+)\]?)?')
_EMC = re.compile(r'EMC_FREQ (\d+)%')
_TEMP = re.compile(r'([A-Za-z0-9_]+)@(-?[\d.]+)C\b')
_POWER = re.compile(r'(VDD_IN|POM_5V_IN) (\d+)mW')


def parse_tegrastats(line: str) -> Dict[str, object]:
    """One tegrastats line -> dict. Missing fields are left out."""
    out: Dict[str, object] = {}
    m = _RAM.search(line)
    if m:
        out['ram_used_mb'], out['ram_total_mb'] = int(m.group(1)), int(m.group(2))
    m = _SWAP.search(line)
    if m:
        out['swap_used_mb'] = int(m.group(1))
    m = _CPU.search(line)
    if m:
        cores: List[Tuple[Optional[float], Optional[int]]] = []
        for item in m.group(1).split(','):
            item = item.strip()
            if item == 'off' or not item:
                cores.append((None, None))
                continue
            pct, _, mhz = item.partition('@')
            cores.append((float(pct.rstrip('%')), int(mhz) if mhz.isdigit() else None))
        out['cores'] = cores
    m = _GR3D.search(line)
    if m:
        out['gpu_pct'] = int(m.group(1))
        if m.group(2):
            out['gpu_mhz'] = max(int(x) for x in m.group(2).split(',') if x)
    m = _EMC.search(line)
    if m:
        out['emc_pct'] = int(m.group(1))
    temps = {}
    for name, val in _TEMP.findall(line):
        if name in ('GR3D_FREQ', 'EMC_FREQ'):
            continue
        temps[name.lower()] = float(val)
    temps = {k: v for k, v in temps.items() if v > -50}   # -256C: sensor off
    if temps:
        out['temps'] = temps
    m = _POWER.search(line)
    if m:
        out['power_mw'] = int(m.group(2))
    return out


# ---------------------------------------------------------------- /proc/stat
CPU_FIELDS = ('user', 'nice', 'system', 'idle', 'iowait', 'irq', 'softirq', 'steal')


def parse_proc_stat(text: str) -> Dict[str, object]:
    """/proc/stat -> {'cpu': {field: jiffies}, 'cores': [{...}], 'ctxt': n, 'procs_running': n}."""
    out: Dict[str, object] = {'cores': []}
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0].startswith('cpu'):
            vals = dict(zip(CPU_FIELDS, (int(x) for x in parts[1:1 + len(CPU_FIELDS)])))
            if parts[0] == 'cpu':
                out['cpu'] = vals
            else:
                out['cores'].append((int(parts[0][3:]), vals))
        elif parts[0] == 'ctxt':
            out['ctxt'] = int(parts[1])
        elif parts[0] == 'procs_running':
            out['procs_running'] = int(parts[1])
    return out


def cpu_split(prev: Dict[str, int], cur: Dict[str, int]) -> Dict[str, float]:
    """Two /proc/stat cpu rows -> % of the time in each group (user includes nice) and 'busy'."""
    d = {k: cur.get(k, 0) - prev.get(k, 0) for k in CPU_FIELDS}
    total = sum(d.values())
    if total <= 0:
        return {k: 0.0 for k in ('busy', 'user', 'sys', 'iowait', 'irq', 'softirq')}
    pct = lambda v: 100.0 * v / total  # noqa: E731
    return {
        'busy': pct(total - d['idle'] - d['iowait']),
        'user': pct(d['user'] + d['nice']),
        'sys': pct(d['system']),
        'iowait': pct(d['iowait']),
        'irq': pct(d['irq']),
        'softirq': pct(d['softirq']),
    }


# ---------------------------------------------------------------- /proc/<pid>
def parse_pid_stat(text: str) -> Optional[Dict[str, int]]:
    """/proc/<pid>/stat -> utime, stime (ticks), threads, rss (pages). comm may contain spaces."""
    r = text.rfind(')')
    if r < 0:
        return None
    rest = text[r + 2:].split()
    # rest[0] is field 3 (state); utime is field 14 -> rest[11]
    try:
        return {'utime': int(rest[11]), 'stime': int(rest[12]),
                'threads': int(rest[17]), 'rss': int(rest[21])}
    except (IndexError, ValueError):
        return None


def parse_ctxt(status_text: str) -> Tuple[int, int]:
    """/proc/<pid>/status -> (voluntary, nonvoluntary) context switches."""
    vol = nonvol = 0
    for line in status_text.splitlines():
        if line.startswith('voluntary_ctxt_switches:'):
            vol = int(line.split()[1])
        elif line.startswith('nonvoluntary_ctxt_switches:'):
            nonvol = int(line.split()[1])
    return vol, nonvol


_INTERPRETERS = ('python', 'python3', 'bash', 'sh')


def name_process(argv: List[str], comm: str) -> Tuple[str, bool]:
    """(display name, is_ros) from the command line. ROS nodes are named by __node:=."""
    is_ros = '--ros-args' in argv
    for i, a in enumerate(argv):
        if a.startswith('__node:='):
            return a.split(':=', 1)[1], True
        if a == '-r' and i + 1 < len(argv) and argv[i + 1].startswith('__node:='):
            return argv[i + 1].split(':=', 1)[1], True
    if not argv:
        return comm, False
    if len(argv) == 1 and ' ' in argv[0]:
        argv = argv[0].split()          # Electron / Chrome rewrite their argv as one string
    exe = os.path.basename(argv[0])
    if exe.rstrip('0123456789.') in _INTERPRETERS and len(argv) > 1:
        for a in argv[1:]:
            if not a.startswith('-'):
                base = os.path.basename(a)
                if base == 'ros2':
                    # `ros2 bag record`, `ros2 launch ...`, `ros2 topic hz`
                    sub = [x for x in argv[argv.index(a) + 1:] if not x.startswith('-')][:2]
                    return 'ros2 ' + ' '.join(sub), True
                j = argv.index(a)
                if base == 'harness.py' and len(argv) > j + 2:
                    return argv[j + 2], True         # tools/load_profile: harness.py spec LABEL secs
                return base, is_ros
    return (exe or comm)[:40], is_ros


# ---------------------------------------------------------------- statistics
def percentile(values: List[float], q: float) -> float:
    if not values:
        return float('nan')
    s = sorted(values)
    k = (len(s) - 1) * q / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


_LOAD = re.compile(r'\[(\w+)\]: load 5s \((\w+)\): (.*)')
_LOAD_ITEM = re.compile(r'(\w+) n(\d+) avg (\d+)ms max (\d+)ms')


def parse_load_line(line: str) -> Optional[Dict[str, object]]:
    """static_avoid's 'load 5s (state): image n16 avg 9ms max 18ms | ...' -> per-callback counts."""
    m = _LOAD.search(line)
    if not m:
        return None
    items = {name: {'n': int(n), 'avg_ms': int(a), 'max_ms': int(mx)}
             for name, n, a, mx in _LOAD_ITEM.findall(m.group(3))}
    return {'node': m.group(1), 'state': m.group(2), 'items': items}
