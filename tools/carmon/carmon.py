#!/usr/bin/env python3
"""carmon: car load monitor. Read-only, runs next to the stack and records every second:

  system   CPU total split (user / sys / iowait / irq / softirq), per core % and MHz, RAM, swap,
           GPU %, EMC %, temperatures, input power (tegrastats; /proc and /sys without it)
  procs    CPU % (100 = one core) split user / sys, RSS, threads, context switches, per process,
           ROS nodes named by __node:=, other processes (desktop, rustdesk, ...) kept when >= 1 %
  topics   messages per second and the longest gap of a few topics (raw subscriptions, no
           deserialisation). The camera rate comes from camera_info: subscribing the raw image
           would load the car; --with-images adds image_10hz.

    tools/carmon/carmon.sh                 # until Ctrl-C, ~/bags/carmon_MMDD_HHMM
    tools/carmon/carmon.sh 120 /tmp/cm     # 120 s
    python3 tools/carmon/report.py ~/bags/carmon_MMDD_HHMM [--log ~/bags/MMDD_HHMM_full.log]

Output: meta.json (power mode, clocks, governors, git), system.csv, procs.csv, topics.csv,
graph.json (publishers and subscribers of every topic, written after 5 s and 60 s).
"""
import argparse
import csv
import glob
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from carmon_lib import (cpu_split, name_process, parse_ctxt, parse_pid_stat,  # noqa: E402
                        parse_proc_stat, parse_tegrastats)

TOPICS = ['/cmd_vel_lane', '/cmd_vel', '/detections', '/camera/color/camera_info', '/scan', '/odom',
          '/imu']
IMAGE_TOPICS = ['/camera/color/image_10hz']
SHORT = {'/cmd_vel_lane': 'lane', '/cmd_vel': 'cmd', '/detections': 'yolo',
         '/camera/color/camera_info': 'cam', '/scan': 'scan', '/odom': 'odom', '/imu': 'imu',
         '/camera/color/image_10hz': 'img10'}
TICK = os.sysconf('SC_CLK_TCK')
PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1e6


def read(path, default=''):
    """Text of a /proc or /sys file, `default` when it cannot be read. Binary + decode: some Jetson
    sysfs files (thermal zones of sensors that are off) fail inside a text-mode read with a
    TypeError instead of an OSError (limo2, 2026-11-15)."""
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except (OSError, TypeError, ValueError):
        return default
    return data.decode('utf-8', 'replace') if data is not None else default


def run(cmd, timeout=5.0):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (r.stdout + r.stderr).strip(), r.returncode
    except (OSError, subprocess.TimeoutExpired) as e:
        return str(e), -1


# ------------------------------------------------------------------ system
class Tegrastats:
    """tegrastats in the background; `last` is the newest parsed line (None without tegrastats)."""

    def __init__(self):
        self.last = None
        self.proc = None
        exe = shutil.which('tegrastats')
        if exe:
            self.proc = subprocess.Popen([exe, '--interval', '1000'], stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL, text=True, bufsize=1)
            threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.last = parse_tegrastats(line)

    def stop(self):
        if self.proc:
            self.proc.terminate()


def core_freqs():
    """{core: (cur MHz, max MHz)} from cpufreq; offline cores are left out."""
    out = {}
    for d in glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq'):
        n = int(d.split('/')[-2][3:])
        cur, mx = read(d + '/scaling_cur_freq').strip(), read(d + '/cpuinfo_max_freq').strip()
        if cur.isdigit():
            out[n] = (int(cur) // 1000, int(mx) // 1000 if mx.isdigit() else None)
    return out


def thermal():
    out = {}
    for z in glob.glob('/sys/class/thermal/thermal_zone*'):
        t, kind = read(z + '/temp').strip(), read(z + '/type').strip()
        if t.lstrip('-').isdigit() and int(t) > -50000:
            out[kind.replace('-thermal', '').lower()] = int(t) / 1000.0
    return out


def meminfo():
    vals = {}
    for line in read('/proc/meminfo').splitlines():
        k, _, v = line.partition(':')
        if v.strip().split()[0:1]:
            vals[k] = int(v.split()[0]) // 1024
    used = vals.get('MemTotal', 0) - vals.get('MemAvailable', 0)
    return used, vals.get('MemTotal', 0), vals.get('SwapTotal', 0) - vals.get('SwapFree', 0)


class System:
    def __init__(self, tegra: Tegrastats):
        self.tegra = tegra
        self.prev = parse_proc_stat(read('/proc/stat'))
        self.ncores = len(self.prev['cores'])

    def sample(self):
        cur = parse_proc_stat(read('/proc/stat'))
        row = cpu_split(self.prev['cpu'], cur['cpu'])
        row['procs_running'] = cur.get('procs_running', 0)
        row['ctxt_per_s'] = cur.get('ctxt', 0) - self.prev.get('ctxt', 0)
        row['load1'] = float(read('/proc/loadavg', '0').split()[0])
        prev_cores = dict(self.prev['cores'])
        freqs = core_freqs()
        low = 0
        for n, vals in cur['cores']:
            busy = cpu_split(prev_cores.get(n, vals), vals)['busy']
            cur_mhz, max_mhz = freqs.get(n, (None, None))
            row[f'cpu{n}_pct'] = round(busy, 1)
            row[f'cpu{n}_mhz'] = cur_mhz
            # a busy core below its top clock: governor / power cap / heat
            if busy > 80 and cur_mhz and max_mhz and cur_mhz < 0.9 * max_mhz:
                low += 1
        row['low_clock_cores'] = low
        self.prev = cur

        used, total, swap = meminfo()
        row.update(ram_used_mb=used, ram_total_mb=total, swap_used_mb=swap)
        ts = self.tegra.last
        temps = ts.get('temps', {}) if ts else {}
        if not temps:
            temps = thermal()          # no tegrastats (PC) or it has not printed yet
        if ts:
            row['gpu_pct'] = ts.get('gpu_pct')
            row['gpu_mhz'] = ts.get('gpu_mhz')
            row['emc_pct'] = ts.get('emc_pct')
            row['power_mw'] = ts.get('power_mw')
            for k in ('ram_used_mb', 'ram_total_mb', 'swap_used_mb'):
                if k in ts:
                    row[k] = ts[k]
        row['temp_max'] = max(temps.values()) if temps else None
        for want in ('cpu', 'gpu', 'tj'):
            row[f'temp_{want}'] = next((v for k, v in temps.items() if k.startswith(want)), None)
        return row


# ------------------------------------------------------------------ processes
class Procs:
    def __init__(self, min_other: float):
        self.min_other = min_other
        self.prev = {}     # pid -> (utime, stime, vcsw, nvcsw)
        self.names = {}    # pid -> (name, is_ros)
        self.t_prev = time.monotonic()

    def _name(self, pid):
        if pid not in self.names:
            argv = [a for a in read(f'/proc/{pid}/cmdline').split('\0') if a]
            comm = read(f'/proc/{pid}/comm').strip()
            self.names[pid] = name_process(argv, comm)
        return self.names[pid]

    def sample(self):
        now = time.monotonic()
        dt = max(now - self.t_prev, 1e-3)
        self.t_prev = now
        rows, seen = [], {}
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            pid = int(d)
            st = parse_pid_stat(read(f'/proc/{pid}/stat'))
            if st is None:
                continue
            name, is_ros = self._name(pid)
            prev = self.prev.get(pid)
            seen[pid] = (st['utime'], st['stime'], prev[2] if prev else 0, prev[3] if prev else 0)
            if prev is None:
                continue
            user = 100.0 * (st['utime'] - prev[0]) / TICK / dt
            sys_ = 100.0 * (st['stime'] - prev[1]) / TICK / dt
            if not is_ros and user + sys_ < self.min_other:
                continue
            vol, nonvol = parse_ctxt(read(f'/proc/{pid}/status'))
            seen[pid] = (st['utime'], st['stime'], vol, nonvol)
            rows.append({'pid': pid, 'name': name, 'ros': int(is_ros),
                         'cpu': round(user + sys_, 1), 'user': round(user, 1), 'sys': round(sys_, 1),
                         'rss_mb': round(st['rss'] * PAGE_MB, 1), 'threads': st['threads'],
                         # first sample of a process: totals, not rates
                         'vcsw': round((vol - prev[2]) / dt) if prev[2] else '',
                         'nvcsw': round((nonvol - prev[3]) / dt) if prev[3] else ''})
        self.prev = seen
        self.names = {p: v for p, v in self.names.items() if p in seen}
        return rows


# ------------------------------------------------------------------ topics
class Topics:
    """Message counts without an executor: each subscription keeps up to `depth` messages and take()
    drains them once a second, using DDS reception times for the gaps. One wake-up per second instead
    of one callback per message (rclpy callbacks cost ~10 % of a core at 200 msg/s on the PC).
    Types come from the graph, so topics that appear later are picked up on the next take()."""

    def __init__(self, topics, depth=300):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node('carmon')
        self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                              reliability=qos_profile_sensor_data.reliability,
                              durability=qos_profile_sensor_data.durability)
        self.want = list(topics)
        self.subs = {}
        self.last = {t: None for t in topics}     # reception time of the newest message, s
        self.n_resolve = 0

    def resolve(self):
        from rosidl_runtime_py.utilities import get_message
        types = dict(self.node.get_topic_names_and_types())
        for t in list(self.want):
            if t in self.subs or t not in types:
                continue
            try:
                cls = get_message(types[t][0])
            except (AttributeError, ModuleNotFoundError, ValueError) as e:
                print(f'carmon: {t}: {e}', file=sys.stderr)
                self.want.remove(t)
                continue
            self.subs[t] = self.node.create_subscription(cls, t, lambda _m: None, self.qos, raw=True)

    def _drain(self, sub):
        """Reception times (s) of every queued message."""
        times = []
        while True:
            with sub.handle:
                got = sub.handle.take_message(sub.msg_type, True)
            if got is None:
                return times
            info = got[1] or {}
            ns = info.get('received_timestamp') or info.get('source_timestamp') or 0
            times.append(ns * 1e-9 if ns else time.time())

    def take(self):
        if self.n_resolve % 2 == 0:
            self.resolve()
        self.n_resolve += 1
        now = time.time()
        out = {}
        for t in self.want:
            sub = self.subs.get(t)
            times = sorted(self._drain(sub)) if sub else []
            prev = self.last[t]
            gap = None
            for x in times:
                if prev is not None:
                    gap = max(gap or 0.0, x - prev)
                prev = x
            if prev is not None:
                gap = max(gap or 0.0, now - prev)            # a topic that went silent
            self.last[t] = prev
            out[t] = (len(times), gap)
        return out

    def graph(self):
        """Every topic with its publishing and subscribing nodes: who takes the raw image, which
        debug outputs are on, a remote subscriber (rviz / bag on the PC over Wi-Fi)."""
        out = {}
        for topic, types in sorted(self.node.get_topic_names_and_types()):
            name = lambda i: f'{i.node_namespace.rstrip("/")}/{i.node_name}'  # noqa: E731
            out[topic] = {
                'type': types[0],
                'pub': sorted(name(i) for i in self.node.get_publishers_info_by_topic(topic)),
                'sub': sorted(name(i) for i in self.node.get_subscriptions_info_by_topic(topic)
                              if i.node_name != 'carmon')}
        return out

    def stop(self):
        try:
            self.node.destroy_node()
            self.rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


# ------------------------------------------------------------------ main
def meta(args, tegra):
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    out = {
        'start': time.strftime('%Y-%m-%d %H:%M:%S'),
        'host': socket.gethostname(),
        'kernel': os.uname().release,
        'git': run(['git', '-C', repo, 'log', '-1', '--format=%h %ci %s'])[0],
        'tegrastats': tegra.proc is not None,
        'args': vars(args),
        'governors': sorted({read(g).strip() for g in
                             glob.glob('/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor')}),
        'max_mhz': {n: m for n, (_c, m) in core_freqs().items()},
        'online_cores': read('/sys/devices/system/cpu/online').strip(),
        'model': read('/proc/device-tree/model').strip('\0\n '),
    }
    if shutil.which('nvpmodel'):
        out['nvpmodel'] = run(['nvpmodel', '-q'])[0]
    if shutil.which('jetson_clocks'):
        text, rc = run(['sudo', '-n', 'jetson_clocks', '--show'])
        out['jetson_clocks'] = text if rc == 0 else 'not read (needs sudo): run `sudo jetson_clocks --show`'
    return out


def console(sysrow, procs, topics, ncores):
    temp = f" {sysrow['temp_max']:.0f}C" if sysrow.get('temp_max') else ''
    gpu = f" GPU {sysrow['gpu_pct']}%" if sysrow.get('gpu_pct') is not None else ''
    cores = [sysrow.get(f'cpu{n}_pct') or 0 for n in range(ncores)]
    mhz = [sysrow.get(f'cpu{n}_mhz') or 0 for n in range(ncores)]
    low = f" LOWCLK {sysrow['low_clock_cores']}" if sysrow['low_clock_cores'] else ''
    ram = f" RAM {sysrow['ram_used_mb'] / 1024:.1f}/{sysrow['ram_total_mb'] / 1024:.1f}G"
    rates = ' '.join(f'{SHORT.get(t, t)} {c}' for t, (c, _g) in topics.items()) if topics else ''
    top = sorted(procs, key=lambda r: -r['cpu'])[:4]
    tops = ' '.join(f"{r['name']} {r['cpu']:.0f}" for r in top)
    return (f"CPU {sysrow['busy']:4.0f}% (sys {sysrow['sys']:3.0f}, max core {max(cores):3.0f}% "
            f"{min(m for m in mhz if m) if any(mhz) else 0}-{max(mhz)}MHz){gpu}{ram}{temp}{low} | "
            f"{rates} | {tops}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', nargs='?', default=os.path.expanduser(
        time.strftime('~/bags/carmon_%m%d_%H%M')))
    ap.add_argument('--duration', type=float, default=0.0, help='s, 0 = until Ctrl-C')
    ap.add_argument('--no-ros', action='store_true', help='system and processes only')
    ap.add_argument('--with-images', action='store_true',
                    help='also count image_10hz (costs a raw image subscription)')
    ap.add_argument('--topics', default='', help='comma list replacing the default topics')
    ap.add_argument('--min-other', type=float, default=1.0,
                    help='keep non-ROS processes at or above this CPU %%')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    tegra = Tegrastats()
    system = System(tegra)
    procs = Procs(args.min_other)
    procs.sample()
    topics = None
    if not args.no_ros:
        want = [t for t in args.topics.split(',') if t] or TOPICS + (IMAGE_TOPICS if args.with_images else [])
        try:
            topics = Topics(want)
        except Exception as e:  # noqa: BLE001  (no ROS sourced)
            print(f'carmon: no topic rates ({e})', file=sys.stderr)
    with open(os.path.join(args.out, 'meta.json'), 'w') as f:
        json.dump(meta(args, tegra), f, indent=1, ensure_ascii=False)

    sys_cols = ['t', 'busy', 'user', 'sys', 'iowait', 'irq', 'softirq', 'procs_running', 'ctxt_per_s',
                'load1', 'ram_used_mb', 'ram_total_mb', 'swap_used_mb', 'gpu_pct', 'gpu_mhz', 'emc_pct',
                'power_mw', 'temp_max', 'temp_cpu', 'temp_gpu', 'temp_tj', 'low_clock_cores']
    sys_cols += [f'cpu{n}_{k}' for n in range(system.ncores) for k in ('pct', 'mhz')]
    proc_cols = ['t', 'pid', 'name', 'ros', 'cpu', 'user', 'sys', 'rss_mb', 'threads', 'vcsw', 'nvcsw']
    files = {k: open(os.path.join(args.out, f'{k}.csv'), 'w', newline='') for k in ('system', 'procs', 'topics')}
    w_sys = csv.DictWriter(files['system'], sys_cols, extrasaction='ignore')
    w_proc = csv.DictWriter(files['procs'], proc_cols)
    w_top = csv.writer(files['topics'])
    w_sys.writeheader()
    w_proc.writeheader()
    w_top.writerow(['t', 'topic', 'count', 'max_gap_ms'])
    print(f'carmon: writing {args.out} (Ctrl-C to stop)', file=sys.stderr)

    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    nxt = time.monotonic() + 1.0
    n_samples = 0
    try:
        while t_end is None or time.monotonic() < t_end:
            time.sleep(max(nxt - time.monotonic(), 0.0))
            nxt += 1.0
            t = round(time.time(), 3)
            srow = system.sample()
            srow['t'] = t
            prow = procs.sample()
            trow = topics.take() if topics else {}
            w_sys.writerow({k: (round(v, 1) if isinstance(v, float) and k != 't' else v)
                            for k, v in srow.items()})
            for r in prow:
                w_proc.writerow({'t': t, **r})
            for topic, (c, gap) in trow.items():
                w_top.writerow([t, topic, c, round(gap * 1000) if gap is not None else ''])
            for f in files.values():
                f.flush()
            if not args.quiet:
                print(console(srow, prow, trow, system.ncores), flush=True)
            n_samples += 1
            if topics and n_samples in (5, 60):          # after discovery, again once all is up
                with open(os.path.join(args.out, 'graph.json'), 'w') as f:
                    json.dump(topics.graph(), f, indent=1)
    except KeyboardInterrupt:
        pass
    finally:
        tegra.stop()
        if topics:
            topics.stop()
        for f in files.values():
            f.close()
        print(f'carmon: saved {args.out}\n  python3 {os.path.dirname(os.path.abspath(__file__))}/report.py '
              f'{args.out}', file=sys.stderr)


if __name__ == '__main__':
    main()
