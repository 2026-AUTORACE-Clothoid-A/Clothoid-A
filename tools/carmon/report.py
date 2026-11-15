#!/usr/bin/env python3
"""Summary of a carmon folder (tables in Markdown, to paste into an issue or a doc).

    python3 tools/carmon/report.py ~/bags/carmon_1115_1012
    python3 tools/carmon/report.py DIR --from 60 --to 150      # seconds after the start: one lap
    python3 tools/carmon/report.py DIR --log ~/bags/1115_1012_full.log   # + static_avoid 'load 5s' lines
    python3 tools/carmon/report.py DIR --slow-hz 25            # lane slower than this = 'slow' second

CPU % is 100 per core (the car has 6: 600 % = everything).
"""
import argparse
import csv
import json
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from carmon_lib import parse_load_line, percentile  # noqa: E402

LANE = '/cmd_vel_lane'


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_csv(path):
    """Rows with 't' as a 0.1 s key, so the three files line up."""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r['t'] = f"{float(r['t']):.1f}"
    return rows


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float('nan')


def fmt(v, digits=0):
    if v is None or v != v:
        return '-'
    return f'{v:.{digits}f}'


def table(head, rows):
    out = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    out += ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows]
    return '\n'.join(out)


def section_meta(meta):
    lines = ['## 환경']
    lines.append(f"- 보드: {meta.get('model') or '?'}, 코어 {meta.get('online_cores')}, 커널 {meta.get('kernel')}")
    nv = meta.get('nvpmodel', '')
    mode = next((ln for ln in nv.splitlines() if 'Power Mode' in ln), nv.splitlines()[0] if nv else '-')
    lines.append(f'- nvpmodel: {mode}')
    lines.append(f"- governor: {', '.join(meta.get('governors') or ['-'])}, 코어 최대 MHz: "
                 f"{sorted(set((meta.get('max_mhz') or {}).values()))}")
    jc = meta.get('jetson_clocks', '-')
    lines.append(f'- jetson_clocks --show: {jc.splitlines()[0] if jc else "-"}')
    lines.append(f"- git: {meta.get('git')}, 시작 {meta.get('start')}, tegrastats {meta.get('tegrastats')}")
    return '\n'.join(lines)


def section_system(rows, ncores):
    busy = [fnum(r['busy']) for r in rows]
    mhz = [fnum(r.get(f'cpu{n}_mhz')) for r in rows for n in range(ncores)]
    mhz = [m for m in mhz if m]
    def stat(key, d=0):
        v = [fnum(r.get(key)) for r in rows]
        v = [x for x in v if x is not None]
        return fmt(mean(v), d), fmt(percentile(v, 95), d), fmt(max(v) if v else None, d)
    out = ['## 시스템 (초당 표본 %d개)' % len(rows)]
    rws = [['CPU 전체 %', *stat('busy')], ['  user %', *stat('user')], ['  sys %', *stat('sys')],
           ['  iowait %', *stat('iowait')], ['  softirq %', *stat('softirq')],
           ['run queue', *stat('procs_running')], ['ctxt/s', *stat('ctxt_per_s')],
           ['GPU %', *stat('gpu_pct')], ['EMC %', *stat('emc_pct')], ['RAM MB', *stat('ram_used_mb')],
           ['swap MB', *stat('swap_used_mb')], ['온도 최대 C', *stat('temp_max', 1)],
           ['전력 mW', *stat('power_mw')]]
    out.append(table(['항목', '평균', 'p95', '최대'], rws))
    low = sum(1 for r in rows if (fnum(r.get('low_clock_cores')) or 0) > 0)
    out.append(f'\n- 코어 클럭: 최소 {fmt(min(mhz) if mhz else None)} MHz, 평균 {fmt(mean(mhz))} MHz. '
               f'바쁜(>80 %) 코어가 최대 클럭의 90 % 미만이던 초: {low}/{len(rows)}'
               + (' → `sudo jetson_clocks` 확인' if low else ''))
    hot = sum(1 for b in busy if b is not None and b > 90)
    out.append(f'- CPU 전체 90 % 초과: {hot}/{len(rows)}초')
    return '\n'.join(out)


def proc_series(prows):
    """name -> {t: (cpu, user, sys, rss, nvcsw, ros)} summed over pids of the same name."""
    by = defaultdict(lambda: defaultdict(lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0]))
    for r in prows:
        v = by[r['name']][r['t']]
        v[0] += fnum(r['cpu']) or 0
        v[1] += fnum(r['user']) or 0
        v[2] += fnum(r['sys']) or 0
        v[3] += fnum(r['rss_mb']) or 0
        v[4] += fnum(r['nvcsw']) or 0
        v[5] = int(r['ros'])
    return by


def section_procs(by, times, top):
    n = len(times)
    rows = []
    for name, series in by.items():
        cpu = [series[t][0] if t in series else 0.0 for t in times]
        rows.append((mean(cpu), name, series, cpu))
    rows.sort(reverse=True)
    total_ros = sum(r[0] for r in rows if next(iter(r[2].values()))[5])
    total_other = sum(r[0] for r in rows if not next(iter(r[2].values()))[5])
    out = [f'## 프로세스 (CPU 100 % = 코어 1개, {n}초)',
           f'- ROS 합계 {total_ros:.0f} %, 그 밖의 프로세스(≥ --min-other) 합계 {total_other:.0f} %']
    body = []
    for avg, name, series, cpu in rows[:top]:
        vals = list(series.values())
        body.append([name, 'ros' if vals[0][5] else '', fmt(avg), fmt(percentile(cpu, 95)), fmt(max(cpu)),
                     fmt(mean([v[1] for v in vals])), fmt(mean([v[2] for v in vals])),
                     fmt(max(v[3] for v in vals)), fmt(mean([v[4] for v in vals]))])
    out.append(table(['프로세스', '', 'CPU 평균', 'p95', '최대', 'user', 'sys', 'RSS MB', '비자발 cs/s'], body))
    return '\n'.join(out)


def topic_series(trows):
    by = defaultdict(dict)
    for r in trows:
        by[r['topic']][r['t']] = (int(r['count']), fnum(r['max_gap_ms']))
    return by


def section_topics(tby, times):
    out = ['## 토픽 (초당 개수)']
    body = []
    for topic in sorted(tby):
        s = tby[topic]
        c = [s[t][0] if t in s else 0 for t in times]
        gaps = [s[t][1] for t in times if t in s and s[t][1] is not None]
        live = [x for x in c if x > 0]
        body.append([topic, fmt(mean(c), 1), fmt(percentile(live, 5)), fmt(min(live) if live else None),
                     fmt(max(gaps) if gaps else None), sum(1 for x in c if x == 0)])
    out.append(table(['토픽', '평균 Hz', 'p5', '최소', '최대 간격 ms', '0인 초'], body))
    return '\n'.join(out)


def section_slow(srows, pby, tby, times, slow_hz, top):
    lane = tby.get(LANE)
    if not lane:
        return f'## 느린 구간\n- {LANE} 기록 없음'
    c = [lane[t][0] if t in lane else 0 for t in times]
    # 3 s mean: one late frame is not a slow second
    sm = [mean(c[max(0, i - 1):i + 2]) for i in range(len(c))]
    slow = [t for t, v in zip(times, sm) if 0 < v < slow_hz]
    ok = [t for t, v in zip(times, sm) if v >= slow_hz]
    out = [f'## 느린 구간: {LANE} 3초 평균 < {slow_hz} Hz',
           f'- 느림 {len(slow)}초, 정상 {len(ok)}초, 출력 없음 {len(times) - len(slow) - len(ok)}초']
    if not slow or not ok:
        return '\n'.join(out)
    sidx = {r['t']: r for r in srows}
    def avg(ts, key):
        return mean([fnum(sidx[t].get(key)) for t in ts if t in sidx])
    def mhz(ts):
        return mean([fnum(sidx[t].get(k)) for t in ts if t in sidx for k in sidx[t] if k.endswith('_mhz')
                     and k.startswith('cpu')])
    body = [[k, fmt(avg(ok, k), 1), fmt(avg(slow, k), 1)] for k in
            ('busy', 'sys', 'softirq', 'procs_running', 'gpu_pct', 'temp_max')]
    body.append(['코어 MHz 평균', fmt(mhz(ok)), fmt(mhz(slow))])
    diffs = []
    for name, s in pby.items():
        a = mean([s[t][0] if t in s else 0.0 for t in ok])
        b = mean([s[t][0] if t in s else 0.0 for t in slow])
        diffs.append((b - a, name, a, b))
    diffs.sort(reverse=True)
    body += [[f'{name} CPU %', fmt(a), fmt(b)] for _d, name, a, b in diffs[:top]]
    out.append(table(['', '정상', '느림'], body))
    out.append('- 프로세스는 느린 구간에서 더 쓴 순서다. 위에 있는 것이 그때 코어를 가져간 쪽이다.')
    return '\n'.join(out)


def section_log(path):
    per = defaultdict(lambda: defaultdict(list))   # node -> item -> [(n, avg, max)]
    with open(path, errors='replace') as f:
        for line in f:
            p = parse_load_line(line)
            if p:
                for k, v in p['items'].items():
                    per[p['node']][k].append((v['n'], v['avg_ms'], v['max_ms']))
    if not per:
        return f'## 노드 load 로그\n- {path}: load 줄 없음'
    out = [f'## 노드 load 로그 ({os.path.basename(path)}, 5초 창)']
    body = []
    for node, items in per.items():
        for item, vals in items.items():
            n = [v[0] / 5.0 for v in vals]
            body.append([node, item, len(vals), fmt(percentile(n, 50), 1), fmt(min(n), 1),
                         fmt(percentile([v[1] for v in vals], 50)), fmt(max(v[2] for v in vals))])
    out.append(table(['노드', '콜백', '창 수', 'Hz 중앙', 'Hz 최소', 'avg ms 중앙', 'max ms'], body))
    return '\n'.join(out)


HEAVY = ('/camera/color/image_raw', '/camera/color/image_raw/compressed', '/camera/color/image_10hz')


def section_graph(path):
    """Who receives the camera, and which debug / visualisation outputs have a subscriber."""
    g = json.load(open(path))
    out = ['## 그래프 (graph.json)']
    for t in HEAVY:
        if t in g:
            out.append(f"- `{t}` 구독 {len(g[t]['sub'])}: {', '.join(g[t]['sub']) or '-'}")
    dbg = [(t, v) for t, v in g.items() if ('debug' in t or t.startswith('/avoid/')) and v['pub']]
    on = [f"`{t}` → {', '.join(v['sub'])}" for t, v in dbg if v['sub']]
    out.append(f'- 디버그 출력 중 구독자가 있는 것 {len(on)}/{len(dbg)}: ' + ('; '.join(on) if on else '없음'))
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir')
    ap.add_argument('--from', dest='t_from', type=float, default=0.0, help='s after the start')
    ap.add_argument('--to', dest='t_to', type=float, default=None, help='s after the start')
    ap.add_argument('--slow-hz', type=float, default=20.0)
    ap.add_argument('--top', type=int, default=20)
    ap.add_argument('--log', default='', help="launch log with static_avoid 'load 5s' lines")
    args = ap.parse_args()

    meta = json.load(open(os.path.join(args.dir, 'meta.json')))
    srows = load_csv(os.path.join(args.dir, 'system.csv'))
    if not srows:
        sys.exit('system.csv is empty')
    t0 = fnum(srows[0]['t'])
    keep = lambda t: args.t_from <= fnum(t) - t0 and (args.t_to is None or fnum(t) - t0 <= args.t_to)  # noqa
    srows = [r for r in srows if keep(r['t'])]
    times = [r['t'] for r in srows]
    tset = set(times)
    prows = [r for r in load_csv(os.path.join(args.dir, 'procs.csv')) if r['t'] in tset]
    trows = [r for r in load_csv(os.path.join(args.dir, 'topics.csv')) if r['t'] in tset]
    ncores = sum(1 for k in srows[0] if k.endswith('_pct') and k.startswith('cpu'))

    pby, tby = proc_series(prows), topic_series(trows)
    parts = [f'# carmon {os.path.basename(os.path.normpath(args.dir))}'
             f' ({fmt(args.t_from)}~{fmt(args.t_to) if args.t_to else "끝"} s)',
             section_meta(meta), section_system(srows, ncores), section_procs(pby, times, args.top),
             section_topics(tby, times), section_slow(srows, pby, tby, times, args.slow_hz, 8)]
    if os.path.exists(os.path.join(args.dir, 'graph.json')):
        parts.append(section_graph(os.path.join(args.dir, 'graph.json')))
    if args.log:
        parts.append(section_log(args.log))
    print('\n\n'.join(parts))


if __name__ == '__main__':
    main()
