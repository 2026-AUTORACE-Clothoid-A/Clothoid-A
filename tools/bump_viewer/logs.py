"""Recorded diagnostics and state evidence. Reading this module never runs ROS nodes."""
from pathlib import Path
import json
import re
import sqlite3

ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
ROS = re.compile(r'\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s*\[(\d+\.\d+)\]\s*\[([^]]+)\]:\s*(.*)')
LEVEL = re.compile(r'\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]')
PREFIX = re.compile(r'^\[([^]]+)\]')
STATE_KEYS = ('state', 'armed', 'seen', 'bump_state', 'entry_phase', 'line_phase',
              'parking_prepare', 'parking_scene', 'parking_observing', 'parking_skipped',
              'cone_entered', 'steering_lock', 'entry_reached')
SIGNALS = {'/parking_flag', '/parking_done', '/parking_sign', '/parking/observing',
           '/parking/skipped', '/parking/prepare', '/cone/mission_active', '/cone/exiting',
           '/cone/exit_complete', '/avoid/active', '/avoid/done', '/finish/done',
           '/steering_lock', '/bump/approaching', '/stopline_near', '/direction_flag'}


def entry(t, source, node, level, message, raw=None, **extra):
    return dict(t=t, source=source, node=node, level=level.replace('WARNING', 'WARN'),
                message=message, raw=message if raw is None else raw, **extra)


def read_file(path, start):
    rows = []
    for number, raw in enumerate(Path(path).read_text(errors='replace').splitlines(), 1):
        line = ANSI.sub('', raw)
        match = ROS.search(line)
        if match:
            level, stamp, node, message = match.groups()
            t = float(stamp)-start*1e-9
        else:
            severity, prefix = LEVEL.search(line), PREFIX.search(line)
            level = severity.group(1) if severity else 'TEXT'
            node = prefix.group(1) if prefix else 'launch / 원문'
            if node in ('INFO', 'WARN', 'WARNING', 'ERROR', 'DEBUG', 'FATAL'):
                node = 'launch'
            message, t = line, None
        rows.append(entry(t, '실제 파일', node, level, message, raw,
                          location=f'{path}:{number}'))
    return rows


def merge(rows):
    """Merge only cross-source exact-message duplicates with matching node and timestamp.

    File lines are never dropped, including repeated and untimed traceback lines.
    """
    result = [dict(row) for row in rows if row['source'] != '실제 /rosout']
    files = {}
    for row in result:
        if row['source'] == '실제 파일' and row['t'] is not None:
            files.setdefault((row['node'], row['message']), []).append(row)
    for row in rows:
        if row['source'] != '실제 /rosout':
            continue
        same = next((r for r in files.get((row['node'], row['message']), [])
                     if abs(r['t']-row['t']) <= .01 and 'rosout_raw' not in r), None)
        if same is not None:
            same['source'] = '실제 파일 + /rosout'
            same['rosout_raw'] = row['raw']
        else:
            result.append(dict(row))
    return sorted(result, key=lambda r: (r['t'] is None, r['t'] if r['t'] is not None else 0))


def recorded(files, start):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rows, states, signals = [], [], []
    previous = {}
    streams = []
    for file in files:
        with sqlite3.connect(f'file:{file}?mode=ro', uri=True) as db:
            topics = {i: (n, get_message(k)) for i, n, k in db.execute('select id,name,type from topics')
                      if n in SIGNALS | {'/rosout', '/mission/state'}}
            if not topics:
                continue
            query = 'select timestamp,topic_id,data from messages where topic_id in ('+','.join('?' for _ in topics)+') order by timestamp,id'
            for stamp, tid, blob in db.execute(query, tuple(topics)):
                name, cls = topics[tid]
                streams.append((stamp, name, deserialize_message(blob, cls)))
    for stamp, name, msg in sorted(streams, key=lambda r: r[0]):
        t = (stamp-start)*1e-9
        if name == '/rosout':
            source_stamp = msg.stamp.sec+msg.stamp.nanosec*1e-9
            t = source_stamp-start*1e-9 if source_stamp else t
            level = {10:'DEBUG',20:'INFO',30:'WARN',40:'ERROR',50:'FATAL'}.get(msg.level, str(msg.level))
            rows.append(entry(t, '실제 /rosout', msg.name, level, msg.msg,
                              location=f'{msg.file}:{msg.line} {msg.function}'))
        elif name == '/mission/state':
            try:
                state = json.loads(msg.data)
                if not isinstance(state, dict):
                    raise ValueError('state is not an object')
            except (ValueError, TypeError):
                rows.append(entry(t, '실제 상태', name, 'WARN', '상태 해석 실패', str(msg.data)))
                continue
            states.append((t, state))
            key = {k: state[k] for k in STATE_KEYS if k in state}
            if previous.get(name) != key:
                rows.append(entry(t, '실제 상태', name, 'STATE',
                                  json.dumps(key, ensure_ascii=False), msg.data))
                previous[name] = key
        else:
            value = getattr(msg, 'data', None)
            signals.append((t, name, value))
            if name not in previous or previous[name] != value:
                rows.append(entry(t, '실제 토픽', name, 'STATE', f'{name} = {value}', str(msg)))
                previous[name] = value
    return rows, states, signals


def load(files, start, bag):
    rows, states, signals = recorded(files, start)
    path = Path(bag)
    log = path.with_suffix('.log') if path.is_dir() else path.parent.with_suffix('.log')
    file_rows = read_file(log, start) if log.is_file() else []
    return dict(recorded_logs=rows, file_logs=file_rows, log_file=str(log) if log.is_file() else None,
                recorded_states=states, recorded_signals=signals, start_ns=start)
