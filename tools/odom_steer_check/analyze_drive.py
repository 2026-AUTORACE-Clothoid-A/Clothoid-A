#!/usr/bin/env python3
"""Odometry / steering check from a test bag (docs/ODOM_STEERING_TEST.md).

Reads /cmd_vel, /odom and /imu, cuts the bag into runs of constant /cmd_vel and prints:
  * straight runs: /odom distance (speed integral). Compare with the tape (--tape).
  * curved runs: the actual central steering angle from the IMU gyro, the angle /odom assumes,
    and the steering_scale that would make /odom match the gyro.

limo_base model (src/limo_driver.cpp): /cmd_vel -> central angle atan(L*wz/v) -> inner wheel angle
(Ackermann, track 0.172) -> clamp max_inner_angle -> value = inner / steering_scale to the chassis.
/odom wz is computed back from the chassis feedback value with the same scale. So
    true scale = scale_used * inner(gyro) / inner(odom)
when the odom speed is right. Usage:
    python3 analyze_drive.py <bag_dir> [--scale 2.47] [--tape 1:2.00,2:1.98] [--plot out.png]
"""
import argparse
import math
import sys

import numpy as np

L = 0.2          # wheelbase
TRACK = 0.172    # left-right wheel distance


def central_to_inner(c):
    c = np.asarray(c, dtype=float)
    a = np.abs(c)
    inner = np.arctan(2 * L * np.sin(a) / (2 * L * np.cos(a) - TRACK * np.sin(a)))
    return np.sign(c) * inner


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    types = {'/cmd_vel': Twist, '/odom': Odometry, '/imu': Imu}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    cmd, odom, imu = [], [], []
    t0 = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        t0 = t if t0 is None else t0
        s = (t - t0) * 1e-9
        m = deserialize_message(data, types[topic])
        if topic == '/cmd_vel':
            cmd.append((s, m.linear.x, m.angular.z))
        elif topic == '/odom':
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            odom.append((s, m.twist.twist.linear.x, m.twist.twist.angular.z, yaw))
        else:
            q = m.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            imu.append((s, m.angular_velocity.z, yaw))
    return np.array(cmd).reshape(-1, 3), np.array(odom).reshape(-1, 4), np.array(imu).reshape(-1, 3)


def runs(cmd, gap=0.6):
    """Consecutive /cmd_vel with the same (v, wz) and v != 0 -> (t_start, t_end, v, wz)."""
    out = []
    start = None
    for i, (t, v, w) in enumerate(cmd):
        same = start is not None and abs(v - cmd[start, 1]) < 1e-4 and abs(w - cmd[start, 2]) < 1e-4 \
            and t - cmd[i - 1, 0] < gap
        if not same:
            if start is not None and abs(cmd[start, 1]) > 1e-3:
                out.append((cmd[start, 0], cmd[i - 1, 0], cmd[start, 1], cmd[start, 2]))
            start = i
    if start is not None and abs(cmd[start, 1]) > 1e-3:
        out.append((cmd[start, 0], cmd[-1, 0], cmd[start, 1], cmd[start, 2]))
    return out


def runs_from_odom(odom, imu, v_min=0.03, min_len=2.0):
    """Fallback when /cmd_vel was not recorded: moving stretches of /odom speed, cmd unknown (nan).
    The gyro decides straight (|wz| < 0.05 rad/s) or curved; wz is the mean gyro then."""
    moving = np.abs(odom[:, 1]) > v_min
    out = []
    i = 0
    n = len(odom)
    while i < n:
        if not moving[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and moving[j + 1]:
            j += 1
        t0, t1 = odom[i, 0], odom[j, 0]
        if t1 - t0 >= min_len:
            g = mean(imu, 1, t0, t1) if len(imu) else 0.0
            out.append((t0, t1, float('nan'), 0.0 if abs(g) < 0.05 else float('nan')))
        i = j + 1
    return out


def integrate(tab, col, t0, t1):
    m = (tab[:, 0] >= t0) & (tab[:, 0] <= t1)
    if m.sum() < 2:
        return float('nan')
    return float(np.trapz(tab[m, col], tab[m, 0]))


def mean(tab, col, t0, t1):
    m = (tab[:, 0] >= t0) & (tab[:, 0] <= t1)
    return float(np.mean(tab[m, col])) if m.any() else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--scale', type=float, default=2.47, help='steering_scale limo_base ran with')
    ap.add_argument('--max-inner', type=float, default=0.48869, help='max_inner_angle limo_base ran with')
    ap.add_argument('--settle', type=float, default=1.0, help='seconds skipped at the start of a run')
    ap.add_argument('--tape', default='', help='tape distances per straight run, e.g. "1:2.00,2:1.98"')
    ap.add_argument('--plot', default='', help='save a time plot here')
    a = ap.parse_args()

    cmd, odom, imu = read_bag(a.bag)
    if len(odom) == 0:
        sys.exit('no /odom in the bag')
    tape = {int(k): float(v) for k, v in (p.split(':') for p in a.tape.split(',') if p)}
    if len(cmd):
        rs = runs(cmd)
    else:
        print('no /cmd_vel in the bag: runs are cut from /odom speed, cmd columns are nan')
        rs = runs_from_odom(odom, imu)
    print(f'{a.bag}: {len(rs)} runs, /odom {len(odom)} msgs, /imu {len(imu)} msgs, scale used {a.scale}\n')

    straight, curved = [], []
    for k, (t0, t1, v, w) in enumerate(rs, 1):
        (straight if abs(w) < 1e-3 else curved).append((k, t0, t1, v, w))

    if straight:
        print('STRAIGHT  run  t[s]          cmd v   odom dist[m]  imu dyaw[deg]  odom dyaw[deg]  tape[m]  odom/tape')
        for k, t0, t1, v, w in straight:
            # whole movement: from the command start until the car stopped (cmd_timeout 0.5 s + coasting)
            end = t1 + 2.0
            d = integrate(odom, 1, t0, end)
            dyi = math.degrees(integrate(imu, 1, t0, end)) if len(imu) else float('nan')
            dyo = math.degrees(integrate(odom, 2, t0, end))
            tp = tape.get(k)
            ratio = f'{d / tp:.3f}' if tp else '-'
            print(f'          {k:3d}  {t0:6.1f}-{t1:6.1f}  {v:+.2f}   {d:8.3f}      {dyi:+8.2f}       {dyo:+8.2f}'
                  f'      {tp if tp else "-":>6}   {ratio}')
        print()

    if curved:
        print('CURVED  run  t[s]          cmd v  cmd wz | cmd central/inner/value | gyro wz  odom wz  odom v |'
              ' true central/inner  odom inner | odom/gyro  scale->  R_rear[m]  turned[deg]')
        rows = []
        for k, t0, t1, v, w in curved:
            s0 = t0 + a.settle
            if t1 - s0 < 1.0:
                print(f'        {k:3d}  too short ({t1 - t0:.1f} s)')
                continue
            c_cmd = math.atan(L * w / v)
            i_cmd = float(central_to_inner(c_cmd))
            if math.isfinite(i_cmd):
                i_cmd = max(-a.max_inner, min(a.max_inner, i_cmd))
            value = i_cmd / a.scale
            g = mean(imu, 1, s0, t1)
            ow = mean(odom, 2, s0, t1)
            ov = mean(odom, 1, s0, t1)
            c_true = math.atan(L * g / ov)
            c_odom = math.atan(L * ow / ov)
            i_true = float(central_to_inner(c_true))
            i_odom = float(central_to_inner(c_odom))
            new_scale = a.scale * i_true / i_odom if abs(i_odom) > 1e-3 else float('nan')
            r_rear = ov / g if abs(g) > 1e-3 else float('inf')
            turned = math.degrees(integrate(imu, 1, t0, t1 + 2.0))
            rows.append((math.degrees(i_cmd), value, math.degrees(i_true), new_scale))
            print(f'        {k:3d}  {t0:6.1f}-{t1:6.1f}  {v:+.2f}  {w:+.3f} | {math.degrees(c_cmd):+5.1f}° {math.degrees(i_cmd):+5.1f}°'
                  f' {value:+.3f} | {g:+.3f}  {ow:+.3f}  {ov:+.3f} | {math.degrees(c_true):+5.1f}° {math.degrees(i_true):+5.1f}°'
                  f'  {math.degrees(i_odom):+5.1f}° | {ow / g:5.2f}   {new_scale:5.2f}   {r_rear:+6.3f}   {turned:+7.1f}')
        if rows:
            r = np.array(rows)
            ok = np.isfinite(r[:, 3])
            print(f'\n  steering_scale estimate (median over {ok.sum()} runs): {np.median(r[ok, 3]):.3f}'
                  f'  (limo_base ran with {a.scale})')
            print('  R_rear = odom speed / gyro rate = rear-axle-centre radius. Compare 2*R_rear with the tape diameter.')
            print('  odom speed is ~3% short in earlier bags; if the tape says so, R_rear is short by the same factor.')

    if len(imu) > 1:
        yaw = np.degrees(np.unwrap(imu[:, 2]))
        gyro = math.degrees(float(np.trapz(imu[:, 1], imu[:, 0])))
        print(f'\nwhole bag: /imu yaw change {yaw[-1] - yaw[0]:+.1f} deg, gyro integral {gyro:+.1f} deg'
              ' (hand-turn test: compare with 360)')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        ax[0].plot(cmd[:, 0], cmd[:, 1], '.', ms=2, label='cmd v')
        ax[0].plot(odom[:, 0], odom[:, 1], lw=0.8, label='odom v')
        ax[1].plot(cmd[:, 0], cmd[:, 2], '.', ms=2, label='cmd wz')
        ax[1].plot(odom[:, 0], odom[:, 2], lw=0.8, label='odom wz')
        if len(imu):
            ax[1].plot(imu[:, 0], imu[:, 1], lw=0.8, label='gyro z')
            ax[2].plot(imu[:, 0], np.degrees(np.unwrap(imu[:, 2])), lw=0.8, label='imu yaw')
        ax[2].plot(odom[:, 0], np.degrees(np.unwrap(odom[:, 3])), lw=0.8, label='odom yaw')
        for k, (t0, t1, v, w) in enumerate(rs, 1):
            for x in ax:
                x.axvspan(t0, t1, color='0.9')
            ax[0].text(t0, ax[0].get_ylim()[1] * 0.9, str(k), fontsize=8)
        for x in ax:
            x.grid(alpha=0.3)
            x.legend(fontsize=8, loc='upper right')
        ax[2].set_xlabel('bag time [s]')
        fig.tight_layout()
        fig.savefig(a.plot, dpi=100)
        print(f'\nplot: {a.plot}')


if __name__ == '__main__':
    main()
