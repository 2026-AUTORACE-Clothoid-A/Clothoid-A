"""Trigger timeline (recorded vs current-code replay, one row per trigger) and the driven-path minimap.

Both follow the viewer's time slider; clicking either one seeks there.
"""
import bisect
import math

from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget

from . import triggers

LEFT, RIGHT = 70, 15          # same x mapping as viewer.Timeline, so the two timelines line up
ROW = 15
SOURCES = (('recorded', '기록'), ('replay', '현재 코드'))


def _x(widget, t, duration):
    return LEFT + t / max(.01, duration) * (widget.width() - LEFT - RIGHT)


class TriggerTimeline(QWidget):
    selected = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.data = None; self.lanes = None; self.at = 0.
        self.setMinimumHeight(len(triggers.LANES) * ROW + 26)
        self.setMouseTracking(True)
        self.setToolTip('트리거별 한 줄: 위 절반 = 기록(실제 주행), 아래 절반 = 현재 코드 재계산.\n'
                        '연한 색 = 준비/탐색(표지판 확정, 선 찾는 중), 진한 색 = 실행(감속·확정·정지·주차 신호).\n'
                        '클릭하면 그 시간으로 이동합니다.\n'
                        '현재 코드 재계산은 미션·차선만 다시 돌립니다. 라바콘·장애물·RC는 기록에만 있고,\n'
                        '주차 신호는 라이다 주차칸 판정이 백에 기록된 경우에만 재계산됩니다.')

    def set_data(self, data):
        self.data = data; self.lanes = triggers.build(data); self.update()

    def mousePressEvent(self, event):
        if self.data:
            fraction = (event.x() - LEFT) / max(1, self.width() - LEFT - RIGHT)
            self.selected.emit(min(1., max(0., fraction)) * self.data['duration'])

    def mouseMoveEvent(self, event):
        if not self.data or event.x() < LEFT:
            return
        row = (event.y() - 20) // ROW
        if not 0 <= row < len(triggers.LANES):
            return
        key, label, _ = triggers.LANES[int(row)]
        t = (event.x() - LEFT) / max(1, self.width() - LEFT - RIGHT) * self.data['duration']
        parts = []
        for source, name in SOURCES:
            hit = [s for s in self.lanes[source].get(key, []) if s[0] - .2 <= t <= s[1] + .2]
            parts.append(f'{name}: ' + (', '.join(f'{a:.1f}~{b:.1f}s {"실행" if lv == 2 else "준비"}'
                                                  for a, b, lv in hit) or '없음'))
        self.setToolTip(f'{label} @ {t:.1f}s\n' + '\n'.join(parts))

    def paintEvent(self, event):
        p = QPainter(self); p.fillRect(self.rect(), QColor('#111e2b'))
        if not self.data:
            return
        duration = self.data['duration']
        p.setPen(QColor('#b8cddd')); p.drawText(8, 14, '트리거')
        p.setPen(QColor('#ffbd70')); p.drawText(self.width() - 300, 14, '▀ 기록')
        p.setPen(QColor('#65e5b3')); p.drawText(self.width() - 230, 14, '▄ 현재 코드')
        p.setPen(QColor('#9db3c7')); p.drawText(self.width() - 140, 14, '연함 준비 · 진함 실행')
        for row, (key, label, colour) in enumerate(triggers.LANES):
            top = 20 + row * ROW
            if row % 2:
                p.fillRect(QRectF(0, top, self.width(), ROW), QColor('#152536'))
            p.setPen(QColor('#b8cddd')); p.drawText(QRectF(2, top, LEFT - 4, ROW), Qt.AlignVCenter | Qt.AlignRight, label)
            for half, (source, _) in enumerate(SOURCES):
                y = top + 1 + half * (ROW - 2) / 2
                for t0, t1, level in self.lanes[source].get(key, []):
                    c = QColor(colour)
                    c.setAlpha(230 if level == 2 else 90)
                    x0, x1 = _x(self, t0, duration), _x(self, t1, duration)
                    p.fillRect(QRectF(x0, y, max(2., x1 - x0), (ROW - 2) / 2 - 1), c)
        p.setPen(QPen(QColor('#f48dba'), 2))
        x = _x(self, self.at, duration)
        p.drawLine(QPointF(x, 18), QPointF(x, self.height() - 2))


# point events drawn on the map: (lane, level 2 start) -> marker colour, name
MARKERS = (('stop', '#ff4d4d', '정지'), ('parking', '#4dd0c4', '주차'), ('line', '#ff6f91', '정지선 확정'),
           ('entry', '#ffa25c', '서행 시작선'), ('rc', '#b0b0b0', 'RC 조종'))


class MiniMap(QWidget):
    selected = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.data = None; self.at = 0.; self.times = []; self.points = []
        self.lanes = None; self.source = 'recorded'
        self.setMinimumSize(260, 260)
        self.setToolTip('IMU 회전 + 바퀴 속도로 그린 경로(보정 없음, 오래 달리면 조금씩 틀어집니다).\n'
                        '● 차 위치 / 표시 = 실행된 트리거 지점 (기록 기준). 경로를 클릭하면 그 시간으로 이동합니다.')

    def set_data(self, data, lanes):
        self.data = data; self.lanes = lanes
        frames = [f for f in data['frames'] if f.get('pose') is not None]
        self.times = [f['t'] for f in frames]
        self.points = [f['pose'] for f in frames]
        self.update()

    def _transform(self):
        xs = [p[0] for p in self.points]; ys = [p[1] for p in self.points]
        span = max(max(xs) - min(xs), max(ys) - min(ys), .5)
        scale = (min(self.width(), self.height()) - 40) / span
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        # map x (forward at start) points up, map y (left) points left
        return lambda x, y: QPointF(self.width() / 2 - (y - cy) * scale, self.height() / 2 - (x - cx) * scale)

    def pose_at(self, t):
        i = max(0, min(len(self.times) - 1, bisect.bisect_right(self.times, t) - 1))
        return self.points[i]

    def mousePressEvent(self, event):
        if not self.points:
            return
        to = self._transform()
        best = min(range(len(self.points)), key=lambda i: (to(*self.points[i][:2]).x() - event.x()) ** 2 +
                   (to(*self.points[i][:2]).y() - event.y()) ** 2)
        self.selected.emit(self.times[best])

    def paintEvent(self, event):
        p = QPainter(self); p.fillRect(self.rect(), QColor('#111e2b'))
        if not self.points:
            p.setPen(QColor('#9db3c7')); p.drawText(self.rect(), Qt.AlignCenter, 'IMU / odom 미기록')
            return
        p.setRenderHint(QPainter.Antialiasing)
        to = self._transform()
        path = QPainterPath(to(*self.points[0][:2]))
        for x, y, _ in self.points[1:]:
            path.lineTo(to(x, y))
        p.setPen(QPen(QColor('#3d5873'), 3)); p.drawPath(path)
        # driven so far
        done = QPainterPath(to(*self.points[0][:2]))
        for t, (x, y, _) in zip(self.times, self.points):
            if t > self.at:
                break
            done.lineTo(to(x, y))
        p.setPen(QPen(QColor('#8bbfff'), 2)); p.drawPath(done)
        # trigger markers (start of each acting segment)
        legend = []
        for lane, colour, name in MARKERS:
            segs = [s for s in (self.lanes or {}).get(self.source, {}).get(lane, []) if s[2] == 2]
            if not segs:
                continue
            legend.append((colour, name))
            p.setPen(QPen(QColor('#111e2b'), 1)); p.setBrush(QColor(colour))
            for t0, _, _ in segs:
                point = to(*self.pose_at(t0)[:2])
                p.drawEllipse(point, 5, 5)
        # start and the car
        p.setBrush(QColor('#ffffff')); p.drawEllipse(to(*self.points[0][:2]), 4, 4)
        x, y, yaw = self.pose_at(self.at)
        c = to(x, y)
        # screen direction of the heading (map x up, map y left)
        dx, dy = -math.sin(yaw), -math.cos(yaw)
        tip = QPointF(c.x() + dx * 12, c.y() + dy * 12)
        side = QPointF(-dy * 6, dx * 6)
        p.setBrush(QColor('#f48dba')); p.setPen(QPen(QColor('#ffffff'), 1))
        p.drawPolygon(QPolygonF([tip, c + side, c - side]))
        p.setPen(QColor('#dce8f3')); p.drawText(8, 16, f'{self.at:.1f}s · 시작 ○')
        if legend:
            box = QColor('#111e2b'); box.setAlpha(200)
            p.fillRect(QRectF(4, self.height() - 8 - 15 * len(legend), 110, 15 * len(legend) + 4), box)
        for i, (colour, name) in enumerate(legend):
            p.setPen(QColor(colour)); p.drawText(8, self.height() - 10 - 15 * i, f'● {name}')


class MapTab(QWidget):
    """Minimap + the triggers active at the current time, for the side tabs."""
    seek = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.map = MiniMap(); self.map.selected.connect(self.seek); layout.addWidget(self.map, 1)
        self.now = QLabel('분석 대기'); self.now.setWordWrap(True); self.now.setTextFormat(Qt.RichText)
        self.now.setStyleSheet('background:#203348;border-radius:5px;padding:8px')
        layout.addWidget(self.now)
        self.lanes = None

    def set_data(self, data, lanes):
        self.lanes = lanes; self.map.set_data(data, lanes)

    def update_time(self, t):
        self.map.at = t; self.map.update()
        if not self.lanes:
            return
        rows = []
        for source, name in SOURCES:
            active = triggers.active_at(self.lanes[source], t)
            items = [f'<span style="color:{c}">{label}{"" if active[k] == 2 else " (준비)"}</span>'
                     for k, label, c in triggers.LANES if k in active]
            rows.append(f'<b>{name}</b>: ' + (' · '.join(items) or '없음'))
        self.now.setText(f'<b>{t:.2f}s 지금 켜진 트리거</b><br>' + '<br>'.join(rows))
