#!/usr/bin/env python3
"""Native bump/stop-line diagnostics window. Run in the ROS Docker container."""
import argparse
import bisect
from copy import deepcopy
import html
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np
import yaml
from PyQt5.QtCore import Qt, QPointF, QRectF, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QKeySequence, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QHBoxLayout,
    QGroupBox, QLabel, QListWidget, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QShortcut, QSlider, QSpinBox, QSplitter, QTabWidget, QTextBrowser, QVBoxLayout, QWidget)

from tools.bump_viewer import backend, render
from tools.bump_viewer.log_panel import LogPanel
from tools.bump_viewer.color_settings import PRESETS, load_settings, preset_settings, settings_yaml, validate_detector

BUMP_NAMES = {'idle': '대기', 'approaching': '접근·서행', 'on_bump': '턱 통과 중', 'cooldown': '통과 후 대기'}
COLOR = {'idle': '#91a9bd', 'approaching': '#ffca75', 'on_bump': '#65e5b3', 'cooldown': '#a89aff'}


class Loader(QThread):
    loaded = pyqtSignal(object)
    failed = pyqtSignal(str)
    progress = pyqtSignal(int)

    def __init__(self, path, camera, hsv_overrides=None):
        super().__init__(); self.path, self.camera = path, camera
        self.hsv_overrides = deepcopy(hsv_overrides)

    def run(self):
        try:
            def progress(value):
                if self.isInterruptionRequested():
                    raise InterruptedError('분석을 취소했습니다.')
                self.progress.emit(value)
            self.loaded.emit(backend.analyse(self.path, self.camera, progress, self.hsv_overrides))
        except Exception as error:
            import traceback
            traceback.print_exc()
            self.failed.emit(str(error))


class Picture(QWidget):
    hovered = pyqtSignal(str)

    def __init__(self, title):
        super().__init__(); self.title = title; self.image = None; self.hsv = None; self.pixels = None
        self.area = QRectF(); self.setMinimumSize(210, 165); self.setMouseTracking(True)

    def update_image(self, bgr, hsv=None, pixels=None):
        self.hsv = hsv
        self.pixels = pixels
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
        self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.fillRect(self.rect(), QColor('#0e1925'))
        p.setPen(QColor('#dce9f5')); p.drawText(10, 22, self.title)
        if self.image is None:
            return
        size = self.image.size(); size.scale(self.width()-10, self.height()-35, Qt.KeepAspectRatio)
        self.area = QRectF((self.width()-size.width())/2, 30+(self.height()-30-size.height())/2,
                           size.width(), size.height())
        p.drawImage(self.area, self.image)

    def mouseMoveEvent(self, event):
        if self.hsv is None or not self.area.contains(event.pos()):
            return
        x = min(self.hsv.shape[1]-1, int((event.x()-self.area.x())/self.area.width()*self.hsv.shape[1]))
        y = min(self.hsv.shape[0]-1, int((event.y()-self.area.y())/self.area.height()*self.hsv.shape[0]))
        h, s, v = map(int, self.hsv[y, x])
        detail = f'H {h} / S {s} / V {v}'
        if self.pixels is not None:
            pixel = self.pixels[y, x]
            b, g, r = map(int, pixel)
            l, a, bb = cv2.cvtColor(pixel.reshape(1, 1, 3).astype(np.float32)/255., cv2.COLOR_BGR2LAB)[0, 0]
            detail = f'RGB {r}/{g}/{b} · {detail} · Lab {l:.1f}/{a:.1f}/{bb:.1f} · 색차 {min(r,g)-b} / 폭 {max(r,g,b)-min(r,g,b)}'
        self.hovered.emit(f'{self.title} · ({x}, {y}) → {detail}')


class Timeline(QWidget):
    selected = pyqtSignal(float)

    def __init__(self):
        super().__init__(); self.data = None; self.at = 0.; self.setMinimumHeight(125)

    def mousePressEvent(self, event):
        if self.data:
            self.selected.emit(np.clip((event.x()-70)/max(1, self.width()-85), 0, 1)*self.data['duration'])

    def paintEvent(self, event):
        p = QPainter(self); p.fillRect(self.rect(), QColor('#111e2b'))
        if not self.data:
            return
        p.setRenderHint(QPainter.Antialiasing)
        data = self.data; duration = max(.01, data['duration'])
        x = lambda t: 70+t/duration*(self.width()-85)
        p.setPen(QColor('#b8cddd')); p.drawText(8, 23, 'pitch °'); p.drawText(8, 87, '속도 m/s')
        pitch = [abs(f['raw_pitch']) for f in data['frames'] if f['raw_pitch'] is not None]
        bound = max(6., min(45., max(pitch, default=6.)*1.1))
        py = lambda value: 40-value/bound*24
        vy = lambda value: 111-value*30
        p.setPen(QPen(QColor('#314454'), 1))
        p.drawLine(QPointF(70, py(0)), QPointF(self.width()-15, py(0)))
        p.drawLine(QPointF(70, vy(0)), QPointF(self.width()-15, vy(0)))
        p.setPen(QPen(QColor('#b48d5e'), 1, Qt.DashLine))
        for value in (-data['params']['bump_pitch_thresh'], data['params']['bump_pitch_thresh']):
            p.drawLine(QPointF(70, py(value)), QPointF(self.width()-15, py(value)))
        def curve(key, transform, color):
            path = QPainterPath(); last = None
            for frame in data['frames']:
                value = key(frame)
                if value is None:
                    last = None; continue
                point = QPointF(x(frame['t']), transform(value))
                if last is None or frame['t']-last > .3:
                    path.moveTo(point)
                else:
                    path.lineTo(point)
                last = frame['t']
            p.setPen(QPen(QColor(color), 1.7)); p.drawPath(path)
        curve(lambda f: f['raw_pitch'], py, '#8bbfff')
        curve(lambda f: None if f['command'] is None else f['command'][0], vy, '#65e5b3')
        curve(lambda f: None if f['recorded_command'] is None else f['recorded_command'][0], vy, '#ffbd70')
        p.setPen(QPen(QColor('#f48dba'), 2)); p.drawLine(QPointF(x(self.at), 3), QPointF(x(self.at), self.height()-3))
        p.setPen(QColor('#8bbfff')); p.drawText(self.width()-450, 16, '━━ IMU pitch')
        p.setPen(QColor('#65e5b3')); p.drawText(self.width()-330, 16, '━━ 현재 코드 속도')
        p.setPen(QColor('#ffbd70')); p.drawText(self.width()-170, 16, '━━ 기록된 /cmd_vel')


class Viewer(QMainWindow):
    def __init__(self, bags, camera='limo2', at=None, self_test=None):
        super().__init__(); self.setWindowTitle('Bump Viewer · 방지턱 / 정지선 · 색상 / 보정')
        self.resize(1560, 980)
        self.bags = list(bags); self.data = None; self.index = 0; self.worker = None
        self.loading = False
        self.hsv_settings = None
        self.hsv_preview_timer = QTimer(self); self.hsv_preview_timer.setSingleShot(True)
        self.hsv_preview_timer.timeout.connect(self.refresh_hsv_preview)
        self.playing = False; self.wall = time.monotonic(); self.clock = 0.
        self.requested_time = at; self.self_test = self_test; self.error = None
        root = QWidget(); layout = QVBoxLayout(root); layout.setContentsMargins(14, 10, 14, 10)
        self.setCentralWidget(root)
        top = QHBoxLayout(); title = QLabel('BUMP / STOP VIEWER')
        title.setStyleSheet('font-size:20px;font-weight:600;color:#edf7ff'); top.addWidget(title)
        top.addStretch(); self.combo = QComboBox(); self.combo.setMinimumWidth(260)
        for path in bags:
            self.combo.addItem(path.name)
        self.combo.currentIndexChanged.connect(self.load_selected); top.addWidget(self.combo)
        self.profile = QComboBox(); self.profile.addItems(['limo2', 'limo1']); self.profile.setCurrentText(camera)
        self.profile.currentIndexChanged.connect(self.load_selected); top.addWidget(self.profile)
        self.open_button = QPushButton('백 열기'); self.open_button.clicked.connect(self.open_bag); top.addWidget(self.open_button)
        self.reload_button = QPushButton('현재 코드로 재계산'); self.reload_button.clicked.connect(self.load_selected)
        top.addWidget(self.reload_button)
        self.stopline_settings_button = QPushButton('정지선 마스크 조정')
        self.stopline_settings_button.clicked.connect(self.open_stopline_settings)
        top.addWidget(self.stopline_settings_button); layout.addLayout(top)
        self.source = QLabel('현재 소스와 YAML을 읽는 중'); self.source.setWordWrap(True); layout.addWidget(self.source)
        cards = QHBoxLayout(); self.cards = []
        for _ in range(4):
            label = QLabel('분석 대기'); label.setMinimumHeight(52); label.setWordWrap(True)
            label.setStyleSheet('background:#203348;border-radius:5px;padding:8px;font-size:14px')
            cards.addWidget(label, 1); self.cards.append(label)
        layout.addLayout(cards)
        splitter = QSplitter(Qt.Horizontal)
        gallery = QWidget(); grid = QGridLayout(gallery); grid.setContentsMargins(0, 0, 0, 0)
        self.pictures = {}
        titles = [('camera', '원본 카메라 · 표지판 / BEV 영역'), ('processed', '보정 카메라 · 차선 설정'),
                  ('lane', '차선 BEV · 마지막 재계산 결과'), ('line', '정지선 BEV · 중앙 검사 영역 / 거리'),
                  ('yellow', '차선 노란색 마스크'), ('white', '정지선 흰색 마스크 · 상시 미리보기'),
                  ('comparison', '차선 색 마스크 · YAML 기준 / 조정값'), ('hsv', '보정 BEV의 H / S / V 채널')]
        for i, (key, name) in enumerate(titles):
            picture = Picture(name); picture.hovered.connect(self.hover)
            self.pictures[key] = picture; grid.addWidget(picture, i//4, i%4)
        splitter.addWidget(gallery)
        side = self.side_tabs = QTabWidget(); self.events = QListWidget(); self.events.itemClicked.connect(
            lambda item: self.seek_time(item.data(Qt.UserRole)))
        side.addTab(self.events, '이벤트 · 클릭 이동')
        self.hsv_tab = self.make_hsv_tab(); side.addTab(self.hsv_tab, '색상 / 보정')
        self.log_tab = LogPanel(); side.addTab(self.log_tab, '미션 / 로그')
        self.log_tab.seek.connect(self.seek_time)
        side.setCurrentWidget(self.hsv_tab)
        self.details = QTextBrowser(); side.addTab(self.details, '설정 / 검출 상세')
        side.setMinimumWidth(330); splitter.addWidget(side); splitter.setSizes([1150, 380])
        layout.addWidget(splitter, 1)
        self.pixel = QLabel('영상 위에 마우스를 올리면 RGB · HSV · Lab · 색차를 볼 수 있어요.')
        self.pixel.setStyleSheet('color:#a8bdcf'); layout.addWidget(self.pixel)
        self.timeline = Timeline(); self.timeline.selected.connect(self.seek_time); layout.addWidget(self.timeline)
        controls = QHBoxLayout(); self.time_label = QLabel('0.00 s'); self.time_label.setMinimumWidth(150)
        controls.addWidget(self.time_label); self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self.seek_index); controls.addWidget(self.slider, 1); layout.addLayout(controls)
        row = QHBoxLayout(); self.play_button = QPushButton('재생 [Space]'); self.play_button.clicked.connect(self.toggle)
        row.addWidget(self.play_button)
        for label, action in [('◀ 한 프레임', lambda: self.step(-1)), ('한 프레임 ▶', lambda: self.step(1)),
                              ('방지턱 표지판', self.seek_bump), ('최대 기울기', self.seek_pitch)]:
            button = QPushButton(label); button.clicked.connect(action); row.addWidget(button)
        self.speed = QComboBox(); self.speed.addItems(['0.25×', '0.5×', '1×', '2×']); self.speed.setCurrentIndex(2)
        row.addWidget(self.speed); row.addStretch()
        row.addWidget(QLabel('흰색 마스크')); row.addWidget(self.white_mode)
        self.color_source = QComboBox(); self.color_source.addItems(['차선 보정 보기', '미션 보정 보기'])
        self.color_source.currentIndexChanged.connect(self.refresh_hsv_preview); row.addWidget(self.color_source)
        row.addWidget(QLabel('HSV 보기'))
        self.hsv_mode = QComboBox(); self.hsv_mode.addItems(['H 색상', 'S 채도', 'V 명도', 'H / S / V 전체'])
        self.hsv_mode.setCurrentIndex(2)
        self.hsv_mode.currentIndexChanged.connect(lambda: self.show_frame() if self.data else None)
        row.addWidget(self.hsv_mode)
        save = QPushButton('현재 화면 저장'); save.clicked.connect(self.save); row.addWidget(save); layout.addLayout(row)
        footer = QLabel('기록된 YOLO + IMU·odom을 현재 미션/차선 코드로 재생합니다. YOLO 재추론·동역학 재현은 하지 않습니다. '
                        '흰색 마스크는 비활성 구간도 확인하는 미리보기이며, 실제 정지 실행 여부는 미션 상태에서 확인하세요. 자동 반복 없음.')
        footer.setWordWrap(True); footer.setStyleSheet('color:#9db3c7;font-size:11px'); layout.addWidget(footer)
        self.pictures['line'].setToolTip('평면으로 투영한 BEV입니다. 방지턱에서 카메라가 기울면 표시된 거리도 왜곡될 수 있습니다.')
        self.setStyleSheet('QMainWindow,QWidget{background:#172433;color:#dce8f3;font-size:12px}'
            'QPushButton,QComboBox{background:#2a4055;border:1px solid #496177;border-radius:4px;padding:7px}'
            'QPushButton:disabled{color:#758493} QListWidget,QTextBrowser{background:#101d2b;border:0}'
            'QListWidget::item{padding:7px} QListWidget::item:selected{background:#2a5264}'
            'QTabBar::tab{padding:7px;background:#23374a} QSlider::handle:horizontal{background:#63dfb3;width:12px;border-radius:5px}')
        self.play_shortcut = QShortcut(QKeySequence('Space'), self, activated=self.toggle)
        self.frame_shortcuts = [QShortcut(QKeySequence(Qt.Key_Left), self, activated=lambda: self.step(-1)),
                                QShortcut(QKeySequence(Qt.Key_Right), self, activated=lambda: self.step(1))]
        QApplication.instance().focusChanged.connect(self.hsv_focus_changed)
        self.timer = QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(20)
        QTimer.singleShot(0, self.load_selected)

    def hover(self, value):
        self.pixel.setText(value)

    def open_stopline_settings(self):
        self.side_tabs.setCurrentWidget(self.hsv_tab)
        self.color_sections.setCurrentWidget(self.stopline_color_page)
        self.white_mode.setCurrentIndex(0)
        self.color_source.setCurrentIndex(1)

    def hsv_focus_changed(self, old, current):
        editing = current is not None and self.hsv_tab.isAncestorOf(current)
        self.play_shortcut.setEnabled(not editing)
        for shortcut in self.frame_shortcuts:
            shortcut.setEnabled(not editing)

    def make_hsv_tab(self):
        tab = QWidget(); layout = QVBoxLayout(tab)
        layout.setContentsMargins(6, 6, 6, 6); layout.setSpacing(5)
        help_text = QLabel('드래그하면 마스크가 즉시 바뀝니다.\n'
                           '상단 상태·차선 추종점·속도는 재계산 후 반영됩니다.')
        help_text.setWordWrap(True); layout.addWidget(help_text)
        self.hsv_status = QLabel('백 분석 후 조정할 수 있습니다.')
        self.hsv_status.setWordWrap(True); layout.addWidget(self.hsv_status)
        self.hsv_controls = {}; self.color_controls = {}; self.method_boxes = {}
        self.color_sections = QTabWidget()
        preset_row = QHBoxLayout(); self.preset_combo = QComboBox()
        self.preset_combo.addItems([name for name, _ in PRESETS]); preset_row.addWidget(self.preset_combo, 1)
        self.preset_button = QPushButton('차선에 적용'); self.preset_button.clicked.connect(self.apply_color_preset)
        preset_row.addWidget(self.preset_button); layout.addLayout(preset_row)
        # Each source has its own preprocessing and classifier. Mission is white-only.
        containers = {}
        for group, title in [('lane', '차선'), ('mission', '정지선 / 시작선')]:
            page = QWidget(); page_layout = QVBoxLayout(page)
            if group == 'mission':
                self.stopline_color_page = page
            inner_scroll = QScrollArea(); inner_scroll.setWidgetResizable(True)
            form = QWidget(); form_layout = QVBoxLayout(form); containers[group] = form_layout
            method = QComboBox(); method.addItems(['hsv', 'lab', 'rgb'])
            method.currentTextChanged.connect(lambda value, group=group: self.change_hsv((group, 'color_method', None), value))
            self.color_controls[group, 'color_method'] = method
            row = QHBoxLayout(); row.addWidget(QLabel('색 판정')); row.addWidget(method, 1); form_layout.addLayout(row)
            self.add_color_number(form_layout, group, 'blue_glare_strength', '청색 빛번짐 억제', 0., 1., .05)
            for name, label in [('gamma_enabled', 'RGB 감마'), ('clahe_enabled', 'CLAHE · Lab 명암')]:
                check = QCheckBox(label)
                check.toggled.connect(lambda value, group=group, name=name: self.change_hsv((group, name, None), value))
                self.color_controls[group, name] = check; form_layout.addWidget(check)
                if name == 'gamma_enabled':
                    self.add_color_number(form_layout, group, 'gamma', '감마 지수 · >1 어둡게', .1, 4., .05)
                else:
                    self.add_color_number(form_layout, group, 'clahe_clip_limit', '대비 제한', .1, 16., .1)
                    self.add_color_number(form_layout, group, 'clahe_grid_size', '격자 수 · 가로/세로', 2, 32)
            hint = QLabel('빛번짐 억제 → 감마 → CLAHE → 기존 블러·BEV → 색 판정\n'
                          + ('노란색 우선, 나머지 픽셀에서 흰색 판정' if group == 'lane' else '차선과 독립된 흰색 판정 · 노란색 차감 없음'))
            hint.setWordWrap(True); form_layout.addWidget(hint)
            inner_scroll.setWidget(form); page_layout.addWidget(inner_scroll)
            self.color_sections.addTab(page, title)
        specs = [('노란 차선 · HSV 범위', 'lane', [
                    (f'{channel} {bound}', f'hsv_{name}', index, maximum)
                    for index, (channel, maximum) in enumerate((('H', 179), ('S', 255), ('V', 255)))
                    for bound, name in (('최소', 'lower'), ('최대', 'upper'))]),
                 ('흰색 차선 · track2026', 'lane', [
                    ('H 최소', 'white_h_min', None, 179), ('H 최대', 'white_h_max', None, 179),
                    ('S 상한', 'white_s_max', None, 255), ('V 하한', 'white_v_min', None, 255)]),
                 ('흰색 정지선 / 시작선', 'mission', [
                    ('H 최소', 'white_h_min', None, 179), ('H 최대', 'white_h_max', None, 179),
                    ('S 상한', 'white_s_max', None, 255), ('V 하한', 'white_v_min', None, 255)])]
        insert_at = {'lane': 1, 'mission': 1}
        for title, group, rows in specs:
            box = QGroupBox(title); box_layout = QVBoxLayout(box)
            for label, name, index, maximum in rows:
                row = QHBoxLayout(); row.addWidget(QLabel(label))
                slider = QSlider(Qt.Horizontal); slider.setRange(0, maximum)
                spin = QSpinBox(); spin.setRange(0, maximum); spin.setFixedWidth(65)
                spin.setKeyboardTracking(False)
                key = (group, name, index)
                slider.setObjectName('_'.join(str(part) for part in key))
                self.hsv_controls[key] = slider, spin
                slider.valueChanged.connect(lambda value, key=key: self.change_hsv(key, value))
                spin.valueChanged.connect(lambda value, key=key: self.change_hsv(key, value))
                row.addWidget(slider, 1); row.addWidget(spin); box_layout.addLayout(row)
            self.method_boxes.setdefault((group, 'hsv'), []).append(box)
            containers[group].insertWidget(insert_at[group], box); insert_at[group] += 1
        lab_rows = [('lab_yellow_b_min', '노란색 b 하한', 0., 127.),
                    ('lab_yellow_a_abs_max', '노란색 |a| 상한', 0., 128.),
                    ('lab_yellow_l_min', '노란색 L 하한', 0., 100.),
                    ('lab_white_chroma_max', '흰색 √(a²+b²) 상한', 0., 200.),
                    ('lab_white_l_min', '흰색 L 하한', 0., 100.)]
        rgb_rows = [('rgb_yellow_difference_min', '노란색 min(R,G)−B 하한', 0, 255),
                    ('rgb_yellow_v_min', '노란색 밝기 하한', 0, 255),
                    ('rgb_white_spread_max', '흰색 max−min 상한', 0, 255),
                    ('rgb_white_v_min', '흰색 밝기 하한', 0, 255)]
        for group in containers:
            for mode, title, rows in [('lab', 'CIELab · L 0~100 / a,b 부호 있음', lab_rows),
                                      ('rgb', 'RGB · 보정 후 채널 0~255', rgb_rows)]:
                box = QGroupBox(title); box_layout = QVBoxLayout(box)
                for name, label, low, high in rows:
                    if group == 'mission' and 'yellow' in name:
                        continue
                    self.add_color_number(box_layout, group, name, label, low, high, .5 if mode == 'lab' else None)
                containers[group].insertWidget(insert_at[group], box); insert_at[group] += 1
                self.method_boxes[group, mode] = [box]
            containers[group].addStretch()
        layout.addWidget(self.color_sections, 1)
        self.white_mode = QComboBox(); self.white_mode.addItems(['정지선', '차선'])
        self.white_mode.currentIndexChanged.connect(self.refresh_hsv_preview)
        row = QHBoxLayout()
        self.hsv_apply_button = QPushButton('조정값 전체 재계산')
        self.hsv_apply_button.clicked.connect(self.apply_hsv); row.addWidget(self.hsv_apply_button)
        self.hsv_reset_button = QPushButton('YAML 값으로 복원')
        self.hsv_reset_button.clicked.connect(self.reset_hsv); row.addWidget(self.hsv_reset_button)
        layout.addLayout(row)
        row = QHBoxLayout()
        self.color_load_button = QPushButton('설정 불러오기'); self.color_load_button.clicked.connect(self.load_color_file)
        self.color_save_button = QPushButton('설정 저장'); self.color_save_button.clicked.connect(self.save_color_file)
        row.addWidget(self.color_load_button); row.addWidget(self.color_save_button)
        self.hsv_copy_button = QPushButton('YAML 복사')
        self.hsv_copy_button.clicked.connect(self.copy_hsv); row.addWidget(self.hsv_copy_button)
        layout.addLayout(row)
        note = QLabel('뷰어에서 조정·재계산하며, 파일 저장은 별도입니다.')
        note.setWordWrap(True); note.setStyleSheet('color:#a8bdcf;font-size:11px'); layout.addWidget(note)
        return tab

    def add_color_number(self, layout, group, name, label, low, high, step=None):
        row = QHBoxLayout(); row.addWidget(QLabel(label), 1)
        spin = QDoubleSpinBox() if step is not None else QSpinBox()
        spin.setRange(low, high); spin.setKeyboardTracking(False); spin.setFixedWidth(85)
        if step is not None:
            spin.setDecimals(2); spin.setSingleStep(step)
        spin.valueChanged.connect(lambda value: self.change_hsv((group, name, None), value))
        self.color_controls[group, name] = spin; row.addWidget(spin); layout.addLayout(row)

    def apply_color_preset(self):
        if self.data and not self.loading:
            self.hsv_settings = preset_settings(self.preset_combo.currentIndex(), self.hsv_settings, self.data['hsv_baseline'])
            self.settings_changed()

    def load_color_file(self):
        if not self.data or self.loading:
            return
        path, _ = QFileDialog.getOpenFileName(self, '색상 설정 불러오기', str(ROOT/'ros2_ws/src/clothoid_lane/config'), 'YAML (*.yaml *.yml)')
        if path:
            try:
                values = load_settings(path, self.hsv_settings)
                validate_detector(values, self.data['detector_values']['detector'])
                self.hsv_settings = values
                self.settings_changed()
            except (OSError, ValueError, yaml.YAMLError) as error:
                QMessageBox.warning(self, '설정을 불러오지 못했습니다', str(error))

    def save_color_file(self):
        if self.hsv_settings is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, '색상 설정 저장', str(ROOT/'color_tuning.yaml'), 'YAML (*.yaml *.yml)')
        if path:
            try:
                Path(path).write_text(settings_yaml(self.hsv_settings))
                self.statusBar().showMessage('색상 설정을 저장했습니다: '+path, 4000)
            except OSError as error:
                QMessageBox.warning(self, '설정을 저장하지 못했습니다', str(error))

    def sync_hsv_controls(self):
        for (group, name, index), controls in self.hsv_controls.items():
            value = self.hsv_settings[group][name]
            if index is not None:
                value = value[index]
            for control in controls:
                control.blockSignals(True); control.setValue(value); control.blockSignals(False)
        for (group, name), control in self.color_controls.items():
            control.setEnabled(group == 'mission' or self.data['detector_values']['detector'] == 'track2026')
            value = self.hsv_settings[group][name]; control.blockSignals(True)
            if isinstance(control, QCheckBox):
                control.setChecked(value)
            elif isinstance(control, QComboBox):
                control.setCurrentText(value)
            else:
                control.setValue(value)
            control.blockSignals(False)
        self.preset_button.setEnabled(self.data['detector_values']['detector'] == 'track2026')
        for (group, method), boxes in self.method_boxes.items():
            for box in boxes:
                box.setVisible(self.hsv_settings[group]['color_method'] == method)

    def update_hsv_status(self):
        if not self.data or self.hsv_settings is None:
            return
        dirty = self.hsv_settings != self.data['hsv_applied']
        self.hsv_status.setText('마스크 미리보기 변경됨 · 전체 결과는 재계산 필요' if dirty
                                else '마지막 재계산과 같은 색상 / 보정 설정입니다.')
        self.hsv_status.setStyleSheet('color:#ffca75' if dirty else 'color:#81dfc0')
        self.hsv_apply_button.setEnabled(dirty)
        self.hsv_reset_button.setEnabled(self.hsv_settings != self.data['hsv_baseline'])

    def change_hsv(self, key, value):
        if self.hsv_settings is None or self.loading:
            return
        group, name, index = key
        if index is None:
            self.hsv_settings[group][name] = value
            if name in ('white_h_min', 'white_h_max'):
                other = 'white_h_max' if name == 'white_h_min' else 'white_h_min'
                compare = max if name == 'white_h_min' else min
                self.hsv_settings[group][other] = compare(value, self.hsv_settings[group][other])
        else:
            self.hsv_settings[group][name][index] = value
            # Keep each min/max pair ordered while allowing either endpoint to be dragged.
            other = 'hsv_upper' if name == 'hsv_lower' else 'hsv_lower'
            compare = max if name == 'hsv_lower' else min
            self.hsv_settings[group][other][index] = compare(value, self.hsv_settings[group][other][index])
        self.settings_changed()

    def settings_changed(self):
        self.playing = False; self.play_button.setText('재생 [Space]')
        self.sync_hsv_controls(); self.update_hsv_status()
        if not self.hsv_preview_timer.isActive():
            self.hsv_preview_timer.start(40)

    def refresh_hsv_preview(self):
        if self.data and not self.loading:
            self.show_frame()

    def apply_hsv(self):
        if self.data and not self.loading:
            self.load_selected(hsv_overrides=self.hsv_settings)

    def reset_hsv(self):
        if self.data and not self.loading:
            self.playing = False; self.play_button.setText('재생 [Space]')
            self.hsv_settings = deepcopy(self.data['hsv_baseline'])
            self.sync_hsv_controls(); self.update_hsv_status(); self.refresh_hsv_preview()

    def copy_hsv(self):
        if self.hsv_settings is not None:
            QApplication.clipboard().setText(settings_yaml(self.hsv_settings))
            self.statusBar().showMessage('현재 색상 / 보정 설정을 YAML로 복사했습니다.', 4000)

    def busy(self, state):
        self.loading = state
        for widget in (self.combo, self.profile, self.open_button, self.reload_button, self.slider):
            widget.setEnabled(not state)
        self.timeline.setEnabled(not state)
        self.hsv_tab.setEnabled(not state)
        self.log_tab.setEnabled(not state)

    def load_selected(self, _=None, *, hsv_overrides=None):
        if self.worker is not None and self.worker.isRunning():
            return
        self.playing = False; self.play_button.setText('재생 [Space]'); self.busy(True)
        if self.data is not None and Path(self.data['path']).resolve() == self.bags[self.combo.currentIndex()].resolve():
            self.requested_time = self.data['frames'][self.index]['t']
        elif self.data is not None:
            self.requested_time = None
        self.source.setText('현재 코드 / YAML로 분석 중… 0%')
        self.worker = Loader(self.bags[self.combo.currentIndex()], self.profile.currentText(), hsv_overrides)
        self.worker.progress.connect(lambda n: self.source.setText(f'현재 코드 / YAML로 분석 중… {n}%'))
        self.worker.loaded.connect(self.loaded); self.worker.failed.connect(self.failed); self.worker.start()

    def loaded(self, data):
        self.data = data; self.times = [f['t'] for f in data['frames']]; self.index = 0
        self.log_tab.set_data(data)
        self.hsv_settings = deepcopy(data['hsv_applied'])
        self.sync_hsv_controls(); self.update_hsv_status()
        self.slider.setRange(0, len(self.times)-1); self.busy(False)
        self.events.clear()
        for stamp, label in data['events']:
            self.events.addItem(f'{stamp:6.2f}s  {label}')
            self.events.item(self.events.count()-1).setData(Qt.UserRole, stamp)
            self.events.item(self.events.count()-1).setForeground(QColor('#ffca75' if label.startswith('기록') else '#81dfc0'))
        warnings = ' / '.join(data['warnings'])
        self.source.setText(f'{Path(data["path"]).name} | {data["camera"]} | 현재 소스 {data["digest"]} | '
                            f'{len(self.times):,} 프레임'+(' | ⚠ '+warnings if warnings else ''))
        self.timeline.data = data
        if self.requested_time is None:
            self.seek_bump()
        else:
            self.seek_time(self.requested_time)
        self.requested_time = None
        print(f'READY: Bump Viewer | {Path(data["path"]).name} | {len(self.times)} frames | '
              f'source {data["digest"]}', flush=True)
        if self.self_test:
            QTimer.singleShot(100, self.test_ui)

    def failed(self, message):
        self.error = message; self.busy(False); self.source.setText('분석 실패: '+message)
        if self.self_test:
            QApplication.instance().exit(1)
        else:
            QMessageBox.critical(self, '백 분석 실패', message)

    def seek_time(self, stamp):
        if self.data and not self.loading:
            self.seek_index(max(0, min(len(self.times)-1, bisect.bisect_right(self.times, stamp)-1)))

    def seek_index(self, index):
        if not self.data or self.loading:
            return
        self.index = max(0, min(len(self.times)-1, index)); self.clock = self.times[self.index]
        self.wall = time.monotonic(); self.slider.blockSignals(True); self.slider.setValue(self.index)
        self.slider.blockSignals(False); self.show_frame()

    def step(self, count):
        if self.loading:
            return
        self.playing = False; self.play_button.setText('재생 [Space]'); self.seek_index(self.index+count)

    def seek_bump(self):
        if not self.data or self.loading:
            return
        index = next((i for i, f in enumerate(self.data['frames']) if
                      any(d['name'] == 'sign_bump' for d in f['detections'])), 0)
        self.seek_index(index)

    def toggle(self):
        if not self.data or (self.worker and self.worker.isRunning()):
            return
        if self.index == len(self.times)-1:
            self.seek_index(0)
        self.playing = not self.playing; self.wall = time.monotonic()
        self.play_button.setText('일시정지 [Space]' if self.playing else '재생 [Space]')

    def seek_pitch(self):
        if self.data and not self.loading:
            self.seek_index(max(range(len(self.times)), key=lambda i: abs(self.data['frames'][i]['raw_pitch'] or 0.)))

    def tick(self):
        if not self.playing or not self.data:
            return
        now = time.monotonic(); self.clock += (now-self.wall)*[.25, .5, 1., 2.][self.speed.currentIndex()]
        self.wall = now
        index = min(len(self.times)-1, max(0, bisect.bisect_right(self.times, self.clock)-1))
        if index != self.index:
            self.index = index; self.slider.blockSignals(True); self.slider.setValue(index)
            self.slider.blockSignals(False); self.show_frame()
        if self.clock >= self.times[-1]:
            self.playing = False; self.play_button.setText('재생 [Space]')

    def show_frame(self):
        f = self.data['frames'][self.index]
        self.log_tab.update_time(f)
        white_source = 'mission' if self.white_mode.currentIndex() == 0 else 'lane'
        color_source = 'lane' if self.color_source.currentIndex() == 0 else 'mission'
        source_name = '차선' if color_source == 'lane' else '미션'
        views = render.panels(self.data, f, self.hsv_settings, white_source, color_source)
        preview = self.hsv_settings != self.data['hsv_applied']
        self.pictures['lane'].title = '현재 차선 BEV · 마지막 재계산 결과'
        self.pictures['processed'].title = f'보정 카메라 · {source_name} 설정'
        self.pictures['line'].title = ('정지선 BEV · 조정 미리보기' if preview
                                       else '정지선 BEV · 중앙 검사 영역 / 거리')
        self.pictures['white'].title = ('정지선 흰색 마스크 · 상시 미리보기' if white_source == 'mission'
                                        else '차선 흰색 마스크 · 구분선 필터 전')
        self.pictures['yellow'].title = '차선 노란색 마스크 · '+self.hsv_settings['lane']['color_method'].upper()
        for key, picture in self.pictures.items():
            pixel_key = {'camera': 'raw_pixels', 'processed': 'processed_pixels',
                         'yellow': 'lane_pixels', 'line': 'mission_pixels',
                         'white': 'lane_pixels' if white_source == 'lane' else 'mission_pixels',
                         'hsv': 'color_bev_pixels'}.get(key)
            pixels = views[pixel_key] if pixel_key else None
            hsv = cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV) if pixels is not None else None
            image = views[key]
            if key == 'hsv':
                mode = self.hsv_mode.currentIndex()
                image = views['hsv_views'][mode]
                picture.title = f'{source_name} 보정 BEV · '+self.hsv_mode.currentText()
                if mode == 3:
                    hsv = pixels = None
            picture.update_image(image, hsv, pixels)
        state, recorded = f['state'], f['recorded_state'] or {}
        fresh = f['detection_age'] is not None and f['detection_age'] <= self.data['params']['input_timeout']
        bump_boxes = [d for d in f['detections'] if d['name'] == 'sign_bump'] if fresh else []
        status = bump_boxes[0]['status'] if bump_boxes else '현재 검출 없음'
        if 'sign_bump' in state.get('seen', []):
            status = '표지판 확정'
        self.cards[0].setText('방지턱 표지판\n'+status)
        bump = state.get('bump_state', 'idle')
        self.cards[1].setText(f'현재: {BUMP_NAMES.get(bump, bump)}\n기록: '
                            f'{BUMP_NAMES.get(recorded.get("bump_state"), recorded.get("bump_state", "미기록"))}')
        self.cards[1].setStyleSheet(f'background:#203348;border-radius:5px;padding:8px;font-size:14px;color:{COLOR.get(bump,"white")}')
        self.cards[2].setText(f'차선 {"검출" if f["lane_found"] else "유실"} · 조향 {f["steering_deg"]:+.1f}°\n'
                            f'조향 잠금 {bool(state.get("steering_lock"))} · 가림 보완 {f["bump_gap"]}')
        number = lambda value: '--' if value is None else f'{value:.2f}'
        speed = None if f['command'] is None else f['command'][0]
        actual = None if f['recorded_command'] is None else f['recorded_command'][0]
        self.cards[3].setText(f'현재 {number(speed)} / 기록 {number(actual)} m/s\n'
                            f'미션 시작선 {state.get("entry_phase") or "비활성"} · {number(state.get("entry_remaining"))} m\n'
                            f'주차 진입선 {number(f.get("parking_entry_distance"))} m (차체 기준)\n'
                            f'정지선 {state.get("line_phase") or "비활성"} · {number(state.get("remaining"))} m')
        if preview:
            for card in self.cards:
                card.setText('마지막 재계산 결과\n'+card.text())
        self.time_label.setText(f'{f["t"]:.2f} / {self.data["duration"]:.2f} s')
        p = self.data['params']
        rows = ''.join('<tr><td>'+html.escape(d['name'])+'</td><td>'+f'{d["confidence"]:.2f}'+'</td><td>'+
                       html.escape(d['status'])+'</td></tr>' for d in f['detections'])
        settings = ''.join(f'<tr><td>{html.escape(k)}</td><td>{html.escape(str(v))}</td></tr>' for k, v in p.items())
        self.details.setHtml('<b>실제 코드에 적용된 설정</b><br>하늘색: 검사 영역 / 주황색: 미션 시작선 / '
            '초록색: 확정된 주차 진입선 / 자홍색: 정지선 후보<br>'
            '흰색 마스크는 상시 미리보기입니다. 범프의 높이나 모양을 영상으로 확정하는 기능은 현재 코드에 없습니다.<br><br>'
            '<b>YOLO 기록 → 현재 필터</b><table>'+rows+'</table><br><b>YAML / 기본값</b><table>'+settings+'</table>')
        self.timeline.at = f['t']; self.timeline.update()

    def open_bag(self):
        path = QFileDialog.getExistingDirectory(self, 'ROS 백 폴더', str(ROOT/'bag'))
        if path:
            self.bags.append(Path(path)); self.combo.addItem(Path(path).name)
            self.combo.setCurrentIndex(len(self.bags)-1)

    def save(self):
        path, _ = QFileDialog.getSaveFileName(self, '화면 저장', str(ROOT/'ros2_ws/log/bump_viewer.png'), 'PNG (*.png)')
        if path:
            self.grab().save(path)

    def test_ui(self):
        try:
            if getattr(self, '_tested_color', False):
                assert self.data['hsv_applied'] == self._test_color_expected
                assert abs(self.times[self.index]-self._test_color_time) < .001
                assert self.data['detector_values']['color_method'] == 'rgb'
                assert self.data['detector_values']['gamma_enabled']
                assert self.data['params']['white_v_min'] == self._test_hsv_baseline['mission']['white_v_min']
                assert self.data['hsv_applied']['mission'] == self._test_hsv_baseline['mission']
                assert self.log_tab is not None and all(p.image is not None for p in self.pictures.values())
                assert self.white_mode.isVisible(), 'white mask source selector must be visible'
                self.color_source.setCurrentIndex(1)
                assert '미션' in self.pictures['processed'].title
                self.color_source.setCurrentIndex(0)
                assert '차선' in self.pictures['processed'].title
                QApplication.processEvents()
                path = Path(self.self_test); path.parent.mkdir(parents=True, exist_ok=True)
                assert self.grab().save(str(path))
                print('GUI PASS: color presets, independent mission, Lab/RGB/CLAHE preview, YAML save/load, '
                      'RGB full replay, reset, eight panels, HSV controls, stepping and end-stop:', path, flush=True)
                QApplication.instance().quit()
                return
            if getattr(self, '_tested_hsv', False):
                assert abs(self.times[self.index]-self._test_hsv_time) < .001
            self.seek_bump(); initial = self.index; self.step(1); assert self.index == min(initial+1, len(self.times)-1)
            self.seek_index(len(self.times)-1); self.playing = True; self.clock = self.times[-1]+.1; self.tick()
            assert not self.playing and self.index == len(self.times)-1
            self.seek_bump(); assert all(p.image is not None for p in self.pictures.values())
            if not getattr(self, '_tested_reload', False):
                self._tested_reload = True
                self.reload_button.click()
                return
            if not getattr(self, '_tested_hsv', False):
                self._tested_hsv = True
                self._test_hsv_baseline = deepcopy(self.data['hsv_baseline'])
                self.side_tabs.setCurrentWidget(self.log_tab)
                self.white_mode.setCurrentIndex(1)
                self.stopline_settings_button.click()
                assert self.side_tabs.currentWidget() is self.hsv_tab
                assert self.color_sections.currentWidget() is self.stopline_color_page
                assert self.white_mode.currentIndex() == 0 and self.color_source.currentIndex() == 1
                assert self.hsv_controls['mission', 'white_v_min', None][1].isVisible()
                assert self.hsv_settings == self._test_hsv_baseline
                self.color_sections.setCurrentIndex(0)
                self.hsv_controls['lane', 'hsv_lower', 0][0].setValue(179)
                assert self.hsv_settings['lane']['hsv_upper'][0] == 179
                self.hsv_controls['lane', 'hsv_upper', 0][0].setValue(0)
                assert self.hsv_settings['lane']['hsv_lower'][0] == 0
                self.hsv_reset_button.click()
                for group in ('lane', 'mission'):
                    self.hsv_controls[group, 'white_s_max', None][1].setValue(255)
                    self.hsv_controls[group, 'white_v_min', None][1].setValue(0)
                    source = 'lane' if group == 'lane' else 'mission'
                    before = render.panels(self.data, self.data['frames'][self.index],
                                           self.hsv_settings, source)['white'].copy()
                    self.hsv_controls[group, 'white_h_min', None][0].setValue(179)
                    assert self.hsv_settings[group]['white_h_max'] == 179
                    after = render.panels(self.data, self.data['frames'][self.index],
                                          self.hsv_settings, source)['white']
                    assert not np.array_equal(before, after)
                    self.hsv_controls[group, 'white_h_max', None][1].setValue(0)
                    assert self.hsv_settings[group]['white_h_min'] == 0
                    self.hsv_reset_button.click()
                self.seek_time(self.times[-1]*.6)
                self._test_hsv_time = self.times[self.index]
                self.hsv_controls['lane', 'hsv_upper', 2][0].setValue(0)
                assert self.hsv_settings['lane']['hsv_lower'][2] == 0
                assert not np.any(render.panels(self.data, self.data['frames'][self.index],
                                                self.hsv_settings)['yellow'])
                self.hsv_controls['mission', 'white_v_min', None][1].setValue(255)
                for group in ('lane', 'mission'):
                    self.hsv_controls[group, 'white_h_max', None][1].setValue(45)
                self.hsv_copy_button.click()
                copied = yaml.safe_load(QApplication.clipboard().text())
                for node in ('lane_main', 'mission'):
                    assert copied[node]['ros__parameters']['white_h_max'] == 45
                assert self.data['params']['white_v_min'] != 255  # preview does not relabel old results
                self._test_hsv_expected = deepcopy(self.hsv_settings)
                assert self.hsv_apply_button.isEnabled()
                self.hsv_apply_button.click()
                return
            assert self.data['hsv_applied'] == self._test_hsv_expected
            assert self.data['hsv_baseline'] == self._test_hsv_baseline
            assert self.data['params']['white_v_min'] == 255
            assert self.data['params']['white_h_max'] == 45
            assert self.data['detector_values']['white_h_max'] == 45
            assert self.data['detector_values']['hsv_upper'][2] == 0
            self.hsv_reset_button.click()
            assert self.hsv_settings == self._test_hsv_baseline
            assert self.hsv_apply_button.isEnabled()
            self.white_mode.setCurrentIndex(1)
            assert '차선 흰색' in self.pictures['white'].title
            self.white_mode.setCurrentIndex(0)
            self.preset_combo.setCurrentIndex(1); self.preset_button.click()
            assert self.hsv_settings['lane']['gamma_enabled'] and self.hsv_settings['lane']['color_method'] == 'rgb'
            assert self.hsv_settings['mission'] == self._test_hsv_baseline['mission']
            self.color_controls['lane', 'color_method'].setCurrentText('lab')
            self.color_controls['lane', 'clahe_enabled'].setChecked(True)
            self.refresh_hsv_preview()
            assert self.hsv_settings['lane']['clahe_enabled']
            assert self.method_boxes['lane', 'lab'][0].isVisible()
            assert not self.method_boxes['lane', 'rgb'][0].isVisible()
            assert '마지막 재계산' in self.cards[2].text()
            self.preset_combo.setCurrentIndex(2); self.preset_button.click()
            assert self.hsv_settings['lane']['rgb_yellow_difference_min'] == 12
            self.preset_combo.setCurrentIndex(1); self.preset_button.click()
            assert not self.hsv_settings['lane']['clahe_enabled']
            # Exercise the actual file buttons while directing dialogs to a disposable file.
            import tempfile
            from unittest.mock import patch
            with tempfile.TemporaryDirectory() as folder:
                filename = str(Path(folder)/'colors.yaml')
                expected = deepcopy(self.hsv_settings)
                with patch.object(QFileDialog, 'getSaveFileName', return_value=(filename, '')):
                    self.color_save_button.click()
                self.hsv_reset_button.click()
                with patch.object(QFileDialog, 'getOpenFileName', return_value=(filename, '')):
                    self.color_load_button.click()
                assert self.hsv_settings == expected
            self._test_color_expected = deepcopy(self.hsv_settings)
            self._test_color_time = self.times[self.index]
            self._tested_color = True
            self.hsv_apply_button.click()
        except Exception:
            import traceback
            traceback.print_exc(); QApplication.instance().exit(1)

    def closeEvent(self, event):
        if self.worker and self.worker.isRunning():
            self.worker.requestInterruption()
            if not self.worker.wait(3000):
                event.ignore(); return
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', type=Path)
    parser.add_argument('--camera', choices=['limo1', 'limo2'], default='limo2')
    parser.add_argument('--at', type=float)
    parser.add_argument('--self-test', type=Path)
    args = parser.parse_args()
    bags = backend.discover()
    preferred = args.bag or ROOT/'bag/bag_1111_035527'
    if preferred.exists():
        preferred = preferred.resolve(); bags = [preferred]+[p for p in bags if p.resolve() != preferred]
    if not bags:
        raise SystemExit('재생할 백이 없습니다. 백 경로를 지정하세요.')
    app = QApplication(sys.argv[:1]); app.setFont(QFont('Noto Sans CJK KR', 10))
    window = Viewer(bags, args.camera, args.at, args.self_test); window.show()
    raise SystemExit(app.exec_())


if __name__ == '__main__':
    main()
