"""Time-linked diagnostics with explicit recorded versus replay provenance."""
import bisect
import html
import json
from pathlib import Path
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QLineEdit, QCheckBox, QListWidget, QListWidgetItem, QTextBrowser, QPushButton,
    QFileDialog, QApplication, QMessageBox)
from . import logs


class LogPanel(QWidget):
    seek = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.data = None; self.at = 0.; self.visible = []; self.rows = []
        self.pinned_files = {}
        layout = QVBoxLayout(self)
        self.summary = QTextBrowser(); self.summary.setMinimumHeight(65); self.summary.setMaximumHeight(100)
        layout.addWidget(self.summary)
        self.file_label = QLabel(); self.file_label.setWordWrap(True); layout.addWidget(self.file_label)
        buttons = QHBoxLayout()
        open_button = QPushButton('실행 로그 선택'); open_button.clicked.connect(self.open_file)
        copy_button = QPushButton('원문 복사'); copy_button.clicked.connect(self.copy)
        buttons.addWidget(open_button); buttons.addWidget(copy_button); layout.addLayout(buttons)
        self.source = QComboBox(); self.source.addItems(['전체 출처', '실제 기록', '현재 재계산'])
        self.node = QComboBox(); self.level = QComboBox()
        self.level.addItems(['전체 등급', 'INFO', 'WARN', 'ERROR', 'FATAL', 'DEBUG', 'STATE', 'TEXT'])
        filters = QHBoxLayout()
        for box in (self.source, self.node, self.level):
            filters.addWidget(box); box.currentIndexChanged.connect(self.refilter)
        layout.addLayout(filters)
        self.search = QLineEdit(); self.search.setPlaceholderText('노드·미션·원문 검색')
        self.search.textChanged.connect(self.refilter); layout.addWidget(self.search)
        options = QHBoxLayout()
        self.near = QCheckBox('현재 ±3초'); self.follow = QCheckBox('시간 따라가기'); self.follow.setChecked(True)
        self.near.toggled.connect(self.refilter)
        options.addWidget(self.near); options.addWidget(self.follow); layout.addLayout(options)
        self.count = QLabel(); layout.addWidget(self.count)
        self.list = QListWidget(); self.list.setMinimumHeight(155); layout.addWidget(self.list, 1)
        self.list.currentItemChanged.connect(self.show_raw)
        self.list.itemClicked.connect(self.jump)
        self.raw = QTextBrowser(); self.raw.setMinimumHeight(60); self.raw.setMaximumHeight(90); layout.addWidget(self.raw)
        note = QLabel('미기록 ≠ 미작동 · 재계산 범위: 미션·차선')
        note.setToolTip('STATE는 기록된 상태·토픽 변화입니다. 현재 재계산은 콘·주차 등의 전체 제어 재현이 아닙니다.')
        layout.addWidget(note)

    def set_data(self, data):
        self.data = data
        pinned = self.pinned_files.get(str(Path(data['path']).resolve()))
        if pinned:
            try:
                data['file_logs'] = logs.read_file(pinned, data['start_ns']); data['log_file'] = pinned
            except OSError:
                pass
        self.rows = logs.merge(data['recorded_logs']+data['file_logs']+data['replay_logs'])
        self.state_times = [t for t, _ in data['recorded_states']]
        self.signal_series = {}
        for t, name, value in data['recorded_signals']:
            series = self.signal_series.setdefault(name, ([], []))
            series[0].append(t); series[1].append(value)
        self.node.blockSignals(True); old = self.node.currentText(); self.node.clear()
        self.node.addItems(['전체 노드']+sorted({r['node'] for r in self.rows}))
        self.node.setCurrentIndex(max(0, self.node.findText(old))); self.node.blockSignals(False)
        self.file_label.setText('파일: '+(data['log_file'] or '같은 이름의 .log 없음 · /rosout 및 상태 기록 사용'))
        self.refilter()

    def open_file(self):
        if self.data is None:
            return
        path, _ = QFileDialog.getOpenFileName(self, '실행 로그 선택', str(Path(self.data['path']).parent),
                                              'Log files (*.log *.txt);;All files (*)')
        if not path:
            return
        try:
            rows = logs.read_file(path, self.data['start_ns'])
        except OSError as exc:
            QMessageBox.warning(self, '로그 읽기 실패', str(exc)); return
        self.pinned_files[str(Path(self.data['path']).resolve())] = path
        self.data.update(file_logs=rows, log_file=path); self.set_data(self.data)

    def refilter(self, *_):
        if self.data is None:
            return
        self.list.blockSignals(True); self.list.clear(); self.visible = []
        query = self.search.text().casefold()
        for r in self.rows:
            if self.source.currentIndex() == 1 and r['source'] == '현재 재계산': continue
            if self.source.currentIndex() == 2 and r['source'] != '현재 재계산': continue
            if self.node.currentIndex() > 0 and r['node'] != self.node.currentText(): continue
            if self.level.currentIndex() > 0 and r['level'] != self.level.currentText(): continue
            if query and query not in (r['node']+' '+r['message']+' '+r['raw']).casefold(): continue
            if self.near.isChecked() and (r['t'] is None or abs(r['t']-self.at)>3): continue
            stamp = '시간 없음' if r['t'] is None else f'{r["t"]:.2f}s'
            label = f'{stamp} [{r["source"]}] [{r["level"]}] {r["node"]}: {r["message"]}'
            item = QListWidgetItem(label); item.setData(Qt.UserRole, len(self.visible))
            item.setToolTip(label)
            color = '#ff8585' if r['level'] in ('ERROR','FATAL') else '#ffca75' if r['level']=='WARN' else '#81dfc0' if r['source']=='현재 재계산' else '#dce9f5'
            item.setForeground(QColor(color)); self.list.addItem(item); self.visible.append(r)
        self.list.blockSignals(False)
        self.timed = [(r['t'], i) for i, r in enumerate(self.visible) if r['t'] is not None]
        self.timed_values = [t for t, _ in self.timed]
        self.count.setText(f'{len(self.visible):,} / {len(self.rows):,}개 · 시간 없는 줄: 전체 보기')
        self.raw.clear()
        self.highlight()

    def highlight(self):
        if self.follow.isChecked() and self.timed:
            i = bisect.bisect_right(self.timed_values, self.at)-1
            if i >= 0:
                self.list.setCurrentRow(self.timed[i][1]); self.list.scrollToItem(self.list.currentItem())

    def show_raw(self, item, _previous=None):
        if item is None: return
        r = self.visible[item.data(Qt.UserRole)]
        detail = r.get('location','')+'\n'+r['raw']
        if 'rosout_raw' in r: detail += '\n\n[/rosout 원문]\n'+r['rosout_raw']
        if r['t'] is None:
            detail = '정확한 시각 없음 · 자동으로 영상 시점을 추정하지 않습니다.\n'+detail
        elif not self.data['frames'][0]['t'] <= r['t'] <= self.data['frames'][-1]['t']:
            detail = '카메라 녹화 구간 밖 · 대응 영상 없음\n'+detail
        self.raw.setPlainText(detail)

    def copy(self):
        QApplication.clipboard().setText(self.raw.toPlainText())

    def jump(self, item):
        r = self.visible[item.data(Qt.UserRole)]
        if r['t'] is not None and self.data['frames'][0]['t'] <= r['t'] <= self.data['frames'][-1]['t']:
            self.seek.emit(r['t'])

    def update_time(self, frame):
        if self.data is None: return
        self.at = frame['t']
        i = bisect.bisect_right(self.state_times, self.at)-1
        if i >= 0:
            t, state = self.data['recorded_states'][i]
            actual = f'마지막 관측 {self.at-t:.2f}초 전'+'<br>'+self.state_html(state)
        else:
            actual = '미기록 / 확인 불가'
        signals = []
        for name, (times, values) in self.signal_series.items():
            j = bisect.bisect_right(times, self.at)-1
            if j >= 0:
                signals.append(f'{html.escape(name)} = {html.escape(str(values[j]))} ({self.at-times[j]:.1f}초 전 관측)')
        self.summary.setHtml('<b>당시 실제 미션 상태</b><br>'+actual+
            '<br><b>현재 코드 재계산</b><br>'+self.state_html(frame['state'])+
            '<br><b>실제 제어·완료 토픽: 마지막 관측값</b><br>'+('<br>'.join(signals) or '미기록 / 확인 불가'))
        if self.near.isChecked(): self.refilter()
        else: self.highlight()

    @staticmethod
    def state_html(state):
        labels = {'state':'주행 상태','seen':'확정 표지판','armed':'시작 조건 대기',
                  'bump_state':'방지턱','entry_phase':'미션 시작선','line_phase':'정지선',
                  'parking_prepare':'주차 준비','parking_scene':'주차 공간 관측',
                  'parking_observing':'주차 관측 중','parking_skipped':'주차 건너뜀',
                  'cone_entered':'콘 진입 이력','steering_lock':'조향 잠금'}
        words = {'drive':'주행', 'idle':'대기', 'approaching':'접근·서행', 'on_bump':'방지턱 통과 중',
                 'cooldown':'통과 후 대기', 'search':'시작선 탐색', 'confirmed':'확정',
                 'braking':'감속 중', 'entered':'진입 확인', 'follow':'접근 중',
                 'commit':'정지 목표 확정', 'arrived':'정지 위치 도달',
                 'pedestrian':'보행자', 'light':'신호등', 'slow':'서행', 'bump_line':'방지턱 시작선'}
        def value(v):
            if v is None: return '비활성 / 값 없음'
            if isinstance(v, bool): return '예' if v else '아니오'
            if isinstance(v, list): return ', '.join(value(x) for x in v) or '없음'
            return words.get(str(v), str(v))
        return '<br>'.join(html.escape(label)+': '+html.escape(value(state[key]))
                           for key,label in labels.items() if key in state) or '미기록 / 확인 불가'
