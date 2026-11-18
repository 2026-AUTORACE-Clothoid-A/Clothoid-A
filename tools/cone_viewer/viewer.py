#!/usr/bin/env python3
"""Native Qt cone bag viewer. Offline only; no ROS node or command publisher."""
import argparse
import bisect
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/clothoid_lidar'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from PyQt5.QtCore import Qt, QPointF, QRectF, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPolygonF, QKeySequence
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog, QHBoxLayout,
    QLabel, QListWidget, QMainWindow, QMessageBox, QPushButton, QShortcut, QSlider,
    QSplitter, QVBoxLayout, QWidget)
from replay import analyse

COLORS = dict(scan='#65788b', obstacles='#c7d2df', left='#5ad4f5', right='#ffb554',
              center='#61e6a2', target='#f78bd5', body='#e4edf5', turn='#ac9dff')

class Loader(QThread):
    loaded = pyqtSignal(object)
    failed = pyqtSignal(str)
    progress = pyqtSignal(int)
    def __init__(self, path):
        super().__init__(); self.path = path
    def run(self):
        try:self.loaded.emit(analyse(self.path, self.progress.emit))
        except Exception as error:
            import traceback
            traceback.print_exc(); self.failed.emit(str(error))

class Camera(QWidget):
    def __init__(self):
        super().__init__(); self.image = None; self.text = '카메라 로딩 중'; self.setMinimumSize(300,220)
    def paintEvent(self, event):
        p=QPainter(self);p.fillRect(self.rect(),QColor('#0e1620'))
        if self.image is not None and not self.image.isNull():
            size=self.image.size();size.scale(self.width(),self.height()-36,Qt.KeepAspectRatio)
            target=QRectF((self.width()-size.width())/2,36+(self.height()-36-size.height())/2,size.width(),size.height())
            p.drawImage(target,self.image)
        p.setPen(QColor('#dce7f1'));p.drawText(12,24,self.text)

class Map(QWidget):
    def __init__(self):
        super().__init__();self.frame=None;self.params=None;self.show_raw=True;self.show_turn=True
        self.setMinimumSize(400,420)
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing);p.fillRect(self.rect(),QColor('#111d2a'))
        scale=min((self.width()-50)/3.6,(self.height()-70)/3.2)
        origin=QPointF(self.width()/2,self.height()-50-.55*scale)
        def xy(point):return QPointF(origin.x()-float(point[1])*scale,origin.y()-float(point[0])*scale)
        def line(points,color,width=2,style=Qt.SolidLine):
            if len(points)<2:return
            p.setPen(QPen(QColor(color),width,style));p.setBrush(Qt.NoBrush);p.drawPolyline(QPolygonF([xy(a) for a in points]))
        p.setPen(QPen(QColor('#293c4d'),1))
        for x in np.arange(-.5,2.6,.5):
            p.drawLine(xy((x,-1.7)),xy((x,1.7)))
            p.setPen(QColor('#a4b5c4'));p.drawText(QPointF(8,xy((x,0)).y()),f'{x:.1f}m');p.setPen(QColor('#293c4d'))
        for y in np.arange(-1.5,1.6,.5):p.drawLine(xy((-.5,y)),xy((2.5,y)))
        p.setPen(QColor('#dce7f1'));p.drawText(12,22,'라이다 · 경로 / 차량 기준');p.drawText(self.width()-130,22,'↑ 전방   ← 좌측')
        if self.frame is None:return
        f=self.frame
        if self.show_raw:
            p.setPen(QPen(QColor(COLORS['scan']),2))
            for a in f['scan']:p.drawPoint(xy(a))
        p.setPen(QPen(QColor(COLORS['obstacles']),1));p.setBrush(Qt.NoBrush)
        for a in f['obstacles']:p.drawEllipse(xy(a),.04*scale,.04*scale)
        for side in ('left','right'):
            color=COLORS[side]
            line(f['stale_'+side],color,1,Qt.DotLine)
            for a in f['stale_'+side]:
                pos=xy(a);p.setPen(QPen(QColor(color),1,Qt.DotLine));p.setBrush(Qt.NoBrush);p.drawEllipse(pos,5,5)
            line(f[side],color,2)
            for a in f[side]:
                pt=xy(a);p.setBrush(QColor(color));p.setPen(Qt.NoPen)
                if side=='left':p.drawPolygon(QPolygonF([pt+QPointF(0,-6),pt+QPointF(-5,5),pt+QPointF(5,5)]))
                else:p.drawRect(QRectF(pt.x()-4,pt.y()-4,8,8))
        c=f['command']
        if c is not None:
            line(c.center,COLORS['center'],4)
            if len(c.center):
                p.setPen(QPen(QColor(COLORS['center']),2));p.setBrush(Qt.NoBrush)
                p.drawEllipse(xy(c.center[-1]),6,6)
            endpoint=f.get('observed_end')
            if endpoint is not None:
                point=xy(endpoint);p.setPen(QPen(QColor('#ffe080'),3))
                p.drawLine(point+QPointF(-6,-6),point+QPointF(6,6))
                p.drawLine(point+QPointF(-6,6),point+QPointF(6,-6))
            if c.target is not None:
                p.setBrush(QColor(COLORS['target']));p.setPen(QPen(QColor(COLORS['target']),2));p.drawEllipse(xy(c.target),7,7)
                line([[-self.params.rear_axle_x,0],c.target],COLORS['target'],1,Qt.DotLine)
            if self.show_turn:
                limit=math.atan(self.params.wheelbase/(self.params.wheelbase/math.tan(math.radians(28))+.175/2))
                steer=np.clip(c.steering,-limit,limit);k=math.tan(steer)/self.params.wheelbase
                s=np.linspace(0,-.5 if c.reverse else 1.,40)
                trajectory=np.column_stack((s-self.params.rear_axle_x,np.zeros(len(s)))) if abs(k)<1e-6 else np.column_stack((np.sin(s*k)/k-self.params.rear_axle_x,(1-np.cos(s*k))/k))
                line(trajectory,COLORS['turn'],2,Qt.DashLine)
        p.setBrush(QColor('#304659'));p.setPen(QPen(QColor(COLORS['body']),2))
        p.drawPolygon(QPolygonF([xy(a) for a in ((.161,.110),(.161,-.110),(-.161,-.110),(-.161,.110))]))
        line([(0,0),(.23,0)],COLORS['body'],2)
        p.setPen(QColor('#c6d3df'));p.drawText(12,self.height()-15,'1 grid = 0.5 m     base_link x前 / y左')

class Viewer(QMainWindow):
    def __init__(self,bags,at=None,play=False,self_test=None):
        super().__init__();self.setWindowTitle('Cone Viewer — 현재 코드 · 오프라인 백');self.resize(1500,880)
        self.bags=list(bags);self.data=None;self.index=0;self.playing=False;self.initial_at=at;self.initial_play=play;self.self_test=self_test
        self.worker=None;self.last_wall=time.monotonic();self.elapsed=0.
        root=QWidget();layout=QVBoxLayout(root);layout.setContentsMargins(16,12,16,12);layout.setSpacing(10);self.setCentralWidget(root)
        top=QHBoxLayout();title=QLabel('CONE VIEWER');title.setStyleSheet('font-size:20px;font-weight:600;color:#edf5fc');top.addWidget(title)
        top.addStretch();self.combo=QComboBox()
        for bag in bags:self.combo.addItem(bag.name)
        self.combo.setMinimumWidth(260);self.combo.currentIndexChanged.connect(self.load_selected);top.addWidget(self.combo)
        self.open_button=QPushButton('백 폴더 열기');self.open_button.clicked.connect(self.open_bag);top.addWidget(self.open_button)
        self.reload=QPushButton('현재 코드로 재계산');self.reload.clicked.connect(self.load_selected);top.addWidget(self.reload);layout.addLayout(top)
        self.source=QLabel('현재 소스 로딩 중');self.source.setStyleSheet('color:#a9bdce');layout.addWidget(self.source)
        self.status=QLabel('분석 중');self.status.setStyleSheet('background:#203245;padding:10px;font-size:15px');layout.addWidget(self.status)
        self.metrics=QLabel('');self.metrics.setStyleSheet('color:#c5d4e2');layout.addWidget(self.metrics)
        views=QSplitter(Qt.Horizontal);self.camera=Camera();self.map=Map();self.events=QListWidget();self.events.setMinimumWidth(180)
        event_panel=QWidget();evlayout=QVBoxLayout(event_panel);evlayout.setContentsMargins(3,0,0,0);evlayout.addWidget(QLabel('이벤트 · 클릭해서 이동'));evlayout.addWidget(self.events)
        self.events.itemClicked.connect(lambda item:self.seek_time(item.data(Qt.UserRole)))
        views.addWidget(self.camera);views.addWidget(self.map);views.addWidget(event_panel);views.setSizes([580,650,220]);layout.addWidget(views,1)
        legend=QHBoxLayout()
        for color,text in [(COLORS['left'],'▲ 왼쪽 콘'),(COLORS['right'],'■ 오른쪽 콘'),(COLORS['center'],'━ 경로 / ○ 끝점'),('#ffe080','× 관측 경로 끝'),(COLORS['target'],'● 목표점'),(COLORS['turn'],'┄ 조향 궤적')]:
            label=QLabel(text);label.setStyleSheet(f'color:{color}');legend.addWidget(label)
        legend.addStretch();layout.addLayout(legend)
        row=QHBoxLayout();self.time_label=QLabel('0.00 s');self.time_label.setMinimumWidth(145);row.addWidget(self.time_label)
        self.slider=QSlider(Qt.Horizontal);self.slider.valueChanged.connect(self.seek_index);row.addWidget(self.slider,1);layout.addLayout(row)
        row=QHBoxLayout();self.play_button=QPushButton('재생 [Space]');self.play_button.clicked.connect(self.toggle);row.addWidget(self.play_button)
        for text,callback in [('◀ 1프레임',lambda:self.step(-1)),('1프레임 ▶',lambda:self.step(1)),('콘 시작',self.seek_start)]:
            button=QPushButton(text);button.clicked.connect(callback);row.addWidget(button)
        self.speed=QComboBox();self.speed.addItems(['0.25×','0.5×','1×','2×']);self.speed.setCurrentIndex(2);row.addWidget(self.speed)
        for label,field in [('원본 라이다','show_raw'),('조향 궤적','show_turn')]:
            checkbox=QCheckBox(label);checkbox.setChecked(True);checkbox.toggled.connect(lambda value,key=field:self.set_layer(key,value));row.addWidget(checkbox)
        row.addStretch();save=QPushButton('현재 화면 저장');save.clicked.connect(self.save);row.addWidget(save);layout.addLayout(row)
        footer=QLabel('카메라·기록 명령 = 원본 백 | 콘 경로 = 현재 코드 | 스캔 단위 재계산·ROS 콜백 지연 미재현 | 끝에서 정지')
        footer.setStyleSheet('color:#a9bdce');layout.addWidget(footer)
        self.shortcuts=[]
        for sequence,callback in [('Space',self.toggle),('Left',lambda:self.step(-1)),('Right',lambda:self.step(1))]:
            shortcut=QShortcut(QKeySequence(sequence),self);shortcut.activated.connect(callback);self.shortcuts.append(shortcut)
        self.timer=QTimer(self);self.timer.setInterval(30);self.timer.timeout.connect(self.tick);self.timer.start()
        QTimer.singleShot(0,self.load_selected)
    def set_layer(self,field,value):setattr(self.map,field,value);self.map.update()
    def open_bag(self):
        name=QFileDialog.getExistingDirectory(self,'ROS 백 폴더',str(ROOT/'bag'))
        if name:
            self.bags.append(Path(name));self.combo.addItem(Path(name).name);self.combo.setCurrentIndex(len(self.bags)-1)
    def load_selected(self,*args):
        if self.worker is not None and self.worker.isRunning():return
        self.playing=False;self.play_button.setText('재생 [Space]');self.data=None
        self.camera.image=None;self.map.frame=None;self.camera.update();self.map.update()
        for widget in (self.combo,self.reload,self.open_button,self.slider,self.play_button):widget.setEnabled(False)
        self.status.setText('현재 소스 코드로 분석 중…');self.events.clear();self.worker=Loader(self.bags[self.combo.currentIndex()])
        self.worker.progress.connect(lambda n:self.status.setText(f'현재 소스 코드로 분석 중… {n}%'))
        self.worker.loaded.connect(self.loaded);self.worker.failed.connect(self.failed);self.worker.start()
    def loaded(self,data):
        self.data=data;self.times=[f['t'] for f in data['frames']];self.camera_times=[r[0] for r in data['cameras']];self.recorded_times=[r[0] for r in data['recorded']]
        self.map.params=data['params'];self.slider.setRange(0,len(self.times)-1)
        for widget in (self.combo,self.reload,self.open_button,self.slider,self.play_button):widget.setEnabled(True)
        self.source.setText(f"{data['path']}  |  소스 {data['source_hash']}  |  {len(self.times)} scans / {len(data['cameras'])} images")
        for t,label in data['events']:
            self.events.addItem(f'{t:7.2f}s  {label}');self.events.item(self.events.count()-1).setData(Qt.UserRole,t)
        if self.initial_at is not None:self.seek_time(self.initial_at);self.initial_at=None
        else:self.seek_start()
        if self.initial_play:self.toggle();self.initial_play=False
        if self.self_test:QTimer.singleShot(150,self.run_self_test)
    def failed(self,message):
        for w in (self.combo,self.reload,self.open_button):w.setEnabled(True)
        self.status.setText('분석 실패 · '+message)
        if self.self_test:print(message,flush=True);QApplication.exit(1)
        else:QMessageBox.warning(self,'백 분석 실패',message)
    def seek_start(self):
        if self.data:self.seek_time(next((t for t,label in self.data['events'] if label=='콘 시작'),self.times[0])+.15)
    def seek_time(self,t):
        if self.data:self.seek_index(min(len(self.times)-1,max(0,bisect.bisect_left(self.times,t))))
    def seek_index(self,index):
        if not self.data:return
        self.index=int(np.clip(index,0,len(self.times)-1));self.elapsed=self.times[self.index]
        self.slider.blockSignals(True);self.slider.setValue(self.index);self.slider.blockSignals(False);self.draw()
    def step(self,delta):
        self.playing=False;self.play_button.setText('재생 [Space]');self.seek_index(self.index+delta)
    def toggle(self):
        if not self.data:return
        if self.index==len(self.times)-1 and not self.playing:self.seek_index(0)
        self.playing=not self.playing;self.last_wall=time.monotonic();self.play_button.setText('일시정지 [Space]' if self.playing else '재생 [Space]')
    def tick(self):
        now=time.monotonic();dt=now-self.last_wall;self.last_wall=now
        if self.playing and self.data:
            self.elapsed+=dt*[.25,.5,1,2][self.speed.currentIndex()]
            index=min(len(self.times)-1,max(0,bisect.bisect_right(self.times,self.elapsed)-1))
            if index!=self.index:
                elapsed=self.elapsed;self.seek_index(index);self.elapsed=elapsed
            if self.elapsed>=self.times[-1]:self.playing=False;self.play_button.setText('재생 [Space]')
    def draw(self):
        f=self.data['frames'][self.index];t=f['t'];self.map.frame=f;self.map.update();self.time_label.setText(f'{t:.2f} / {self.times[-1]:.2f} s')
        color='#61e6a2' if f['state']=='전진 경로' else '#ffb554' if f['active'] else '#afbecb'
        self.status.setStyleSheet(f'background:#203245;padding:10px;font-size:15px;color:{color}')
        c=f['command'];details=''
        if c is not None:
            details=f'   v {c.speed:+.2f} m/s  ·  조향 요청 {math.degrees(c.steering):+.1f}°'
            limit=math.atan(self.map.params.wheelbase/(self.map.params.wheelbase/math.tan(math.radians(28))+.175/2))
            if abs(c.steering)>limit:details+=f'  → 차체 제한 {math.degrees(np.clip(c.steering,-limit,limit)):+.1f}°'
        if f.get('observed_end') is not None:
            distance=np.linalg.norm(f['observed_end']+np.array([self.map.params.rear_axle_x,0.]))
            details+=f'   관측 끝점까지 {distance*100:.0f}cm (후륜 기준)'
        if f.get('rear_remaining') is not None:
            details+=f"   후차축 통과까지 {f['rear_remaining']*100:+.0f}cm"
        if f.get('lane_image') is not None:
            found, speed = f['lane_image']
            details += '   카메라 차선 ' + ('검출' if found else '유실')
            details += ' · 차선 명령 ' + (f'{speed:.2f} m/s' if speed is not None else '없음')
        self.status.setText(f"{f['state']}   |   좌 {len(f['left'])} / 우 {len(f['right'])}개"+details)
        recorded='기록 명령 없음';ri=bisect.bisect_right(self.recorded_times,t)-1
        if ri>=0 and t-self.recorded_times[ri]<=.3:
            _,v,w=self.data['recorded'][ri];recorded=f'기록 /cmd_vel  v {v:+.2f} m/s · ω {w:+.2f} rad/s'
        methods={'camera wait':'카메라 차선 후보 대기','camera blocked':'차선 경로 라이다 검사',
                 'camera lane':'검출 차선 저속 추종','observed':'main 중심 경로·호장 목표점', 'main_reverse':'main 한쪽 경계 후진',
                 'exit_path':'콘 출구 저장 경로·후축 원 목표점', 'exit_wait':'탈출 관측 대기 정지',
                 'lane_connection':'저속 차선 연결', 'lane_connection_wait':'차선 연결 입력·피드백 대기'}
        method=methods[c.source] if c is not None and c.source in methods else ('안쪽 콘–바깥 경계 중간점' if f['fan'] else '기존 중심 경로' if c is not None and not c.reverse else '전진 경로 없음')
        self.metrics.setText(f"재계산 방식  {method}   |   {recorded}   |   이 스캔 명령 {len(f['callback_results'])}회")
        ci=bisect.bisect_left(self.camera_times,t)
        if self.camera_times:
            ci=min((max(0,ci-1),min(ci,len(self.camera_times)-1)),key=lambda j:abs(self.camera_times[j]-t))
            ct,image=self.data['cameras'][ci]
            self.camera.image=QImage.fromData(image) if isinstance(image,bytes) else image
            self.camera.text=f'원본 카메라  {ct:.2f}s   ·   시간차 {abs(ct-t)*1000:.0f} ms'
        else:self.camera.text='이 백에는 카메라 영상이 없습니다.'
        self.camera.update()
    def save(self):
        name,_=QFileDialog.getSaveFileName(self,'화면 저장',str(ROOT/'ros2_ws/log/cone-viewer.png'),'PNG (*.png)')
        if name:self.grab().save(name)
    def run_self_test(self):
        try:
            assert self.data and self.data['frames']
            before=self.index;self.step(1);assert self.index==min(before+1,len(self.times)-1)
            self.step(-1);assert self.index==before
            self.seek_index(len(self.times)-1);self.playing=True;self.elapsed=self.times[-1]+1;self.tick();assert not self.playing
            target=next((f['t'] for f in self.data['frames'] if f['fan']),self.times[len(self.times)//2]);self.seek_time(target)
            assert self.camera.image is not None and not self.camera.image.isNull()
            self.grab().save(self.self_test)
            print(f'GUI PASS: frame stepping, end-stop, camera decode, screenshot {self.self_test}',flush=True)
            QApplication.exit(0)
        except Exception:
            import traceback;traceback.print_exc();QApplication.exit(1)
    def closeEvent(self,event):
        if self.worker is not None and self.worker.isRunning():
            self.status.setText('분석 종료를 기다리는 중…');event.ignore();return
        self.timer.stop();event.accept()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bags',nargs='*',type=Path);p.add_argument('--at',type=float);p.add_argument('--play',action='store_true');p.add_argument('--self-test',type=str)
    args=p.parse_args();bags=args.bags or sorted((ROOT/'bag/cone_latest4_review').glob('bag_*'),reverse=True)
    if not bags:p.error('백 폴더를 지정하세요.')
    app=QApplication(sys.argv[:1]);app.setFont(QFont('Noto Sans CJK KR',10))
    app.setStyleSheet('QMainWindow,QWidget {background:#172331;color:#dce7f1;} QPushButton,QComboBox {background:#2b4053;border:1px solid #496075;border-radius:5px;padding:7px;} QPushButton:hover {background:#38536c;} QPushButton:disabled {color:#718294;} QListWidget {background:#111d2a;border:0;} QListWidget::item {padding:5px;} QListWidget::item:selected {background:#38536c;} QSlider::groove:horizontal {height:5px;background:#40576b;} QSlider::handle:horizontal {width:14px;margin:-5px 0;background:#61e6a2;border-radius:6px;}')
    window=Viewer(bags,args.at,args.play,args.self_test);window.show();return app.exec_()
if __name__=='__main__':sys.exit(main())
