"""Native Qt editor and controls; Python owns every physical state update."""
import json
import sys
import time
from dataclasses import asdict, fields
from pathlib import Path
import numpy as np
from PySide6.QtCore import Qt, QTimer, QUrl, QObject, Slot, QPointF
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFormLayout, QLabel, QPushButton, QDoubleSpinBox, QComboBox, QCheckBox, QTabWidget,
    QTableWidget, QTableWidgetItem, QHeaderView, QScrollArea, QSplitter, QFileDialog, QMessageBox)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebChannel import QWebChannel
from .core import Design, Leg, Mechanism, Simulation, ISO, DEFAULT_STIFFNESS, DEFAULT_DAMPING

ROOT = Path(__file__).resolve().parent.parent
COLORS = ['#51d4cc', '#ffba62', '#af98ff']


class InteractionBridge(QObject):
    """Viewport events carry targets only; all forces are computed in Python."""
    def __init__(self, window):
        super().__init__(window)
        self.window=window

    @Slot(float,float,float)
    def beginDrag(self,x,y,z):
        w=self.window
        if w.dirty:
            w.status.setText('请先应用参数，再拖动末端施力。')
            return
        w.sim.set_mouse_target([x,y,z])
        w.running=True; w.play.setText('Ⅱ 暂停仿真')
        w.status.setText('鼠标施力中：220 N/m 虚拟弹簧，上限 12 N；松开撤去鼠标外力。')

    @Slot(float,float,float)
    def moveDrag(self,x,y,z):
        if self.window.sim.mouse_target is not None:
            self.window.sim.set_mouse_target([x,y,z])

    @Slot()
    def endDrag(self):
        w=self.window
        if w.sim.mouse_target is None:return
        w.sim.set_mouse_target(None)
        w.status.setText('鼠标外力已撤去；运动由惯性、重力、配重及当前 K/B 设置决定。')
        if not w.running:w.refresh()

    @Slot(str)
    def notify(self,text):
        self.window.status.setText(text)

    @Slot(int,float)
    def frameDone(self,sequence,cost):
        self.window.frame_done(sequence,cost)


def spin(value, low, high, decimals=3, suffix=''):
    w = QDoubleSpinBox()
    w.setRange(low, high); w.setDecimals(decimals); w.setValue(value); w.setSuffix(suffix)
    w.setKeyboardTracking(False)
    return w


class Trace(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.samples = []
        self.setMinimumHeight(135)
        self.setMaximumHeight(190)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#101e30'))
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QColor('#b8cde3'))
        painter.drawText(14, 22, '位移随仿真时间变化  /  X 青 · Y 橙 · Z 紫')
        if len(self.samples) < 2:
            painter.drawText(14, 65, '开始仿真后显示曲线；暂停时可导出 CSV。')
            return
        data = np.array(self.samples[-400:])
        limit = max(.005, float(np.max(np.abs(data[:, 1:4]))))*1000
        left, top, width, height = 58, 34, self.width()-80, self.height()-55
        painter.setPen(QColor('#30465e'))
        painter.drawLine(left, top+height//2, left+width, top+height//2)
        painter.drawText(4, top+10, f'{limit:.1f}')
        painter.drawText(4, top+height, f'-{limit:.1f}')
        painter.drawText(left+width-120, self.height()-5, f'{data[-1,0]:.2f} s   /   mm')
        for axis, color in enumerate(COLORS):
            painter.setPen(QPen(QColor(color), 1.8))
            points=QPolygonF([QPointF(left+width*i/(len(data)-1),top+height/2-row[axis+1]*1000/limit*height/2) for i,row in enumerate(data)])
            painter.drawPolyline(points)


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Constant Jacobian · 参数化机构仿真平台')
        self.resize(1440, 930)
        self.mechanism = Mechanism()
        self.sim = Simulation(self.mechanism)
        self.running = False; self.web_ready = False; self.dirty = False
        self.workspace_points = []
        self._workspace_revision=0;self._sent_workspace=None
        self._design_key=json.dumps(asdict(self.mechanism.design))
        self._render_sequence=0;self._render_inflight=None;self._pending_scene=None
        self.render_sent=0;self.render_coalesced=0;self.render_completed=0;self.last_draw_ms=0.
        self._last_readout=0.
        self.timer = QTimer(self); self.timer.setTimerType(Qt.TimerType.PreciseTimer);self.timer.setInterval(16); self.timer.timeout.connect(self.tick)
        container = QWidget(); outer = QVBoxLayout(container); self.setCentralWidget(container)
        title = QLabel('CONSTANT JACOBIAN   /   参数化设计实验室')
        title.setStyleSheet('font-size:22px;font-weight:600;padding:8px;color:#e7f3ff')
        outer.addWidget(title)
        tools = QHBoxLayout()
        for text, callback in [('打开设计', self.open_design), ('保存设计', self.save_design), ('导出配重报告', self.export_report), ('导出仿真 CSV', self.export_csv)]:
            b = QPushButton(text); b.clicked.connect(callback); tools.addWidget(b)
        tools.addStretch(); outer.addLayout(tools)
        split = QSplitter(); outer.addWidget(split, 1)
        self.tabs = QTabWidget(); self.tabs.setMinimumWidth(405); split.addWidget(self.tabs)
        self.build_design_tab(); self.build_sim_tab(); self.build_result_tab()
        right = QWidget(); layout = QVBoxLayout(right); layout.setContentsMargins(0,0,0,0)
        self.view = QWebEngineView(); self.view.setMinimumSize(380, 320)
        self.channel=QWebChannel(self.view.page())
        self.interaction=InteractionBridge(self)
        self.channel.registerObject('interaction',self.interaction)
        self.view.page().setWebChannel(self.channel)
        self.view.loadFinished.connect(self.loaded)
        self.view.setUrl(QUrl.fromLocalFile(str(ROOT/'assets'/'scene.html')))
        layout.addWidget(self.view, 1)
        opts = QHBoxLayout()
        self.axes = QCheckBox('白色虚线关节轴'); self.axes.setChecked(True)
        self.labels = QCheckBox('零件标注'); self.labels.setChecked(True)
        self.workspace = QCheckBox('采样工作空间')
        for check in [self.axes, self.labels, self.workspace]:
            opts.addWidget(check); check.toggled.connect(self.render)
        home = QPushButton('复位视角'); home.clicked.connect(lambda: self.view.page().runJavaScript('resetCamera()'))
        opts.addWidget(home); layout.addLayout(opts)
        self.readout = QLabel(); self.readout.setWordWrap(True); self.readout.setMinimumHeight(65)
        layout.addWidget(self.readout)
        self.trace = Trace(); layout.addWidget(self.trace)
        split.addWidget(right); split.setSizes([445, 950])
        self.status = QLabel('修改参数后点击“应用参数并计算”；默认启用人为回中。')
        self.status.setWordWrap(True); outer.addWidget(self.status)
        self.populate(Design()); self.dirty=False; self.update_results(); self.refresh()
        self.status.setText(f'就绪。默认完整配重、重力开启；人为回中 K={DEFAULT_STIFFNESS:g} N/m，B={DEFAULT_DAMPING:g} N·s/m。')
        self.timer.start()

    def scroll_tab(self, title):
        pane = QWidget(); layout = QVBoxLayout(pane)
        area = QScrollArea(); area.setWidgetResizable(True); area.setWidget(pane)
        self.tabs.addTab(area, title)
        return layout

    def button(self, layout, text, callback):
        b = QPushButton(text); b.clicked.connect(callback); layout.addWidget(b); return b

    def build_design_tab(self):
        layout = self.scroll_tab('参数建模')
        hint = QLabel('界面单位：mm / g；内核与项目文件使用 m / kg。\n支链 1 青色 · 支链 2 橙色 · 支链 3 紫色')
        hint.setWordWrap(True); layout.addWidget(hint)
        form = QFormLayout(); layout.addLayout(form)
        self.global_inputs = {}
        specs = [('alpha','结构倾角 α', ISO,2,85,1,' °'), ('platform_mass','平台质量',350,1,100000,.001,' g'),
                 ('platform_radius','平台连接半径',68,1,2000,.001,' mm'),
                 ('rail_center_distance','导轨轴线到零位中心距离',Design().rail_center_distance*1000,1,5000,.001,' mm'),
                 ('rail_total_travel','导轨总行程（对称 ± 一半）',55,.2,2000,.001,' mm'), ('gravity','重力加速度',9.81,0,30,1,' m/s²')]
        for name, label, value, low, high, factor, suffix in specs:
            w = spin(value, low, high, 6 if name=='alpha' else 3, suffix)
            self.global_inputs[name] = (w, factor); form.addRow(label, w); w.valueChanged.connect(self.mark_dirty)
        self.button(layout, '恢复正交倾角 35.2644°', lambda: self.global_inputs['alpha'][0].setValue(ISO))
        self.leg_table = QTableWidget(7, 3)
        self.leg_table.setHorizontalHeaderLabels(['支链 1','支链 2','支链 3'])
        self.leg_table.setVerticalHeaderLabels(['近端杆长 L2 / mm','远端杆长 L3 / mm','近端杆质量 / g','远端杆质量 / g','滑块质量 / g','W2 反向臂长 / mm','W3 反向臂长 / mm'])
        self.leg_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.leg_table.setMinimumHeight(280)
        self.leg_inputs = {}
        for row, f in enumerate(fields(Leg)):
            for col in range(3):
                w = spin(getattr(Leg(), f.name)*1000, .1, 100000 if 'mass' in f.name else 3000)
                w.valueChanged.connect(self.mark_dirty); self.leg_table.setCellWidget(row, col, w)
                self.leg_inputs[col, f.name] = w
        layout.addWidget(self.leg_table)
        self.button(layout, '将支链 1 参数复制到支链 2、3', self.copy_leg)
        self.apply_button = self.button(layout, '应用参数并计算配重', self.apply_design)
        self.apply_button.setStyleSheet('background:#126f70;font-weight:bold;padding:12px')
        self.button(layout, '恢复默认参数', lambda: self.populate(Design()))
        note = QLabel('导轨距离是轴线到末端平台零位中心的垂直距离，三条导轨共用此值。\n总行程是滑块允许的运动范围，不含滑块长度及端部安装余量；实际可达范围还受杆长限制。\n配重位置沿各杆反向延长线定义；杆为均质细杆，配重臂无质量。\n改变杆长不会改变 J，但会改变可装配范围、惯性及所需配重。')
        note.setWordWrap(True); layout.addWidget(note); layout.addStretch()

    def build_sim_tab(self):
        layout = self.scroll_tab('仿真控制')
        self.play = self.button(layout, '▶ 开始仿真', self.toggle_running)
        self.button(layout, '重置位置与速度', self.reset)
        self.gravity = QCheckBox('启用重力'); self.gravity.setChecked(True); layout.addWidget(self.gravity)
        form = QFormLayout(); layout.addLayout(form)
        self.mode = QComboBox(); self.mode.addItems(['完整配重（W2 + W3 + H）','仅杆端配重（W2 + W3）','无配重'])
        form.addRow('配重模式', self.mode)
        self.drag_plane=QComboBox(); self.drag_plane.addItems(['随视角','XY','XZ','YZ'])
        form.addRow('鼠标施力平面',self.drag_plane)
        self.drag_plane.currentIndexChanged.connect(self.render)
        self.ratio = spin(100,0,200,1,' %'); form.addRow('全部配重质量比例',self.ratio)
        self.k = spin(DEFAULT_STIFFNESS,0,2000,1,' N/m'); form.addRow('人为回中刚度 K',self.k)
        self.b = spin(DEFAULT_DAMPING,0,100,2,' N·s/m'); form.addRow('阻尼 B',self.b)
        self.position_inputs=[]; self.force_inputs=[]
        for i, name in enumerate('XYZ'):
            w=spin(0,-2000,2000,2,' mm'); self.position_inputs.append(w); form.addRow(f'设置位移 {name}',w)
        self.button(layout,'移动到指定位置（暂停）',self.set_position)
        forces=QFormLayout(); layout.addLayout(forces)
        for name in 'XYZ':
            w=spin(0,-20,20,2,' N'); self.force_inputs.append(w); forces.addRow(f'外力 {name}',w); w.valueChanged.connect(self.change_controls)
        for w in [self.ratio,self.k,self.b]: w.valueChanged.connect(self.change_controls)
        self.gravity.toggled.connect(self.change_controls); self.mode.currentIndexChanged.connect(self.change_controls)
        self.button(layout,'无重力、无配重模式',self.legacy_mode)
        self.button(layout,'完整配重 + 重力模式',self.full_mode)
        self.button(layout,'偏置释放实验（K=0、外力=0）',self.release)
        note=QLabel('拖动白色末端球施力（220 N/m，上限 12 N）；空白处旋转。开始拖动会启动仿真，松开仅撤去鼠标外力。\nK=0 时无自动回中；完整配重使静止机构在可达位置保持平衡。\n到达边界后限位，反向拖动可离开；绳索松弛时配重自由运动，重新拉紧后恢复传力，不自动暂停。\n仿真按固定步长推进，时间以图中仿真时钟为准。')
        note.setWordWrap(True); layout.addWidget(note); layout.addStretch()

    def build_result_tab(self):
        layout=self.scroll_tab('计算结果')
        self.result_label=QLabel(); self.result_label.setWordWrap(True); self.result_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.result_label)
        self.mass_table=QTableWidget(3,4); self.mass_table.setHorizontalHeaderLabels(['支链','W2 / g','W3 / g','H / g'])
        self.mass_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.mass_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers); self.mass_table.setMaximumHeight(160)
        layout.addWidget(self.mass_table)
        self.matrix_label=QLabel(); self.matrix_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.matrix_label.setStyleSheet('font-family:Consolas;font-size:13px'); layout.addWidget(self.matrix_label)
        self.button(layout,'扫描工作空间与平衡残差',self.scan)
        self.scan_label=QLabel('尚未扫描。'); self.scan_label.setWordWrap(True); layout.addWidget(self.scan_label)
        formula=QLabel('W3 = m3·L3 / (2·r3)\nW2 = [m2·L2/2 + (m3+W3)·L2] / r2\nHᵢ = |m平台·J₃ᵢ + M支链ᵢ·Gᵢ₃|\n\nq = Gp；p = Jq；电机力 τ = JᵀF电机\n配重补偿力单独计入重力项。\n\n表中为 100% 理想静平衡质量。比例开关会改变全部九个实际配重的质量。')
        formula.setWordWrap(True); layout.addWidget(formula); layout.addStretch()

    def mark_dirty(self, *args):
        self.dirty=True
        if hasattr(self,'status'): self.status.setText('参数已修改，尚未应用；点击“应用参数并计算配重”更新模型。')

    def populate(self, design):
        for name,(w,factor) in self.global_inputs.items(): w.setValue(getattr(design,name)/factor)
        for (i,name),w in self.leg_inputs.items(): w.setValue(getattr(design.legs[i],name)*1000)
        self.mark_dirty()

    def read_design(self):
        params={name:w.value()*factor for name,(w,factor) in self.global_inputs.items()}
        rail_distance=params.pop('rail_center_distance')
        rail_stroke=params.pop('rail_total_travel')
        params['legs']=[Leg(**{f.name:self.leg_inputs[i,f.name].value()/1000 for f in fields(Leg)}) for i in range(3)]
        design=Design(**params)
        design.rail_center_distance=rail_distance
        design.rail_total_travel=rail_stroke
        return design

    def copy_leg(self):
        for f in fields(Leg):
            for i in (1,2): self.leg_inputs[i,f.name].setValue(self.leg_inputs[0,f.name].value())

    def stop(self):
        self.running=False; self.sim.set_mouse_target(None); self.play.setText('▶ 开始仿真')
        if hasattr(self,'view') and self.web_ready:
            self.view.page().runJavaScript('cancelPointerDrag()')
            self.refresh()

    def apply_design(self):
        try:
            mechanism=Mechanism(self.read_design())
            self.stop(); self.mechanism=mechanism; self.sim=Simulation(mechanism)
            self.workspace_points=[];self._workspace_revision+=1;self._design_key=json.dumps(asdict(mechanism.design))
            self.dirty=False; self.change_controls(); self.update_results()
            self.scan_label.setText('参数已更新，请重新扫描。')
            self.status.setText('参数已应用，九个配重质量已重新计算。'); self.refresh()
            return True
        except (ValueError,TypeError) as e:
            self.message(str(e)); return False

    def change_controls(self,*args):
        self.sim.mode=['full','rods','none'][self.mode.currentIndex()]
        self.sim.gravity_enabled=self.gravity.isChecked(); self.sim.ratio=self.ratio.value()/100
        self.sim.stiffness=self.k.value(); self.sim.damping=self.b.value()
        self.sim.force=np.array([w.value() for w in self.force_inputs])
        if not self.running: self.refresh()

    def toggle_running(self):
        if self.dirty:
            self.message('请先应用修改后的参数。'); return
        if self.running:self.stop()
        else:self.running=True; self.play.setText('Ⅱ 暂停仿真')

    def reset(self):
        self.stop(); self.sim.reset(); self.refresh(); self.status.setText('已重置。')

    def set_position(self):
        try:
            self.stop(); self.sim.reset(np.array([w.value() for w in self.position_inputs])/1000); self.refresh()
        except ValueError as e: self.message(str(e))

    def legacy_mode(self):
        self.stop(); self.gravity.setChecked(False); self.mode.setCurrentIndex(2); self.reset()

    def full_mode(self):
        self.stop(); self.gravity.setChecked(True); self.mode.setCurrentIndex(0); self.ratio.setValue(100); self.reset()

    def release(self):
        self.stop(); self.k.setValue(0)
        for w in self.force_inputs: w.setValue(0)
        direction=np.array([.009,-.008,.012])
        while not self.mechanism.reachable(direction): direction*=.5
        self.sim.reset(direction); self.running=True; self.play.setText('Ⅱ 暂停仿真'); self.refresh()

    def tick(self):
        if not self.running: return
        try:
            for _ in range(4): self.sim.step()
            self.refresh(record=True)
        except (ValueError,np.linalg.LinAlgError) as e:
            self.stop(); self.status.setText(str(e)); self.refresh()

    def refresh(self,record=False):
        try:
            state=self.sim.state()
            if record: self.sim.record(state)
            self.render(state=state)
            now=time.perf_counter()
            if record and now-self._last_readout<.1:return
            self._last_readout=now
            fmt=lambda a:' / '.join(f'{x:.2f}' for x in a)
            self.readout.setText(f't = {self.sim.time:.3f} s    ·    ΔXYZ = {fmt(self.sim.p*1000)} mm\n'
                f'q = {fmt(self.mechanism.G@self.sim.p*1000)} mm    ·    τ = {fmt(state["tau"])} N\n'
                f'鼠标力 = {fmt(state["mouse_force"])} N    ·    名义静平衡残差 {np.linalg.norm(state["residual"]):.3e} N\n'
                f'绳张力 = {fmt(state["tension"])} N    ·    '
                f'{"边界限位 · " if self.sim.contact_active else ""}'
                f'{"松绳 H"+",".join(str(i+1) for i in np.flatnonzero(state["rope_slack"])) if np.any(state["rope_slack"]) else "绳索无松弛"}\n'
                f'动能 {state["kinetic"]:.5f} J    ·    总质量 {state["total_mass"]:.3f} kg')
            self.trace.samples=self.sim.history; self.trace.update()
        except ValueError as e: self.status.setText(str(e))

    def loaded(self,ok):
        self.web_ready=ok
        self._render_inflight=None;self._pending_scene=None;self._sent_workspace=None
        if ok: self.render()
        else: self.status.setText('三维视图加载失败，请检查 assets 文件夹。')

    def render(self,*args,state=None):
        if not self.web_ready:return
        if state is None: state=self.sim.state()
        scene={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in state['scene'].items() if k not in ('positions','masses','is_weight')}
        scene.update(p=self.sim.p.tolist(),force=state['external'].tolist(),motor=state['motor'].tolist(),
                     axes=self.axes.isChecked(),labels=self.labels.isChecked(),travel=self.mechanism.design.travel,
                     design_key=self._design_key,
                     drag_plane=self.drag_plane.currentText(),mouse_target=None if self.sim.mouse_target is None else self.sim.mouse_target.tolist(),
                     can_drag=not self.dirty)
        if self._pending_scene is not None:self.render_coalesced+=1
        self._pending_scene=scene
        self.flush_render()

    def flush_render(self):
        # One outstanding frame + one replaceable latest state. Never queue a
        # history of draw requests when Chromium/GPU is temporarily busy.
        if self._render_inflight is not None or self._pending_scene is None or not self.web_ready:return
        scene=self._pending_scene;self._pending_scene=None
        self._render_sequence+=1;scene['sequence']=self._render_sequence
        workspace_key=(self._workspace_revision,self.workspace.isChecked())
        if self._sent_workspace!=workspace_key:
            scene['workspace']=self.workspace_points if self.workspace.isChecked() else []
            self._sent_workspace=workspace_key
        self._render_inflight=self._render_sequence;self.render_sent+=1
        self.view.page().runJavaScript('updateScene('+json.dumps(scene,allow_nan=False,separators=(',',':'))+')')

    def frame_done(self,sequence,cost):
        if sequence!=self._render_inflight:return
        self._render_inflight=None;self.render_completed+=1;self.last_draw_ms=cost
        self.flush_render()

    def update_results(self):
        m=self.mechanism
        self.result_label.setText(f'J 条件数：{m.condition:.6f}\nα = {m.design.alpha:.6f}°；J 与末端位置无关。\n悬挂绳路方向 s = {m.sign.astype(int).tolist()}')
        for i in range(3):
            for j,value in enumerate([str(i+1),f'{m.w2[i]*1000:.3f}',f'{m.w3[i]*1000:.3f}',f'{m.hanging[i]*1000:.3f}']):
                self.mass_table.setItem(i,j,QTableWidgetItem(value))
        self.matrix_label.setText('J =\n'+np.array2string(m.J,precision=5,suppress_small=True)+'\n\nG =\n'+np.array2string(m.G,precision=5,suppress_small=True))

    def scan(self):
        self.stop()
        points=self.mechanism.workspace(13); self.workspace_points=points.tolist();self._workspace_revision+=1
        report=self.mechanism.balance_report(5)
        bounds=np.ptp(points,axis=0)*1000 if len(points) else np.zeros(3)
        self.scan_label.setText(f'滑块空间 13³ 个采样中，可达 {len(points)} 个。\nXYZ 采样跨度：{bounds.round(2).tolist()} mm\n'
                               f'100% 完整配重最大残差：{report["max_balance_residual_N"]:.3e} N\n采样不考虑杆件碰撞，不等于精确工作空间边界。')
        self.workspace.setChecked(True); self.render()

    def message(self,text):
        QMessageBox.warning(self,'参数提示',text)

    def open_design(self):
        path,_=QFileDialog.getOpenFileName(self,'打开设计',str(ROOT),'JSON (*.json)')
        if not path:return
        try:
            design=Design.load(path); Mechanism(design); self.populate(design); self.apply_design()
        except (ValueError,KeyError,TypeError,OSError) as e:self.message(str(e))

    def save_design(self):
        if self.dirty and not self.apply_design():return
        path,_=QFileDialog.getSaveFileName(self,'保存设计',str(ROOT/'design.json'),'JSON (*.json)')
        if path:
            try:self.mechanism.design.save(path); self.status.setText('设计已保存：'+path)
            except OSError as e:self.message(str(e))

    def export_report(self):
        if self.dirty and not self.apply_design():return
        path,_=QFileDialog.getSaveFileName(self,'导出配重报告',str(ROOT/'balance_report.json'),'JSON (*.json)')
        if path:
            try:
                report=self.mechanism.balance_report()
                report['assumptions']='SI units; uniform rods; massless extensions; nominal 100% static balance with taut ropes; dynamic simulation uses unilateral ropes and end stops, without general collision detection'
                Path(path).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8'); self.status.setText('报告已导出：'+path)
            except (OSError,ValueError) as e:self.message(str(e))

    def export_csv(self):
        self.stop()
        if not self.sim.history:self.message('请先运行仿真，生成轨迹后导出。');return
        path,_=QFileDialog.getSaveFileName(self,'导出轨迹',str(ROOT/'trajectory.csv'),'CSV (*.csv)')
        if path:
            try:self.sim.export_csv(path); self.status.setText('轨迹已导出：'+path)
            except OSError as e:self.message(str(e))


def configure_application(app):
    app.setStyle('Fusion')
    app.setStyleSheet('''QWidget{background:#101b2b;color:#dbe9f7;font-family:"Microsoft YaHei UI";font-size:12px;}
    QLineEdit,QDoubleSpinBox,QComboBox{background:#192c42;border:1px solid #38516b;border-radius:4px;padding:5px;}
    QPushButton{background:#233c54;border:1px solid #42627e;border-radius:5px;padding:8px;}
    QPushButton:hover{background:#315b70;} QTabBar::tab{padding:10px;background:#182b40;}
    QTabBar::tab:selected{background:#246169;} QHeaderView::section{background:#1d354c;color:#dbe9f7;padding:6px;}
    QTableWidget{gridline-color:#32465c;} QScrollArea{border:0;} QLabel{padding:3px;}
    QCheckBox{spacing:8px;padding:5px;} QSplitter::handle{background:#2a4057;}''')
    

def main():
    app=QApplication(sys.argv)
    configure_application(app)
    window=Window(); window.show()
    sys.exit(app.exec())
