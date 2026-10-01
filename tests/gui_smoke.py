"""Application integration test. Exercises owned widgets and checks the render bridge.

Run from project root: python tests/gui_smoke.py
Artifacts are written under outputs/qa; no global desktop input is injected.
"""
import json
import sys
import traceback
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from PySide6.QtCore import QTimer, QEvent
from PySide6.QtWidgets import QApplication
from cj_sim.gui import Window, configure_application


app=QApplication(sys.argv)
configure_application(app)
window=Window()
window.resize(1400,900)
window.show()
out=Path(__file__).resolve().parents[1]/'outputs'/'qa'
out.mkdir(parents=True,exist_ok=True)
checks=[]


def screenshot(name):
    if '--no-screenshots' not in sys.argv:
        window.grab().save(str(out/name))


def fail(error):
    print('GUI_SMOKE_FAILED:',error,flush=True)
    app.exit(1)


def rendered(value):
    try:
        assert value, 'WebGL scene did not render'
        checks.append('WebGL opaque scene loaded')
        screenshot('desktop.png')
        # Unequal lengths and masses must propagate through the editor to mechanics.
        window.leg_inputs[0,'proximal_length'].setValue(110)
        window.leg_inputs[0,'distal_length'].setValue(80)
        window.leg_inputs[1,'distal_mass'].setValue(95)
        window.leg_inputs[2,'proximal_arm'].setValue(75)
        assert window.dirty
        assert window.apply_design()
        assert not window.dirty
        assert window.mechanism.design.legs[0].proximal_length==.11
        assert window.mechanism.w3[1]>window.mechanism.w3[0]
        checks.append('parameter editing -> rebuild -> changed counterweight masses')
        window.sim.reset([.004,-.003,.005])
        np.testing.assert_allclose(window.sim.state()['residual'],0,atol=2e-7)
        window.scan()
        assert len(window.workspace_points)>0
        checks.append('custom-design workspace scan and balance')
        window.legacy_mode()
        assert window.sim.mode=='none' and not window.sim.gravity_enabled
        window.force_inputs[0].setValue(1)
        window.toggle_running()
        for _ in range(5):window.tick()
        window.stop()
        assert window.sim.p[0]>0
        assert len(window.sim.history)==5
        window.tabs.setCurrentWidget(window.plot_page)
        window.refresh_plots()
        assert window.tabs.tabText(window.tabs.currentIndex())=='仿真绘图'
        assert window.plot_rate.value()==30 and window.plot_timer.interval()==33
        assert window.plot_timer.isActive()
        assert window.trace.isVisible() and window.force_trace.isVisible()
        assert window.trace.samples is window.sim.history
        assert window.force_trace.samples is window.sim.history
        assert window.trace.columns==(1,2,3) and window.force_trace.columns==(10,11,12)
        np.testing.assert_allclose(np.array(window.force_trace.samples)[-1,10:13],window.sim.state()['tau'])
        window.plot_rate.setValue(20);assert window.plot_timer.interval()==50
        window.plot_rate.setValue(30)
        checks.append('dedicated displacement/drive-force plot tab, 30 Hz timer, correct CSV channels')
        window.sim.export_csv(out/'trajectory.csv')
        assert np.loadtxt(out/'trajectory.csv',delimiter=',',skiprows=1).shape==(5,24)
        checks.append('zero-gravity mode, force response, trace, CSV')
        window.full_mode()
        assert not window.trace.samples and not window.force_trace.samples
        window.force_inputs[0].setValue(0)
        window.release()
        initial=window.sim.p.copy()
        for _ in range(5):window.tick()
        window.stop()
        np.testing.assert_allclose(window.sim.p,initial,atol=1e-8)
        checks.append('full-balance offset release without artificial recentering')
        window.mechanism.design.save(out/'custom_design.json')
        report=window.mechanism.balance_report()
        (out/'balance_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        window.tabs.setCurrentIndex(2)
        window.refresh()
        QTimer.singleShot(300,begin_mouse)
    except Exception:
        fail(traceback.format_exc())


def begin_mouse():
    try:
        window.tabs.setCurrentIndex(1)
        window.legacy_mode()
        window.drag_plane.setCurrentText('XY')
        window.timer.stop()  # Deterministic physics steps after asynchronous bridge delivery.
        QTimer.singleShot(20,send_mouse)
    except Exception:fail(traceback.format_exc())


def send_mouse():
    if window._render_inflight is not None or window._pending_scene is not None:
        QTimer.singleShot(20,send_mouse)
        return
    try:
        window.view.page().runJavaScript('''(()=>{
            if(!window.interactionReady)return false;
            resetCamera();const p=project(scene.p),r=canvas.getBoundingClientRect();
            canvas.dispatchEvent(new PointerEvent('pointerdown',{clientX:r.left+p.x,clientY:r.top+p.y,button:0,pointerId:1}));
            canvas.dispatchEvent(new PointerEvent('pointermove',{clientX:r.left+p.x+8,clientY:r.top+p.y-6,pointerId:1}));
            return true;
        })()''',mouse_started)
    except Exception:fail(traceback.format_exc())


def mouse_started(value):
    if not value:fail('mouse bridge unavailable');return
    QTimer.singleShot(200,check_mouse)


def check_mouse():
    try:
        assert window.running and window.sim.mouse_target is not None
        assert abs(window.sim.mouse_target[2])<1e-12, 'XY drag must preserve target Z'
        assert np.linalg.norm(window.sim.mouse_force())>.01
        assert np.linalg.norm(window.sim.p)==0, 'mouse must not teleport platform'
        for _ in range(3):window.tick()
        assert np.linalg.norm(window.sim.p)>1e-5
        QTimer.singleShot(200,capture_mouse)
    except Exception:fail(traceback.format_exc())


def capture_mouse():
    window.view.page().runJavaScript('JSON.stringify({p:scene.p,force:scene.force,target:scene.mouse_target})',check_rendered_force)


def check_rendered_force(value):
    try:
        data=json.loads(value)
        np.testing.assert_allclose(data['p'],window.sim.p,atol=1e-12)
        np.testing.assert_allclose(data['force'],window.sim.state()['external'],atol=1e-12)
        assert data['target'] is not None
        screenshot('mouse_force.png')
        window.view.page().runJavaScript("canvas.dispatchEvent(new PointerEvent('pointerup',{pointerId:1}));")
        QTimer.singleShot(200,mouse_released)
    except Exception:fail(traceback.format_exc())


def mouse_released():
    try:
        assert window.sim.mouse_target is None
        assert window.running, 'release should retain natural motion'
        np.testing.assert_allclose(window.sim.mouse_force(),0)
        checks.append('viewport pointer events -> Python spring force -> motion -> release; XY plane')
        window.stop();window.full_mode();window.tabs.setCurrentIndex(2);window.refresh()
        QTimer.singleShot(300,finish)
    except Exception:fail(traceback.format_exc())


def finish():
    window.view.page().runJavaScript('JSON.stringify({error:renderer.gl.getError(),depth:renderer.gl.isEnabled(renderer.gl.DEPTH_TEST),instanced:Boolean(renderer.instancing),received:window.renderStats.received})',check_gpu)


def check_gpu(value):
    try:
        state=json.loads(value)
        assert state['error']==0, f'GPU error: {state}'
        assert state['depth'], 'opaque depth testing must remain enabled'
        checks.append('GPU draws without WebGL errors; depth testing retained')
        before=window.render_sent
        for _ in range(80):window.render()
        assert window.render_sent-before<=1,'unbounded render command queue'
        assert window._pending_scene is not None
        QTimer.singleShot(300,check_queue)
    except Exception:fail(traceback.format_exc())


def check_queue():
    try:
        assert window._pending_scene is None and window._render_inflight is None
        checks.append('80 update burst coalesced; frame acknowledgements drain bounded queue')
    except Exception:fail(traceback.format_exc());return
    screenshot('custom_design.png')
    window.view.page().runJavaScript('''(()=>{
        const p=project(scene.p),r=canvas.getBoundingClientRect();
        window.fastDragStart=[r.left+p.x,r.top+p.y];
        canvas.dispatchEvent(new PointerEvent('pointerdown',{clientX:fastDragStart[0],clientY:fastDragStart[1],button:0,pointerId:2}));
        canvas.dispatchEvent(new PointerEvent('pointermove',{clientX:Math.min(r.right-10,fastDragStart[0]+450),clientY:Math.max(r.top+10,fastDragStart[1]-200),pointerId:2}));
    })()''')
    QTimer.singleShot(150,check_fast_drag)


def check_fast_drag():
    try:
        assert window.running and window.sim.mouse_target is not None
        hits=0
        for _ in range(80):
            window.tick();hits+=window.sim.contact_active
            assert window.running,window.status.text()
            assert window.mechanism.reachable(window.sim.p)
        assert hits>0,'large drag should exercise end stops'
        window.edge_distance=float(np.linalg.norm(window.sim.p))
        window.view.page().runJavaScript("canvas.dispatchEvent(new PointerEvent('pointermove',{clientX:fastDragStart[0],clientY:fastDragStart[1],pointerId:2}));")
        QTimer.singleShot(150,check_reverse_drag)
    except Exception:fail(traceback.format_exc())


def check_reverse_drag():
    try:
        for _ in range(80):window.tick()
        assert window.running,window.status.text()
        assert np.linalg.norm(window.sim.p)<window.edge_distance*.9
        checks.append('450px fast drag: contacts do not pause; held pointer can reverse away from stop')
        window.view.page().runJavaScript("canvas.dispatchEvent(new PointerEvent('pointerup',{pointerId:2}));")
        QTimer.singleShot(150,finish_all)
    except Exception:fail(traceback.format_exc())


def finish_all():
    try:
        assert window.sim.mouse_target is None and window.running
        window.tick();assert window.running
    except Exception:fail(traceback.format_exc());return
    test_exit_case(0)


def test_exit_case(index):
    events=["canvas.dispatchEvent(new PointerEvent('pointermove',{clientX:r.right+500,clientY:r.top-500,pointerId:9}))",
            "canvas.dispatchEvent(new PointerEvent('pointerleave',{pointerId:9}))",
            "canvas.dispatchEvent(new PointerEvent('lostpointercapture',{pointerId:9}))",
            "window.dispatchEvent(new Event('blur'))",
            "window.dispatchEvent(new PointerEvent('pointerup',{pointerId:9}))",
            "canvas.dispatchEvent(new PointerEvent('pointercancel',{pointerId:9}))"]
    if index==len(events):
        complete_checks();return
    window.view.page().runJavaScript('''(()=>{
        const p=project(scene.p),r=canvas.getBoundingClientRect();
        canvas.dispatchEvent(new PointerEvent('pointerdown',{clientX:r.left+p.x,clientY:r.top+p.y,button:0,pointerId:9}));
        canvas.dispatchEvent(new PointerEvent('pointermove',{clientX:r.left+p.x+5,clientY:r.top+p.y,pointerId:9}));
    })()''')
    def exit_drag():
        try:assert window.sim.mouse_target is not None
        except Exception:fail(traceback.format_exc());return
        window.view.page().runJavaScript('(()=>{const r=canvas.getBoundingClientRect();'+events[index]+';return drag===null&&pendingMouseTarget===null;})()',lambda cleared:QTimer.singleShot(100,lambda:verify_exit(cleared)))
    def verify_exit(cleared):
        try:
            if index in (0,1,3):
                assert not cleared and window.sim.mouse_target is not None,'leaving viewport must retain drag'
            else:
                assert cleared and window.sim.mouse_target is None
            before=window.sim.time
            for _ in range(80 if index<2 else 10):
                window.tick()
                assert window.running,window.status.text()
            assert window.running and window.sim.time>before,window.status.text()
            if index in (0,1,3):
                assert np.linalg.norm(window.sim.mouse_force())<=12+1e-10
                window.view.page().runJavaScript("canvas.dispatchEvent(new PointerEvent('pointermove',{clientX:canvas.getBoundingClientRect().left+canvas.clientWidth/2,clientY:canvas.getBoundingClientRect().top+canvas.clientHeight/2,pointerId:9}));")
                QTimer.singleShot(100,check_return)
                return
            np.testing.assert_allclose(window.sim.mouse_force(),0)
            QTimer.singleShot(100,lambda:test_exit_case(index+1))
        except Exception:fail(traceback.format_exc())
    def check_return():
        try:
            assert window.sim.mouse_target is not None and window.running
            for _ in range(20):window.tick()
            assert window.running,window.status.text()
            window.view.page().runJavaScript("window.dispatchEvent(new PointerEvent('pointerup',{pointerId:9}));")
            QTimer.singleShot(100,lambda:test_exit_case(index+1))
        except Exception:fail(traceback.format_exc())
    QTimer.singleShot(100,exit_drag)


def complete_checks():
    try:
        window.interaction.beginDrag(.01,0,0)
        assert window.sim.mouse_target is not None
        QApplication.sendEvent(window,QEvent(QEvent.Type.WindowDeactivate))
        assert window.sim.mouse_target is None and window.running
    except Exception:fail(traceback.format_exc());return
    checks.append('outside viewport, return and internal view blur retain drag; real window deactivation, lost capture, release and cancellation clear only mouse force')
    (out/'integration.json').write_text(json.dumps({'passed':True,'checks':checks},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'passed':True,'checks':checks}),flush=True)
    app.exit(0)


def probe():
    window.view.page().runJavaScript('resetCamera(); Boolean(window.sceneRendered)',rendered)


QTimer.singleShot(3500,probe)
QTimer.singleShot(30000,lambda:fail('timeout'))
sys.exit(app.exec())
