"""End-to-end timing under live mouse-spring motion with labels, axes and workspace.

python tests/performance_check.py --output outputs/performance.json
Runs an isolated application instance and never injects OS input.
"""
import argparse
import json
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from cj_sim.gui import Window, configure_application
from cj_sim.core import Mechanism

parser=argparse.ArgumentParser()
parser.add_argument('--output',default='outputs/performance.json')
parser.add_argument('--seconds',type=float,default=6.)
parser.add_argument('--fast-drag',action='store_true',help='Alternate large mouse targets to stress contacts/slack ropes')
args=parser.parse_args()


class BenchWindow(Window):
    def __init__(self):
        self.tick_cost=[]; self.tick_gaps=[]; self.last_tick=None
        super().__init__()

    def tick(self):
        start=time.perf_counter()
        if self.running:
            if self.last_tick is not None:self.tick_gaps.append((start-self.last_tick)*1000)
            self.last_tick=start
            if args.fast_drag:
                self.sim.set_mouse_target(np.array([.25 if int(self.sim.time*2)%2==0 else -.25,.18,-.12]))
            else:
                self.sim.set_mouse_target(np.array([.0015*np.sin(self.sim.time*2),.001*np.cos(self.sim.time*2),.001]))
            super().tick()
            self.tick_cost.append((time.perf_counter()-start)*1000)


app=QApplication(sys.argv);configure_application(app)
window=BenchWindow();window.show()
result={}


def summary(values):
    return dict(samples=len(values),median_ms=float(np.median(values)),p95_ms=float(np.percentile(values,95)),max_ms=float(max(values))) if values else None


def begin():
    m=Mechanism();cost=[]
    for _ in range(300):
        t=time.perf_counter();m.properties([.004,-.003,.006],[.005,-.003,.004]);cost.append((time.perf_counter()-t)*1000)
    result['properties']=summary(cost)
    window.scan()
    window.view.page().runJavaScript('''(()=>{
      window.perfDraw=[];window.perfGaps=[];window.perfTracking=true;
      const previous=draw;draw=function(){const t=performance.now();previous();if(window.perfTracking)window.perfDraw.push(performance.now()-t);};
      let last=0;function watch(t){if(!window.perfTracking)return;if(last)window.perfGaps.push(t-last);last=t;requestAnimationFrame(watch);}requestAnimationFrame(watch);
      return Boolean(window.sceneRendered);
    })()''',start)


def start(ok):
    if not ok:print('Renderer did not load',flush=True);app.exit(1);return
    result['wall_start']=time.perf_counter()
    window.running=True
    QTimer.singleShot(int(args.seconds*1000),finish)


def finish():
    result['wall_seconds']=time.perf_counter()-result.pop('wall_start')
    result['sim_seconds']=window.sim.time
    result['still_running']=window.running
    window.running=False
    result['tick']=summary(window.tick_cost);result['tick_gap']=summary(window.tick_gaps)
    window.view.page().runJavaScript('window.perfTracking=false;JSON.stringify({draw:window.perfDraw,gaps:window.perfGaps,stats:window.renderStats||null})',save)


def save(value):
    data=json.loads(value)
    result['draw']=summary(data['draw']);result['raf_gap']=summary(data['gaps']);result['render_stats']=data['stats']
    result['render_fps']=len(data['draw'])/result['wall_seconds']
    result['status']=window.status.text()
    result['frame_transport']={'sent':window.render_sent,'completed':window.render_completed,'coalesced':window.render_coalesced,'inflight':window._render_inflight is not None,'pending':window._pending_scene is not None}
    target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=True),flush=True);app.exit(0)


QTimer.singleShot(3000,begin)
QTimer.singleShot(45000,lambda:app.exit(2))
sys.exit(app.exec())
