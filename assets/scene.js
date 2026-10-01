'use strict';
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');
let renderer=null,scene=null,yaw=.68,pitch=.44,zoom=1,drag=null,currentBasis=null,scale=1000;
let bridge=null;
let drawPending=false,pendingMouseTarget=null;
window.renderStats={frames:0,lastDrawMs:0,received:0,coalesced:0};
function scheduleDraw(){if(drawPending){window.renderStats.coalesced++;return;}drawPending=true;requestAnimationFrame(()=>{drawPending=false;if(pendingMouseTarget&&drag?.kind==='force'&&bridge){bridge.moveDrag(...pendingMouseTarget);pendingMouseTarget=null;}draw();});}
new QWebChannel(qt.webChannelTransport,channel=>{bridge=channel.objects.interaction;window.interactionReady=true;scheduleDraw();});
const colors=['#51d4cc','#ffba62','#af98ff'];
try{renderer=new SolidRenderer(document.getElementById('solids'));}catch(e){document.getElementById('error').textContent='三维渲染不可用：'+e.message+'。计算与导出仍可使用。';}
function basis(){const forward=[Math.cos(pitch)*Math.cos(yaw),Math.cos(pitch)*Math.sin(yaw),Math.sin(pitch)],right=unit(cross([0,0,1],forward)),up=cross(forward,right);return{forward,right,up};}
function project(p){const v=sub(p,renderer.center);return{x:canvas.clientWidth/2+dot(v,currentBasis.right)*scale,y:canvas.clientHeight*.49-dot(v,currentBasis.up)*scale,z:dot(p,currentBasis.forward)};}
function block(p,size,color){renderer.box(p,size,color);}
function dashed(p,n,length){renderer.line(sub(p,mul(n,length/2)),add(p,mul(n,length/2)),'#e6eef8',1.2,[6,5]);}
function draw(){
 if(!scene||!renderer)return;
 const started=performance.now();
 const width=canvas.clientWidth,height=canvas.clientHeight,dpr=Math.min(devicePixelRatio||1,2);
 if(canvas.width!==Math.round(width*dpr)||canvas.height!==Math.round(height*dpr)){canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);}
 ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,width,height);
 currentBasis=basis();
 const points=[...scene.a,...scene.b,...scene.c,...scene.c0,...scene.top,...scene.pulley,...scene.hanging_pos,...scene.w2pos,...scene.w3pos];
 for(let i=0;i<3;i++)points.push(sub(scene.c0[i],mul(scene.n[i],scene.travel)),add(scene.c0[i],mul(scene.n[i],scene.travel)));
 // Fixed framing derives from the design, not current platform motion.
 if(!renderer.center||renderer.needsFraming){const lows=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),highs=[0,1,2].map(i=>Math.max(...points.map(p=>p[i])));renderer.center=mul(add(lows,highs),.5);renderer.extent=norm(sub(highs,lows));renderer.ground=lows[2]-renderer.extent*.13;renderer.needsFraming=false;}
 scale=Math.min(width,height)*.86/renderer.extent*zoom;
 renderer.depthRange=Math.max(1,renderer.extent*4);
 renderer.labelsEnabled=scene.labels;
 renderer.begin(width,height,dpr,currentBasis,scale);
 const size=renderer.extent*.007;
 const ground=renderer.ground;
 const extent=renderer.extent*.55;
 renderer.polygon([[-extent,-extent,ground],[extent,-extent,ground],[extent,extent,ground],[-extent,extent,ground]],'#14273a');
 for(let i=-5;i<=5;i++){let v=extent*i/5;renderer.line([-extent,v,ground+.0001],[extent,v,ground+.0001],'#28415b',.65);renderer.line([v,-extent,ground+.0001],[v,extent,ground+.0001],'#28415b',.65);}
 renderer.points(scene.workspace||EMPTY_POINTS);
 renderer.prism(scene.c0,size,'#344c66');renderer.prism(scene.a,size,'#809bb3');
 const labels=[];
 for(let i=0;i<3;i++){
   const a=scene.a[i],b=scene.b[i],c=scene.c[i],n=scene.n[i],c0=scene.c0[i],color=colors[i];
   renderer.cylinder(sub(c0,mul(n,scene.travel)),add(c0,mul(n,scene.travel)),size*.55,'#788ba0');
   renderer.cylinder(c,b,size*.72,color);renderer.cylinder(b,a,size*.72,color);
   block(c,size*2.1,color);
   for(const [j,p] of [c,b,a].entries()){
     renderer.cylinder(sub(p,mul(n,size*1.5)),add(p,mul(n,size*1.5)),size*.92,'#d4deeb');
     if(scene.axes)dashed(p,n,size*11);
     labels.push([p,`R${i+1}${j+1}`,color]);
   }
   if(scene.axes)dashed(c0,n,scene.travel*2+size*9);
   for(const [pos,mass,origin,name] of [[scene.w2pos[i],scene.w2[i],c,'W2'],[scene.w3pos[i],scene.w3[i],b,'W3']]){
     if(mass>0){renderer.cylinder(origin,pos,size*.38,'#ba974c');block(pos,size*2.7,'#edc569');labels.push([pos,`${name}-${i+1} ${(mass*1000).toFixed(1)}g`,'#edc569']);}
   }
   if(scene.hanging[i]>0){
     const pulley=scene.pulley[i],top=scene.top[i],h=scene.hanging_pos[i];
     const slack=scene.rope_slack?.[i],ropeColor=slack?'#aa8c67':'#d7c581',pattern=slack?[4,4]:[];
     renderer.line(c,pulley,ropeColor,1.1,pattern);renderer.line(pulley,top,ropeColor,1.1,pattern);renderer.line(top,h,ropeColor,1.1,pattern);
     renderer.sphere(pulley,size,'#abbdd1');renderer.sphere(top,size,'#abbdd1');block(h,size*3.2,'#c69d48');
     labels.push([h,`H${i+1} ${(scene.hanging[i]*1000).toFixed(1)}g${slack?' 松绳':''}`,'#edc569']);
   }
 }
 renderer.sphere(scene.p,Math.max(size*1.7,8/scale),'#f3f8ff');renderer.draw();
 // Forces are annotations on top of solids, with a circle marking their true origin.
 for(const [f,color,name] of [[scene.force,'#ff7198','F外'],[scene.motor,'#a7e868','F电机']]){
   if(norm(f)<.01)continue;const p=project(scene.p),q=project(add(scene.p,mul(f,24/scale))),angle=Math.atan2(q.y-p.y,q.x-p.x);
   ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2.5;ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.lineTo(q.x,q.y);ctx.stroke();ctx.beginPath();ctx.arc(p.x,p.y,4,0,Math.PI*2);ctx.stroke();ctx.beginPath();ctx.moveTo(q.x,q.y);ctx.lineTo(q.x-8*Math.cos(angle-.4),q.y-8*Math.sin(angle-.4));ctx.lineTo(q.x-8*Math.cos(angle+.4),q.y-8*Math.sin(angle+.4));ctx.fill();ctx.font='12px sans-serif';
   const text=`${name} ${norm(f).toFixed(2)} N`,tx=q.x+9,ty=q.y+(name==='F外'?-12:22);
   ctx.fillStyle='#102033';ctx.fillRect(tx-3,ty-12,ctx.measureText(text).width+6,17);ctx.fillStyle=color;ctx.fillText(text,tx,ty);
 }
 if(scene.mouse_target){const a=project(scene.p),b=project(scene.mouse_target);ctx.strokeStyle='#ff7198';ctx.lineWidth=1.5;ctx.setLineDash([4,4]);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.setLineDash([]);ctx.beginPath();ctx.arc(b.x,b.y,6,0,Math.PI*2);ctx.stroke();}
 if(scene.labels){
   ctx.font='11px "Microsoft YaHei UI",sans-serif';const boxes=[];
   for(const [p,name,color] of labels){if(!renderer.visible(p,project,size*2))continue;const q=project(p),w=ctx.measureText(name).width+8;let box=null;
     for(const [dx,dy] of [[8,-19],[8,6],[-w-8,-19],[-w-8,6],[8,23]]){const b={x:q.x+dx,y:q.y+dy,w,h:17};if(b.x<0||b.x+w>width||b.y<50||b.y+17>height-35)continue;if(!boxes.some(a=>b.x<a.x+a.w&&b.x+w>a.x&&b.y<a.y+a.h&&b.y+b.h>a.y)){box=b;break;}}
     if(!box)continue;boxes.push(box);ctx.fillStyle='#102033';ctx.fillRect(box.x,box.y,box.w,box.h);ctx.fillStyle=color;ctx.fillText(name,box.x+4,box.y+12);
   }
 }
 // Screen-space orientation triad remains legible from every camera angle.
 for(const [axis,color,name] of [[[1,0,0],'#f88787','X'],[[0,1,0],'#8fce8f','Y'],[[0,0,1],'#80b6ef','Z']]){const x=width-65,y=65,dx=dot(axis,currentBasis.right)*30,dy=-dot(axis,currentBasis.up)*30;ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+dx,y+dy);ctx.stroke();ctx.fillText(name,x+dx+3,y+dy);}
 window.sceneRendered=true;
 const cost=performance.now()-started;window.renderStats.frames++;window.renderStats.lastDrawMs=cost;
 if(bridge&&scene.sequence)bridge.frameDone(scene.sequence,cost);
}
const EMPTY_POINTS=[];
window.updateScene=function(data){if(!scene||scene.design_key!==data.design_key){if(renderer)renderer.needsFraming=true;}scene=Object.assign(scene||{workspace:EMPTY_POINTS},data);window.renderStats.received++;scheduleDraw();};
window.resetCamera=function(){yaw=.68;pitch=.44;zoom=1;if(renderer)renderer.needsFraming=true;scheduleDraw();};
function pointerPosition(e){const r=canvas.getBoundingClientRect();return[e.clientX-r.left,e.clientY-r.top];}
function clearDrag(notify){
 const previous=drag;pendingMouseTarget=null;drag=null;canvas.style.cursor='grab';
 if(previous&&canvas.hasPointerCapture(previous.pointerId))canvas.releasePointerCapture(previous.pointerId);
 if(notify&&previous?.kind==='force'&&bridge)bridge.endDrag();
}
window.cancelPointerDrag=function(){clearDrag(false);};
function endDrag(){clearDrag(true);}
canvas.addEventListener('pointerdown',e=>{
 if(!scene||!renderer?.center||e.button!==0)return;
 e.preventDefault(); // Do not start native selection/drag or transfer focus.
 const [x,y]=pointerPosition(e),p=project(scene.p);
 const picked=Math.hypot(x-p.x,y-p.y)<Math.max(16,renderer.extent*.007*1.7*scale+5);
 if(picked&&bridge){
   if(!scene.can_drag){bridge.notify('请先应用修改后的参数。');return;}
   const planes={XY:[[1,0,0],[0,1,0]],XZ:[[1,0,0],[0,0,1]],YZ:[[0,1,0],[0,0,1]]};
   const [u,v]=planes[scene.drag_plane]||[currentBasis.right,currentBasis.up];
   const A=dot(u,currentBasis.right),B=dot(v,currentBasis.right),C=dot(u,currentBasis.up),D=dot(v,currentBasis.up),det=A*D-B*C;
   if(Math.abs(det)<.08){bridge.notify('当前施力平面接近侧视，请旋转视角或选择“随视角”。');return;}
   drag={kind:'force',x,y,origin:[...scene.p],u,v,A,B,C,D,det,scale};
   bridge.beginDrag(...scene.p);canvas.style.cursor='crosshair';
 }else{drag={kind:'orbit',x,y};canvas.style.cursor='grabbing';}
 drag.pointerId=e.pointerId;
 if(e.isTrusted){try{canvas.setPointerCapture(e.pointerId);}catch(_){endDrag();}}
});
canvas.addEventListener('pointermove',e=>{
 const [x,y]=pointerPosition(e);
 if(!drag){if(scene&&renderer?.center){const p=project(scene.p);canvas.style.cursor=Math.hypot(x-p.x,y-p.y)<18?'crosshair':'grab';}return;}
 if(e.pointerId!==drag.pointerId)return;
 // Pointer capture deliberately keeps the drag active outside the viewport.
 // The Python spring force remains capped at 12 N, including outside targets.
 if(!Number.isFinite(x)||!Number.isFinite(y))return;
 if(e.isTrusted&&(e.buttons&1)===0){endDrag();return;}
 if(drag.kind==='orbit'){yaw-=(x-drag.x)*.007;pitch=Math.max(-1.45,Math.min(1.45,pitch+(y-drag.y)*.007));drag.x=x;drag.y=y;scheduleDraw();return;}
 const dx=(x-drag.x)/drag.scale,dy=-(y-drag.y)/drag.scale;
 const a=(drag.D*dx-drag.B*dy)/drag.det,b=(drag.A*dy-drag.C*dx)/drag.det;
 const target=add(drag.origin,add(mul(drag.u,a),mul(drag.v,b)));
 pendingMouseTarget=target;scheduleDraw();
});
canvas.addEventListener('pointerup',endDrag);canvas.addEventListener('pointercancel',endDrag);canvas.addEventListener('lostpointercapture',endDrag);
window.addEventListener('pointerup',endDrag);
window.addEventListener('blur',()=>{if(bridge)bridge.viewBlur();});
canvas.addEventListener('dragstart',e=>e.preventDefault());
document.addEventListener('visibilitychange',()=>{if(document.hidden)endDrag();});
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.3,Math.min(5,zoom*Math.exp(-e.deltaY*.001)));scheduleDraw();},{passive:false});
canvas.addEventListener('dblclick',resetCamera);new ResizeObserver(scheduleDraw).observe(canvas);
