/* Opaque, depth-tested WebGL rendering. No network or third-party dependency. */
(function(root){
'use strict';
const {add,sub,mul,dot,norm,unit,cross}=Physics;
class SolidRenderer{
constructor(){
this.canvas=document.createElement('canvas');
this.gl=this.canvas.getContext('webgl',{alpha:true,antialias:true,premultipliedAlpha:false,preserveDrawingBuffer:true});
if(!this.gl)throw new Error('WebGL unavailable');
const gl=this.gl;
const shader=(type,source)=>{const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(s));return s;};
const vs=shader(gl.VERTEX_SHADER,`precision mediump float;attribute vec3 position;attribute vec3 normal;attribute vec3 color;
uniform vec3 right;uniform vec3 up;uniform vec3 forward;uniform vec3 center;uniform vec2 viewport;uniform float scale;uniform float depthRange;
varying vec3 N;varying vec3 C;
void main(){vec3 p=position-center;gl_Position=vec4(2.0*dot(p,right)*scale/viewport.x,0.02+2.0*dot(p,up)*scale/viewport.y,-dot(p,forward)/depthRange,1.0);N=normal;C=color;}`);
const fs=shader(gl.FRAGMENT_SHADER,`precision mediump float;varying vec3 N;varying vec3 C;uniform vec3 forward;
void main(){float len=length(N);if(len<0.1){gl_FragColor=vec4(C,1.0);return;}
vec3 n=normalize(N);vec3 light=normalize(vec3(0.6,-0.35,1.0));
float diffuse=max(dot(n,light),0.0);float rim=max(dot(n,normalize(vec3(-0.4,0.7,0.4))),0.0);
float spec=pow(max(dot(n,normalize(light+forward)),0.0),35.0);
gl_FragColor=vec4(C*(0.35+0.58*diffuse+0.14*rim)+vec3(0.17*spec),1.0);}`);
this.program=gl.createProgram();gl.attachShader(this.program,vs);gl.attachShader(this.program,fs);gl.linkProgram(this.program);
if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(this.program));
this.buffer=gl.createBuffer();this.locations={};for(const name of ['right','up','forward','center','viewport','scale','depthRange'])this.locations[name]=gl.getUniformLocation(this.program,name);
this.attributes=['position','normal','color'].map(n=>gl.getAttribLocation(this.program,n));
gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.disable(gl.BLEND);this.colorCache=new Map();
}
begin(width,height,dpr,basis,scale){this.width=width;this.height=height;this.scale=scale;this.basis=basis;this.data=[];this.faces=[];this.projected=null;const w=Math.round(width*dpr),h=Math.round(height*dpr);if(this.canvas.width!==w||this.canvas.height!==h){this.canvas.width=w;this.canvas.height=h;}}
color(hex){if(!this.colorCache.has(hex)){const n=parseInt(hex.slice(1,7),16);this.colorCache.set(hex,[(n>>16&255)/255,(n>>8&255)/255,(n&255)/255]);}return this.colorCache.get(hex);}
triangle(a,b,c,color,normals,occluder=true){const rgb=this.color(color),n=normals||Array(3).fill(unit(cross(sub(b,a),sub(c,a))));for(const [i,p] of [a,b,c].entries())this.data.push(...p,...n[i],...rgb);if(occluder)this.faces.push([a,b,c]);}
polygon(points,color,unlit=false,occluder=true){let n=unit(cross(sub(points[1],points[0]),sub(points[2],points[0])));if(dot(n,this.basis.forward)<0)n=mul(n,-1);for(let i=1;i<points.length-1;i++)this.triangle(points[0],points[i],points[i+1],color,Array(3).fill(unlit?[0,0,0]:n),occluder);}
prism(points,thickness,color){const n=unit(cross(sub(points[1],points[0]),sub(points[2],points[0]))),front=points.map(p=>add(p,mul(n,thickness/2))),back=points.map(p=>sub(p,mul(n,thickness/2)));this.polygon(front,color);this.polygon([...back].reverse(),color);for(let i=0;i<points.length;i++){const j=(i+1)%points.length;this.polygon([front[i],back[i],back[j],front[j]],color);}}
cylinder(a,b,r,color,unlit=false,occluder=true,sides=12){const axis=unit(sub(b,a));if(norm(sub(b,a))<1e-8)return;const u=unit(cross(axis,Math.abs(axis[2])<.9?[0,0,1]:[0,1,0])),v=cross(axis,u);for(let i=0;i<sides;i++){const t=i*2*Math.PI/sides,t2=(i+1)*2*Math.PI/sides,n=add(mul(u,Math.cos(t)),mul(v,Math.sin(t))),n2=add(mul(u,Math.cos(t2)),mul(v,Math.sin(t2))),a1=add(a,mul(n,r)),a2=add(a,mul(n2,r)),b1=add(b,mul(n,r)),b2=add(b,mul(n2,r));const zero=[0,0,0];this.triangle(a1,b1,a2,color,unlit?[zero,zero,zero]:[n,n,n2],occluder);this.triangle(a2,b1,b2,color,unlit?[zero,zero,zero]:[n2,n,n2],occluder);this.triangle(a,a2,a1,color,Array(3).fill(unlit?zero:mul(axis,-1)),occluder);this.triangle(b,b1,b2,color,Array(3).fill(unlit?zero:axis),occluder);}}
line(a,b,color,width,dash=[]){const r=width>=4?width*.0005:width/(2*this.scale);if(!dash.length){this.cylinder(a,b,r,color,width<3,width>=3,width<3?5:12);return;}
const d=sub(b,a),pixels=Math.hypot(dot(d,this.basis.right),dot(d,this.basis.up))*this.scale,cycle=dash[0]+dash[1];for(let x=0;x<pixels;x+=cycle){this.cylinder(add(a,mul(d,x/pixels)),add(a,mul(d,Math.min(1,(x+dash[0])/pixels))),r,color,true,false,5);}}
sphere(center,r,color){const pos=(lat,lon)=>[Math.sin(lat)*Math.cos(lon),Math.sin(lat)*Math.sin(lon),Math.cos(lat)];for(let i=0;i<10;i++)for(let j=0;j<16;j++){const a=pos(i*Math.PI/10,j*Math.PI/8),b=pos((i+1)*Math.PI/10,j*Math.PI/8),c=pos(i*Math.PI/10,(j+1)*Math.PI/8),d=pos((i+1)*Math.PI/10,(j+1)*Math.PI/8);this.triangle(add(center,mul(a,r)),add(center,mul(b,r)),add(center,mul(c,r)),color,[a,b,c]);this.triangle(add(center,mul(c,r)),add(center,mul(b,r)),add(center,mul(d,r)),color,[c,b,d]);}}
draw(ctx){const gl=this.gl;gl.viewport(0,0,this.canvas.width,this.canvas.height);gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(this.data),gl.DYNAMIC_DRAW);this.attributes.forEach((a,i)=>{gl.enableVertexAttribArray(a);gl.vertexAttribPointer(a,3,gl.FLOAT,false,36,i*12);});for(const key of ['right','up','forward'])gl.uniform3fv(this.locations[key],this.basis[key]);gl.uniform3fv(this.locations.center,this.center||[-.065,0,.005]);gl.uniform2f(this.locations.viewport,this.width,this.height);gl.uniform1f(this.locations.scale,this.scale);gl.uniform1f(this.locations.depthRange,this.depthRange||1);gl.drawArrays(gl.TRIANGLES,0,this.data.length/9);ctx.drawImage(this.canvas,0,0,this.width,this.height);}
visible(p,project,tolerance=.003){if(!this.projected)this.projected=this.faces.map(f=>f.map(project));const q=project(p);for(const [a,b,c] of this.projected){if(q.x<Math.min(a.x,b.x,c.x)||q.x>Math.max(a.x,b.x,c.x)||q.y<Math.min(a.y,b.y,c.y)||q.y>Math.max(a.y,b.y,c.y))continue;const den=(b.y-c.y)*(a.x-c.x)+(c.x-b.x)*(a.y-c.y);if(Math.abs(den)<1e-6)continue;const u=((b.y-c.y)*(q.x-c.x)+(c.x-b.x)*(q.y-c.y))/den,v=((c.y-a.y)*(q.x-c.x)+(a.x-c.x)*(q.y-c.y))/den;if(u>=0&&v>=0&&u+v<=1&&u*a.z+v*b.z+(1-u-v)*c.z>q.z+tolerance)return false;}return true;}
}
root.SolidRenderer=SolidRenderer;
})(window);
