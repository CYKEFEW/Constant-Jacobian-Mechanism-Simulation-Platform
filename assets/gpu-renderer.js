/* Retained primitive meshes, instanced GPU transforms, direct WebGL display. */
(function(root){
'use strict';
const {add,sub,mul,dot,norm,unit,cross}=Physics;
class Stream {
 constructor(stride,capacity=4096){this.stride=stride;this.data=new Float32Array(capacity);this.used=0;this.buffer=null;this.allocated=0;}
 reset(){this.used=0;}
 append(values){if(this.used+values.length>this.data.length){const next=new Float32Array(Math.max(this.data.length*2,this.used+values.length));next.set(this.data);this.data=next;}this.data.set(values,this.used);this.used+=values.length;}
 upload(gl){if(!this.buffer)this.buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);if(this.allocated<this.data.byteLength){gl.bufferData(gl.ARRAY_BUFFER,this.data.byteLength,gl.DYNAMIC_DRAW);this.allocated=this.data.byteLength;}if(this.used)gl.bufferSubData(gl.ARRAY_BUFFER,0,this.data.subarray(0,this.used));}
}
class SolidRenderer{
 constructor(canvas){
  this.canvas=canvas;
  this.gl=canvas.getContext('webgl',{alpha:false,antialias:true,premultipliedAlpha:false,preserveDrawingBuffer:false});
  if(!this.gl)throw new Error('WebGL unavailable');
  const gl=this.gl;this.instancing=gl.getExtension('ANGLE_instanced_arrays');
  const camera=`uniform vec3 right,up,center;uniform mediump vec3 forward;uniform vec2 viewport;uniform float scale,depthRange,pointSize;
  varying mediump vec3 N,C;vec4 project(vec3 world){vec3 p=world-center;return vec4(2.0*dot(p,right)*scale/viewport.x,0.02+2.0*dot(p,up)*scale/viewport.y,-dot(p,forward)/depthRange,1.0);}`;
  const fragment=`precision mediump float;varying vec3 N,C;uniform vec3 forward;
  void main(){if(length(N)<0.000001){gl_FragColor=vec4(C,1.0);return;}vec3 n=normalize(N);if(dot(n,forward)<0.0)n=-n;
  vec3 light=normalize(vec3(0.6,-0.35,1.0));float diffuse=max(dot(n,light),0.0),rim=max(dot(n,normalize(vec3(-0.4,0.7,0.4))),0.0);
  float spec=pow(max(dot(n,normalize(light+forward)),0.0),35.0);gl_FragColor=vec4(C*(0.35+0.58*diffuse+0.14*rim)+vec3(0.17*spec),1.0);}`;
  this.meshProgram=this.program(`precision highp float;attribute vec3 position,normal,axisX,axisY,axisZ,offset,tint;${camera}
  void main(){gl_Position=project(offset+axisX*position.x+axisY*position.y+axisZ*position.z);N=normalize(axisX*normal.x+axisY*normal.y+axisZ*normal.z);C=tint;}`,fragment,['position','normal','axisX','axisY','axisZ','offset','tint']);
  this.flatProgram=this.program(`precision highp float;attribute vec3 position,normal,color;${camera}
  void main(){gl_Position=project(position);gl_PointSize=pointSize;N=normal;C=color;}`,fragment,['position','normal','color']);
  this.meshes={};this.buildMeshes();this.triangles=new Stream(9);this.lines=new Stream(9);this.pointStream=new Stream(9);
  this.pointReference=null;this.colorCache=new Map();this.faces=[];this.capsules=[];
  gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.disable(gl.BLEND);
 }
 program(vs,fs,attributes){const gl=this.gl,shader=(type,code)=>{const s=gl.createShader(type);gl.shaderSource(s,code);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;};
  const p=gl.createProgram();gl.attachShader(p,shader(gl.VERTEX_SHADER,vs));gl.attachShader(p,shader(gl.FRAGMENT_SHADER,fs));gl.linkProgram(p);if(!gl.getProgramParameter(p,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(p));
  return{p,attributes:attributes.map(n=>gl.getAttribLocation(p,n)),uniforms:Object.fromEntries(['right','up','forward','center','viewport','scale','depthRange','pointSize'].map(n=>[n,gl.getUniformLocation(p,n)]))};
 }
 buildMeshes(){
  const emit=(data,a,b,c,na,nb=na,nc=na)=>data.push(...a,...na,...b,...nb,...c,...nc);
  const store=(name,data)=>{const gl=this.gl,buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(data),gl.STATIC_DRAW);this.meshes[name]={buffer,count:data.length/6,instances:new Stream(15,2048)};};
  let data=[];
  for(let i=0;i<12;i++){const t=i*Math.PI/6,u=(i+1)*Math.PI/6,n=[Math.cos(t),Math.sin(t),0],m=[Math.cos(u),Math.sin(u),0],a=[...n],b=[n[0],n[1],1],c=[...m],d=[m[0],m[1],1];emit(data,a,b,c,n,n,m);emit(data,c,b,d,m,n,m);emit(data,[0,0,0],c,a,[0,0,-1]);emit(data,[0,0,1],b,d,[0,0,1]);}store('cylinder',data);
  data=[];const pos=(lat,lon)=>[Math.sin(lat)*Math.cos(lon),Math.sin(lat)*Math.sin(lon),Math.cos(lat)];
  for(let i=0;i<10;i++)for(let j=0;j<16;j++){const a=pos(i*Math.PI/10,j*Math.PI/8),b=pos((i+1)*Math.PI/10,j*Math.PI/8),c=pos(i*Math.PI/10,(j+1)*Math.PI/8),d=pos((i+1)*Math.PI/10,(j+1)*Math.PI/8);emit(data,a,b,c,a,b,c);emit(data,c,b,d,c,b,d);}store('sphere',data);
  this.boxVertices=[];for(let x of [-.5,.5])for(let y of [-.5,.5])for(let z of [-.5,.5])this.boxVertices.push([x,y,z]);
  this.boxFaces=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]];data=[];
  for(const ids of this.boxFaces){const p=ids.map(i=>this.boxVertices[i]),n=unit(cross(sub(p[1],p[0]),sub(p[2],p[0])));emit(data,p[0],p[1],p[2],n);emit(data,p[0],p[2],p[3],n);}store('box',data);
 }
 begin(width,height,dpr,basis,scale){this.width=width;this.height=height;this.dpr=dpr;this.scale=scale;this.basis=basis;this.faces.length=0;this.capsules.length=0;this.projected=null;this.projectedCapsules=null;
  this.triangles.reset();this.lines.reset();for(const mesh of Object.values(this.meshes))mesh.instances.reset();
  const w=Math.round(width*dpr),h=Math.round(height*dpr);if(this.canvas.width!==w||this.canvas.height!==h){this.canvas.width=w;this.canvas.height=h;}
 }
 color(hex){if(!this.colorCache.has(hex)){const n=parseInt(hex.slice(1,7),16);this.colorCache.set(hex,[(n>>16&255)/255,(n>>8&255)/255,(n&255)/255]);}return this.colorCache.get(hex);}
 instance(name,x,y,z,p,color){this.meshes[name].instances.append([...x,...y,...z,...p,...this.color(color)]);}
 triangle(a,b,c,color,normals,occluder=true){const rgb=this.color(color),n=normals||Array(3).fill(unit(cross(sub(b,a),sub(c,a))));this.triangles.append([...a,...n[0],...rgb,...b,...n[1],...rgb,...c,...n[2],...rgb]);if(occluder&&this.labelsEnabled)this.faces.push([a,b,c]);}
 polygon(points,color,unlit=false,occluder=true){const n=unlit?[0,0,0]:unit(cross(sub(points[1],points[0]),sub(points[2],points[0])));for(let i=1;i<points.length-1;i++)this.triangle(points[0],points[i],points[i+1],color,[n,n,n],occluder);}
 prism(points,thickness,color){const n=unit(cross(sub(points[1],points[0]),sub(points[2],points[0]))),front=points.map(p=>add(p,mul(n,thickness/2))),back=points.map(p=>sub(p,mul(n,thickness/2)));this.polygon(front,color);this.polygon([...back].reverse(),color);for(let i=0;i<points.length;i++){const j=(i+1)%points.length;this.polygon([front[i],back[i],back[j],front[j]],color);}}
 cylinder(a,b,r,color){const z=sub(b,a),axis=unit(z);if(norm(z)<1e-8)return;const x=mul(unit(cross(axis,Math.abs(axis[2])<.9?[0,0,1]:[0,1,0])),r),y=cross(axis,x);this.instance('cylinder',x,y,z,a,color);if(this.labelsEnabled)this.capsules.push({a,b,r});}
 sphere(p,r,color){this.instance('sphere',[r,0,0],[0,r,0],[0,0,r],p,color);}
 box(p,size,color){this.instance('box',[size,0,0],[0,size,0],[0,0,size],p,color);if(this.labelsEnabled){const v=this.boxVertices.map(a=>add(p,mul(a,size)));for(const [a,b,c,d] of this.boxFaces)this.faces.push([v[a],v[b],v[c]],[v[a],v[c],v[d]]);}}
 line(a,b,color,width,dash=[]){const rgb=this.color(color),segment=(p,q)=>this.lines.append([...p,0,0,0,...rgb,...q,0,0,0,...rgb]);if(!dash.length){segment(a,b);return;}
  const d=sub(b,a),pixels=Math.hypot(dot(d,this.basis.right),dot(d,this.basis.up))*this.scale;
  for(let x=0;x<pixels;x+=dash[0]+dash[1])segment(add(a,mul(d,x/pixels)),add(a,mul(d,Math.min(1,(x+dash[0])/pixels))));
 }
 points(points){if(this.pointReference===points)return;this.pointReference=points;this.pointStream.reset();for(const p of points)this.pointStream.append([...p,0,0,0,.278,.408,.482]);this.pointStream.upload(this.gl);}
 use(program){const gl=this.gl,u=program.uniforms;gl.useProgram(program.p);for(const k of ['right','up','forward'])gl.uniform3fv(u[k],this.basis[k]);gl.uniform3fv(u.center,this.center);gl.uniform2f(u.viewport,this.width,this.height);gl.uniform1f(u.scale,this.scale);gl.uniform1f(u.depthRange,this.depthRange||1);gl.uniform1f(u.pointSize,Math.max(1.5,this.dpr*1.5));}
 draw(){const gl=this.gl;gl.viewport(0,0,this.canvas.width,this.canvas.height);gl.clearColor(.051,.098,.161,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  this.use(this.meshProgram);const attrs=this.meshProgram.attributes;
  for(const mesh of Object.values(this.meshes)){
   const count=mesh.instances.used/15;if(!count)continue;
   gl.bindBuffer(gl.ARRAY_BUFFER,mesh.buffer);for(let i=0;i<2;i++){gl.enableVertexAttribArray(attrs[i]);gl.vertexAttribPointer(attrs[i],3,gl.FLOAT,false,24,i*12);}
   if(this.instancing){mesh.instances.upload(gl);for(let i=0;i<5;i++){gl.enableVertexAttribArray(attrs[i+2]);gl.vertexAttribPointer(attrs[i+2],3,gl.FLOAT,false,60,i*12);this.instancing.vertexAttribDivisorANGLE(attrs[i+2],1);}
    this.instancing.drawArraysInstancedANGLE(gl.TRIANGLES,0,mesh.count,count);
    for(let i=2;i<7;i++){this.instancing.vertexAttribDivisorANGLE(attrs[i],0);gl.disableVertexAttribArray(attrs[i]);}
   }else{for(let i=2;i<7;i++)gl.disableVertexAttribArray(attrs[i]);for(let j=0;j<count;j++){for(let i=0;i<5;i++){const o=j*15+i*3;gl.vertexAttrib3f(attrs[i+2],mesh.instances.data[o],mesh.instances.data[o+1],mesh.instances.data[o+2]);}gl.drawArrays(gl.TRIANGLES,0,mesh.count);}}
  }
  for(const a of attrs)gl.disableVertexAttribArray(a);
  this.use(this.flatProgram);const flat=(stream,mode,upload=true)=>{if(!stream.used)return;if(upload)stream.upload(gl);else gl.bindBuffer(gl.ARRAY_BUFFER,stream.buffer);for(let i=0;i<3;i++){const a=this.flatProgram.attributes[i];gl.enableVertexAttribArray(a);gl.vertexAttribPointer(a,3,gl.FLOAT,false,36,i*12);}gl.drawArrays(mode,0,stream.used/9);};
  flat(this.triangles,gl.TRIANGLES);gl.lineWidth(1);flat(this.lines,gl.LINES);flat(this.pointStream,gl.POINTS,false);
  for(const a of this.flatProgram.attributes)gl.disableVertexAttribArray(a);
 }
 visible(p,project,tolerance=.003){
  if(!this.projected)this.projected=this.faces.map(f=>f.map(project));const q=project(p);
  for(const [a,b,c] of this.projected){if(q.x<Math.min(a.x,b.x,c.x)||q.x>Math.max(a.x,b.x,c.x)||q.y<Math.min(a.y,b.y,c.y)||q.y>Math.max(a.y,b.y,c.y))continue;const den=(b.y-c.y)*(a.x-c.x)+(c.x-b.x)*(a.y-c.y);if(Math.abs(den)<1e-6)continue;const u=((b.y-c.y)*(q.x-c.x)+(c.x-b.x)*(q.y-c.y))/den,v=((c.y-a.y)*(q.x-c.x)+(a.x-c.x)*(q.y-c.y))/den;if(u>=0&&v>=0&&u+v<=1&&u*a.z+v*b.z+(1-u-v)*c.z>q.z+tolerance)return false;}
  if(!this.projectedCapsules)this.projectedCapsules=this.capsules.map(c=>({a:project(c.a),b:project(c.b),r:c.r}));
  for(const {a,b,r} of this.projectedCapsules){const dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy;if(l<1e-9)continue;const t=Math.max(0,Math.min(1,((q.x-a.x)*dx+(q.y-a.y)*dy)/l)),dist=Math.hypot(q.x-a.x-t*dx,q.y-a.y-t*dy)/this.scale;if(dist<r&&a.z+t*(b.z-a.z)+Math.sqrt(r*r-dist*dist)>q.z+tolerance)return false;}
  return true;
 }
}
root.SolidRenderer=SolidRenderer;
})(window);
