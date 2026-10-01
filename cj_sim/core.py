"""SI mechanics: uniform rods, point counterweights, unilateral ropes/end stops."""
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import numpy as np

ISO = float(np.degrees(np.arctan(1 / np.sqrt(2))))
# Near-critical damping for the default mechanism's 1.36–1.76 kg modal masses.
DEFAULT_STIFFNESS = 80.  # N/m, artificial centering about p=0
DEFAULT_DAMPING = 24.  # N s/m


@dataclass
class Leg:
    proximal_length: float = .09
    distal_length: float = .09
    proximal_mass: float = .06
    distal_mass: float = .06
    slider_mass: float = .045
    proximal_arm: float = .0585
    distal_arm: float = .0478


@dataclass
class Design:
    alpha: float = ISO
    platform_mass: float = .35
    platform_radius: float = .068
    anchor_distance: float = .105
    travel: float = .0275
    gravity: float = 9.81
    legs: list[Leg] = field(default_factory=lambda: [Leg() for _ in range(3)])

    @property
    def rail_total_travel(self):
        """Symmetric usable slider stroke, in metres (not physical rail stock length)."""
        return 2*self.travel

    @rail_total_travel.setter
    def rail_total_travel(self, value):
        self.travel = float(value)/2

    @property
    def rail_center_distance(self):
        """Perpendicular distance from the zero-pose platform origin to each rail axis."""
        return self.anchor_distance+self.platform_radius*np.sin(np.radians(self.alpha))

    @rail_center_distance.setter
    def rail_center_distance(self, value):
        self.anchor_distance = float(value)-self.platform_radius*np.sin(np.radians(self.alpha))

    def validate(self):
        values = [self.alpha, self.platform_mass, self.platform_radius,
                  self.anchor_distance, self.travel, self.gravity]
        values += [v for leg in self.legs for v in asdict(leg).values()]
        if not np.all(np.isfinite(values)):
            raise ValueError('参数必须为有限数值。')
        if len(self.legs) != 3 or not 2 <= self.alpha <= 85:
            raise ValueError('需要三条支链，倾角范围为 2–85°。')
        if min(self.platform_mass, self.platform_radius, self.anchor_distance, self.travel) <= 0 or self.gravity < 0:
            raise ValueError('质量与几何尺寸必须大于零，重力加速度不得为负。')
        for i, leg in enumerate(self.legs):
            if min(asdict(leg).values()) <= 0:
                raise ValueError(f'支链 {i+1} 的长度、质量和配重臂长必须大于零。')
            if not abs(leg.proximal_length-leg.distal_length)+1e-5 < self.anchor_distance < leg.proximal_length+leg.distal_length-1e-5:
                raise ValueError(f'支链 {i+1} 无法在中心装配：需 |L2−L3| < 安装间距 < L2+L3。')

    def save(self, path):
        Path(path).write_text(json.dumps({'schema': 1, 'units': 'SI', 'design': asdict(self)}, ensure_ascii=False, indent=2), encoding='utf-8')

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if data.get('schema') != 1 or data.get('units') != 'SI':
            raise ValueError('不支持的项目格式或单位。')
        params = dict(data['design'])
        params['legs'] = [Leg(**v) for v in params['legs']]
        design = cls(**params)
        design.validate()
        return design


class Mechanism:
    def __init__(self, design=None):
        self.design = design or Design()
        self.design.validate()
        d = self.design
        a = np.radians(d.alpha)
        s, c = np.sin(a), np.cos(a)
        self.G = np.array([[s, np.sqrt(3)*c/2, c/2], [s, -np.sqrt(3)*c/2, c/2], [s, 0, -c]])
        self.J = np.linalg.inv(self.G)
        self.condition = float(np.linalg.cond(self.J))
        theta = np.array([np.pi/6, 5*np.pi/6, -np.pi/2])
        self.offset = np.column_stack((np.zeros(3), d.platform_radius*np.cos(theta), d.platform_radius*np.sin(theta)))
        t = np.array([1., 0, 0])-self.G*self.G[:, :1]
        t /= np.linalg.norm(t, axis=1)[:, None]
        self.c0 = self.offset-d.anchor_distance*t
        self.l2 = np.array([x.proximal_length for x in d.legs])
        self.l3 = np.array([x.distal_length for x in d.legs])
        self.m2 = np.array([x.proximal_mass for x in d.legs])
        self.m3 = np.array([x.distal_mass for x in d.legs])
        self.ms = np.array([x.slider_mass for x in d.legs])
        self.r2 = np.array([x.proximal_arm for x in d.legs])
        self.r3 = np.array([x.distal_arm for x in d.legs])
        self.w3 = self.m3*self.l3/(2*self.r3)
        self.w2 = (self.m2*self.l2/2+(self.m3+self.w3)*self.l2)/self.r2
        self.branch = self.ms+self.m2+self.m3+self.w2+self.w3
        support = d.platform_mass*self.J[2]+self.branch*self.G[:, 2]
        self.hanging = np.abs(support)
        self.sign = np.where(support >= 0, 1., -1.)
        self.eye = np.eye(3)
        self.slider_D = self.G[:, :, None]*self.G[:, None, :]
        self.plane_D = self.eye-self.slider_D
        self.cross_n = np.array([[[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]] for n in self.G])
        self.cross_plane_D = self.cross_n@self.plane_D
        size = max(d.anchor_distance, max(self.l2+self.l3))
        self.top = np.column_stack((np.full(3,-size),np.linspace(-size*.85,size*.85,3),np.full(3,size)))
        self.pulley = self.c0+self.G*(self.sign*(d.travel+.05))[:,None]
        # Every mass point is an affine combination of A/B/C; H is overwritten.
        t0,t1=.5-1/(2*np.sqrt(3)),.5+1/(2*np.sqrt(3))
        self.ca=np.zeros((3,8));self.cb=np.zeros((3,8));self.cc=np.zeros((3,8))
        self.cc[:,0]=1
        self.cb[:,1:3]=[t0,t1];self.cc[:,1:3]=[1-t0,1-t1]
        self.ca[:,3:5]=[t0,t1];self.cb[:,3:5]=[1-t0,1-t1]
        self.cb[:,5]=-self.r2/self.l2;self.cc[:,5]=1+self.r2/self.l2
        self.ca[:,6]=-self.r3/self.l3;self.cb[:,6]=1+self.r3/self.l3
        self.hanging_D=np.zeros((3,3,3));self.hanging_D[:,2,:]=-self.sign[:,None]*self.G
        self.is_weight=np.r_[False,np.tile([False]*5+[True]*3,3)]
        self._mass_key=None
        self.geometry(np.zeros(3))

    def geometry(self, p):
        p = np.asarray(p, float)
        q = self.G@p
        a = self.offset+p
        c = self.c0+q[:, None]*self.G
        ac = a-c
        distance = np.linalg.norm(ac, axis=1)
        if np.any(distance <= abs(self.l2-self.l3)+1e-6) or np.any(distance >= self.l2+self.l3-1e-6):
            raise ValueError('到达连杆装配边界；请减小位移或调整杆长。')
        along = (self.l2**2-self.l3**2+distance**2)/(2*distance)
        height = np.sqrt(np.maximum(0, self.l2**2-along**2))
        direction = ac/distance[:, None]
        b = c+along[:, None]*direction+height[:, None]*np.cross(self.G, direction)
        return a, b, c, q

    def reachable(self, p):
        try:
            self.geometry(p)
            return bool(np.all(np.abs(self.G@p) <= self.design.travel+1e-12))
        except ValueError:
            return False

    def assembly(self, p, mode='full', ratio=1.):
        if mode not in ('none', 'rods', 'full') or not np.isfinite(ratio) or ratio < 0:
            raise ValueError('无效的配重模式或比例。')
        a, b, c, q = self.geometry(p)
        w3pos = b-(a-b)/self.l3[:, None]*self.r3[:, None]
        w2pos = c-(b-c)/self.l2[:, None]*self.r2[:, None]
        hanging_pos = self.top.copy()
        hanging_pos[:, 2] -= (self.design.travel+.10)+self.sign*q
        local = mode != 'none'
        w2, w3 = self.w2*local*ratio, self.w3*local*ratio
        hanging = self.hanging*(mode == 'full')*ratio
        positions=np.empty((25,3));positions[0]=p
        legs=self.ca[:,:,None]*a[:,None,:]+self.cb[:,:,None]*b[:,None,:]+self.cc[:,:,None]*c[:,None,:]
        legs[:,7,:]=hanging_pos;positions[1:]=legs.reshape(24,3)
        if self._mass_key!=(mode,ratio):
            self._masses=np.r_[self.design.platform_mass,np.column_stack((self.ms,self.m2/2,self.m2/2,self.m3/2,self.m3/2,w2,w3,hanging)).ravel()]
            self._mass_key=(mode,ratio)
        return dict(a=a, b=b, c=c, q=q, c0=self.c0, n=self.G, w2pos=w2pos, w3pos=w3pos,
                    w2=w2, w3=w3, hanging=hanging, hanging_pos=hanging_pos, top=self.top, pulley=self.pulley,
                    positions=positions, masses=self._masses, is_weight=self.is_weight)

    def properties(self, p, v=None, mode='full', ratio=1., gravity=True):
        p = np.asarray(p, float)
        v = np.zeros(3) if v is None else np.asarray(v, float)
        scene = self.assembly(p, mode, ratio)
        # B=C+f(d)r+k(d)(n×r). Exact derivatives avoid nine geometry builds
        # per integration step and remove finite-difference cancellation.
        r=scene['a']-scene['c'];d=np.linalg.norm(r,axis=1)
        delta=self.l2**2-self.l3**2
        f=(1+delta/d**2)/2;fp=-delta/d**3;fpp=3*delta/d**4
        k=np.sqrt(self.l2**2/d**2-f*f)
        s1=-2*self.l2**2/d**3-2*f*fp
        s2=6*self.l2**2/d**4-2*(fp*fp+f*fpp)
        kp=s1/(2*k);kpp=s2/(2*k)-s1*s1/(4*k**3)
        nr=np.einsum('nij,nj->ni',self.cross_n,r)
        radial=(fp[:,None]*r+kp[:,None]*nr)[:,:,None]*(r/d[:,None])[:,None,:]
        db=self.slider_D+f[:,None,None]*self.plane_D+k[:,None,None]*self.cross_plane_D+radial
        D=np.empty((25,3,3));D[0]=self.eye
        leg_D=self.ca[:,:,None,None]*self.eye+self.cb[:,:,None,None]*db[:,None,:,:]+self.cc[:,:,None,None]*self.slider_D[:,None,:,:]
        leg_D[:,7]=self.hanging_D;D[1:]=leg_D.reshape(24,3,3)
        masses = scene['masses']
        M = np.einsum('n,nki,nkj->ij', masses, D, D)
        g = self.design.gravity if gravity else 0.
        forces = -g*masses[:, None]*D[:, 2, :]
        structural = forces[~scene['is_weight']].sum(axis=0)
        compensation = forces[scene['is_weight']].sum(axis=0)
        bias = np.zeros(3)
        if np.linalg.norm(v) > 1e-7:
            rv=self.plane_D@v;dv=np.einsum('ni,ni->n',r,rv)/d
            ddv=(np.einsum('ni,ni->n',rv,rv)-dv*dv)/d
            nrv=np.einsum('nij,nj->ni',self.cross_n,rv)
            bdd=(fpp*dv*dv+fp*ddv)[:,None]*r+2*(fp*dv)[:,None]*rv+(kpp*dv*dv+kp*ddv)[:,None]*nr+2*(kp*dv)[:,None]*nrv
            curvature=np.zeros((25,3));curvature[1:]=(self.cb[:,:,None]*bdd[:,None,:]).reshape(24,3)
            bias = np.einsum('n,nki,nk->i', masses, D, curvature)
        return dict(scene=scene, M=M, bias=bias, gravity=structural, compensation=compensation,
                    residual=structural+compensation, potential=float(g*masses@scene['positions'][:, 2]),
                    kinetic=float(.5*v@M@v), total_mass=float(sum(masses)))

    def workspace(self, resolution=13):
        q = np.array(np.meshgrid(*([np.linspace(-self.design.travel, self.design.travel, resolution)]*3))).reshape(3, -1).T
        points = q@self.J.T
        return np.array([p for p in points if self.reachable(p)]).reshape(-1, 3)

    def balance_report(self, samples=5):
        points = self.workspace(samples)
        errors = [np.linalg.norm(self.properties(p)['residual']) for p in points]
        return dict(w2_kg=self.w2.tolist(), w3_kg=self.w3.tolist(), hanging_kg=self.hanging.tolist(),
                    rope_sign=self.sign.tolist(), J=self.J.tolist(), G=self.G.tolist(), condition=self.condition,
                    reachable_samples=len(points), max_balance_residual_N=float(max(errors, default=0)))


class Simulation:
    def __init__(self, mechanism):
        self.mechanism = mechanism
        self.mode = 'full'
        self.ratio = 1.
        self.gravity_enabled = True
        self.stiffness = DEFAULT_STIFFNESS
        self.damping = DEFAULT_DAMPING
        self.force = np.zeros(3)
        self.mouse_target = None
        self.mouse_stiffness = 220.
        self.mouse_force_limit = 12.
        self.reset()

    def reset(self, p=None):
        candidate = np.zeros(3) if p is None else np.asarray(p, float)
        if candidate.shape != (3,) or not np.all(np.isfinite(candidate)) or not self.mechanism.reachable(candidate):
            raise ValueError('目标位移超出杆长或滑块行程允许的工作空间。')
        self.p = candidate.copy(); self.v = np.zeros(3); self.time = 0.
        self.mouse_target = None
        self.history = []
        self._rope_key=None
        self.hanging_z=np.zeros(3)
        self.hanging_v=np.zeros(3)
        self.contact_active=False
        self.rope_slack=np.zeros(3,dtype=bool)

    @staticmethod
    def _contact_impulses(W,b):
        """Small convex unilateral-contact solve: impulse>=0, b+W*impulse>=0.

        Active-set dual QP with a line search when a multiplier turns negative.
        No impulses are applied to separated contacts unless the next step
        would cross them. Rope impulses are tensile only.
        """
        impulses=np.zeros(len(b));active=[]
        for _ in range(64):
            residual=b+W@impulses
            inactive=[i for i in range(len(b)) if i not in active]
            if not inactive or min(residual[inactive])>=-1e-10:return impulses
            active.append(min(inactive,key=lambda i:residual[i]))
            for _ in range(32):
                proposal=np.zeros(len(b))
                try:proposal[active]=np.linalg.solve(W[np.ix_(active,active)],-b[active])
                except np.linalg.LinAlgError:proposal[active]=np.linalg.lstsq(W[np.ix_(active,active)],-b[active],rcond=1e-12)[0]
                bad=[i for i in active if proposal[i]<=0]
                if not bad:impulses=proposal;break
                fraction=min(impulses[i]/(impulses[i]-proposal[i]) if impulses[i]>proposal[i] else 0. for i in bad)
                impulses+=fraction*(proposal-impulses)
                active=[i for i in active if impulses[i]>1e-12]
                if not active:break
            else:break
        # Bounded, convergent coordinate refinement for redundant corner contacts.
        diagonal=np.diag(W)
        for _ in range(80):
            for i in range(len(b)):
                if diagonal[i]>1e-14:impulses[i]=max(0.,impulses[i]-(b[i]+W[i]@impulses)/diagonal[i])
            if np.min(b+W@impulses)>-1e-9:return impulses
        raise ValueError('约束求解未收敛，请减小外力或检查极端机构参数。')

    def _constrained_velocity(self,result,dt):
        m=self.mechanism;scene=result['scene'];mass=scene['hanging'];g=m.design.gravity if self.gravity_enabled else 0.
        # Separate counterweights from the old taut-rope reduced mass matrix.
        # Their vertical coordinates are now independent while a rope is slack.
        bare_M=result['M']-np.einsum('i,ij,ik->jk',mass,m.G,m.G)
        inverse=np.linalg.inv(bare_M)
        rope_axes=m.sign[:,None]*m.G
        bare_gravity=result['residual']-g*(mass@rope_axes)
        free_v=self.v+dt*(inverse@(result['external']+result['motor']+bare_gravity-result['bias']))
        enabled=np.flatnonzero(mass>1e-12)
        size=3+len(enabled);inverse_mass=np.zeros((size,size));inverse_mass[:3,:3]=inverse
        if len(enabled):inverse_mass[3:,3:]=np.diag(1/mass[enabled])
        free=np.r_[free_v,self.hanging_v[enabled]-g*dt]
        taut_z=scene['top'][:,2]-(m.design.travel+.10)-m.sign*scene['q']
        gap=self.hanging_z-taut_z
        rows=[];gaps=[];types=[]
        for j,i in enumerate(enabled):
            row=np.zeros(size);row[:3]=rope_axes[i];row[3+j]=1
            rows.append(row);gaps.append(max(0.,gap[i]));types.append(('rope',i))
        # Finite end stops on the three sliders. The geometric margins stay
        # slightly inside the rod singularities, where derivatives diverge.
        for sign in (-1,1):
            for i in range(3):
                row=np.zeros(size);row[:3]=sign*m.G[i]
                rows.append(row);gaps.append(m.design.travel+sign*scene['q'][i]);types.append(('stop',i))
        ac=scene['a']-scene['c'];distance=np.linalg.norm(ac,axis=1)
        margin=1.05e-6
        for sign in (-1,1):
            for i in range(3):
                row=np.zeros(size);row[:3]=sign*ac[i]/distance[i]
                clearance=(distance[i]-abs(m.l2[i]-m.l3[i])-margin) if sign==1 else (m.l2[i]+m.l3[i]-margin-distance[i])
                rows.append(row);gaps.append(clearance);types.append(('stop',i))
        A=np.array(rows);gaps=np.maximum(gaps,0.)
        b=A@free+gaps/dt
        # Most boundaries are far away: avoid forming and solving their QP.
        relevant=b<1e-9
        impulses=np.zeros(len(b))
        if np.any(relevant):
            # A rope/stop initially safe can become active due to another impulse.
            W=A@inverse_mass@A.T
            impulses=self._contact_impulses(W,b)
            free+=inverse_mass@A.T@impulses
        tension=np.zeros(3);contact=False
        for impulse,(kind,i) in zip(impulses,types):
            if kind=='rope':tension[i]=impulse/dt
            elif impulse>1e-10:contact=True
        hv=self.hanging_v.copy();hv[enabled]=free[3:]
        return free[:3],hv,tension,contact,taut_z,gap

    def mouse_force(self):
        if self.mouse_target is None:
            return np.zeros(3)
        force=self.mouse_stiffness*(self.mouse_target-self.p)
        magnitude=np.linalg.norm(force)
        if magnitude>self.mouse_force_limit:
            force*=self.mouse_force_limit/magnitude
        return force

    def set_mouse_target(self, target):
        if target is None:
            self.mouse_target=None
            return
        value=np.asarray(target,float)
        if value.shape!=(3,) or not np.all(np.isfinite(value)):
            raise ValueError('鼠标目标必须为有限三维坐标。')
        self.mouse_target=value.copy()

    def state(self,dt=1/240):
        result = self.mechanism.properties(self.p, self.v, self.mode, self.ratio, self.gravity_enabled)
        scene=result['scene'];m=self.mechanism
        key=(self.mode,self.ratio)
        if self._rope_key!=key:
            self.hanging_z=scene['hanging_pos'][:,2].copy()
            self.hanging_v=-m.sign*(m.G@self.v)
            self._rope_key=key
        motor = -self.stiffness*self.p-self.damping*self.v
        result['motor'] = motor
        result['mouse_force'] = self.mouse_force()
        result['external'] = self.force+result['mouse_force']
        result['tau'] = self.mechanism.J.T@motor
        velocity,hv,tension,contact,taut_z,gap=self._constrained_velocity(result,dt)
        result['acceleration']=(velocity-self.v)/dt
        result['next_velocity']=velocity;result['next_hanging_velocity']=hv
        result['tension']=tension;result['contact']=contact
        result['rope_slack']=(scene['hanging']>0)&((gap>1e-7)|((tension<1e-9)&(hv+m.sign*(m.G@velocity)>1e-7)))
        g=m.design.gravity if self.gravity_enabled else 0.
        result['potential']+=float(g*scene['hanging']@(self.hanging_z-taut_z))
        result['kinetic']+=float(.5*scene['hanging']@(self.hanging_v**2-(m.G@self.v)**2))
        # Nominal static gravity balance is still useful for design; actual rope
        # force is reported separately and used by the unilateral dynamics.
        result['rope_force']=tension@(m.sign[:,None]*m.G)
        scene['hanging_pos'][:,2]=self.hanging_z
        scene['positions'][8::8,2]=self.hanging_z
        scene['rope_slack']=result['rope_slack']
        return result

    def step(self, dt=1/240):
        if not np.isfinite(dt) or dt<=0:raise ValueError('积分步长必须为正有限数。')
        result = self.state(dt)
        v = result['next_velocity']
        p = self.p+dt*v
        if not self.mechanism.reachable(p):
            # Curved assembly limits need a position-level correction after the
            # tangent contact solve. Bisection cannot jump across a singularity.
            low,high=0.,1.
            for _ in range(35):
                mid=(low+high)/2
                if self.mechanism.reachable(self.p+mid*dt*v):low=mid
                else:high=mid
            p=self.p+max(0.,low-1e-8)*dt*v
            result['contact']=True
        self.hanging_v=result['next_hanging_velocity']
        self.hanging_z+=dt*self.hanging_v
        taut_z=result['scene']['top'][:,2]-(self.mechanism.design.travel+.10)-self.mechanism.sign*(self.mechanism.G@p)
        self.hanging_z=np.maximum(self.hanging_z,taut_z)
        self.contact_active=result['contact'];self.rope_slack=result['rope_slack']
        self.p, self.v = p, v
        self.time += dt
        return result

    def record(self, state):
        self.history.append([self.time, *self.p, *self.v, *(self.mechanism.G@self.p),
                             *state['tau'], *state['residual'], state['kinetic'], state['potential'],
                             *state['external'], *state['mouse_force']])

    def export_csv(self, path):
        np.savetxt(path, np.asarray(self.history).reshape(-1, 24), delimiter=',', comments='',
                   header='time_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,q1_m,q2_m,q3_m,tau1_N,tau2_N,tau3_N,Fgx_N,Fgy_N,Fgz_N,kinetic_J,potential_J,Fext_x_N,Fext_y_N,Fext_z_N,Fmouse_x_N,Fmouse_y_N,Fmouse_z_N', encoding='utf-8')
