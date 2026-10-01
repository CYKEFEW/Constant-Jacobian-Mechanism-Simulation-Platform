import json
import tempfile
import subprocess
import shutil
import unittest
from pathlib import Path
import numpy as np
from cj_sim.core import Design, Leg, Mechanism, Simulation, ISO


class MechanicsTests(unittest.TestCase):
    def test_parameterized_rail_distance_and_stroke(self):
        for alpha in (25,ISO,50):
            design=Design(alpha=alpha)
            design.rail_center_distance=.145
            design.rail_total_travel=.04
            m=Mechanism(design)
            distances=np.linalg.norm(m.c0-np.sum(m.c0*m.G,axis=1)[:,None]*m.G,axis=1)
            np.testing.assert_allclose(distances,.145,atol=1e-12)
            np.testing.assert_allclose(np.linalg.norm(2*design.travel*m.G,axis=1),.04)
            self.assertTrue(m.reachable(m.J@np.array([.0199,0,0])))
            self.assertFalse(m.reachable(m.J@np.array([.0201,0,0])))
            np.testing.assert_allclose(m.properties(np.zeros(3))['residual'],0,atol=1e-10)
            with tempfile.TemporaryDirectory() as temp:
                path=Path(temp)/'rails.json';design.save(path);loaded=Design.load(path)
                self.assertAlmostEqual(loaded.rail_center_distance,.145)
                self.assertAlmostEqual(loaded.rail_total_travel,.04)
        invalid=Design();invalid.rail_center_distance=.01
        with self.assertRaises(ValueError):Mechanism(invalid)
        invalid=Design();invalid.rail_total_travel=0
        with self.assertRaises(ValueError):Mechanism(invalid)

    def test_analytic_dynamics_against_independent_finite_differences(self):
        rng=np.random.default_rng(42)
        m=Mechanism(Design(alpha=48,legs=[Leg(.11,.08,.09,.045,.04,.06,.04),Leg(.085,.105,.04,.12,.06,.07,.055),Leg(.13,.10,.10,.08,.05,.08,.045)]))
        for mode in ('none','rods','full'):
            for _ in range(12):
                p=rng.uniform(-.012,.012,3);v=rng.uniform(-.03,.03,3);ratio=.83
                result=m.properties(p,v,mode,ratio)
                scene=m.assembly(p,mode,ratio);mass=scene['masses']
                eps=2e-6;h=.002
                D=np.stack([(m.assembly(p+e,mode,ratio)['positions']-m.assembly(p-e,mode,ratio)['positions'])/(2*eps) for e in np.eye(3)*eps],axis=2)
                curvature=(m.assembly(p+h*v,mode,ratio)['positions']-2*scene['positions']+m.assembly(p-h*v,mode,ratio)['positions'])/h**2
                expected_M=np.einsum('n,nki,nkj->ij',mass,D,D)
                expected_bias=np.einsum('n,nki,nk->i',mass,D,curvature)
                np.testing.assert_allclose(result['M'],expected_M,atol=1e-7,rtol=1e-6)
                np.testing.assert_allclose(result['bias'],expected_bias,atol=1e-7,rtol=2e-5)

    def test_existing_web_model_parity_when_available(self):
        reference=Path(__file__).resolve().parents[2]/'网页端仿真'/'jacobian-lab'/'dist'/'physics.js'
        if not reference.exists() or not shutil.which('node'):
            self.skipTest('Optional original web reference is not present')
        script="""const P=require(process.argv[1]);const {G,J}=P.matrices();
        const rows=[];for(const mode of ['none','rods','full'])for(const p of [[0,0,0],[.005,-.009,.008],[-.004,.007,-.011]]){
          const r=P.massProperties(p,[.01,-.004,.005],G,J,mode,1,9.81);
          rows.push({mode,p,M:r.M,bias:r.bias,gravity:r.gravity,compensation:r.compensation});
        }process.stdout.write(JSON.stringify(rows));"""
        rows=json.loads(subprocess.check_output(['node','-e',script,str(reference)],text=True,encoding='utf-8'))
        m=Mechanism()
        for row in rows:
            state=m.properties(row['p'],[.01,-.004,.005],mode=row['mode'])
            for key in ['M','gravity','compensation']:
                np.testing.assert_allclose(state[key],row[key],atol=1e-8)
            np.testing.assert_allclose(state['bias'],row['bias'],atol=2e-8)

    def test_default_reference_and_virtual_work(self):
        m=Mechanism()
        np.testing.assert_allclose(m.J.T@m.J,np.eye(3),atol=1e-14)
        np.testing.assert_allclose(m.w3,np.full(3,.06*.09/(2*.0478)))
        np.testing.assert_allclose(m.w2,np.full(3,(.06*.09/2+(.06+m.w3[0])*.09)/.0585))
        self.assertAlmostEqual(m.hanging[2],2*m.hanging[0])
        rng=np.random.default_rng(7)
        for _ in range(50):
            p=rng.uniform(-.012,.012,3);f=rng.normal(size=3);dq=rng.normal(size=3)
            np.testing.assert_allclose(m.J@(m.G@p),p,atol=1e-14)
            self.assertAlmostEqual(float(f@(m.J@dq)),float((m.J.T@f)@dq),12)

    def test_unequal_rods_asymmetric_masses_balance_every_pose(self):
        d=Design(alpha=48,legs=[Leg(.11,.08,.09,.045,.04,.06,.04),Leg(.085,.105,.04,.12,.06,.07,.055),Leg(.13,.10,.10,.08,.05,.08,.045)])
        m=Mechanism(d);reference=None
        for p in m.workspace(5):
            a,b,c,q=m.geometry(p)
            np.testing.assert_allclose(np.linalg.norm(b-c,axis=1),m.l2,atol=1e-13)
            np.testing.assert_allclose(np.linalg.norm(a-b,axis=1),m.l3,atol=1e-13)
            np.testing.assert_allclose(np.einsum('ij,ij->i',a-c,m.G),0,atol=1e-13)
            state=m.properties(p)
            np.testing.assert_allclose(state['residual'],0,atol=2e-7)
            self.assertTrue(np.linalg.eigvalsh(state['M']).min()>0)
            if reference is None: reference=state['potential']
            self.assertAlmostEqual(state['potential'],reference,10)

    def test_gravity_is_potential_gradient_and_inertia_survives(self):
        m=Mechanism();p=np.array([.004,-.006,.008]);delta=2e-6
        for mode in ['none','rods','full']:
            state=m.properties(p,mode=mode,ratio=.83)
            gradient=np.array([(m.properties(p+e,mode=mode,ratio=.83)['potential']-m.properties(p-e,mode=mode,ratio=.83)['potential'])/(2*delta) for e in np.eye(3)*delta])
            np.testing.assert_allclose(-gradient,state['residual'],atol=1e-7)
            no_g=m.properties(p,mode=mode,ratio=.83,gravity=False)
            np.testing.assert_allclose(state['M'],no_g['M'])
            np.testing.assert_allclose(no_g['residual'],0)

    def test_release_modes_and_force_response(self):
        p=np.array([.009,-.008,.012])
        for mode,gravity,should_drop in [('full',True,False),('none',False,False),('none',True,True)]:
            sim=Simulation(Mechanism());sim.mode=mode;sim.gravity_enabled=gravity;sim.reset(p)
            sim.stiffness=0;sim.damping=3  # Passive release, independent of UI defaults.
            for _ in range(12):sim.step()
            if should_drop:self.assertLess(sim.p[2],p[2]-.002)
            else:np.testing.assert_allclose(sim.p,p,atol=1e-8)
        sim=Simulation(Mechanism());sim.mode='none';sim.gravity_enabled=False;sim.force=np.array([1.,0,0])
        for _ in range(12):sim.step()
        self.assertGreater(sim.p[0],0)

    def test_default_centering_settles_without_rebound(self):
        sim=Simulation(Mechanism())
        self.assertEqual(sim.stiffness,80)
        self.assertEqual(sim.damping,24)
        initial=np.array([.009,-.008,.012]);sim.reset(initial)
        previous=np.linalg.norm(initial)
        for i in range(480):
            sim.step()
            distance=np.linalg.norm(sim.p)
            self.assertTrue(np.all(np.isfinite(sim.p)))
            self.assertLessEqual(distance,previous+1e-10)
            self.assertTrue(np.all(sim.p*initial>=-1e-10))
            if i>=180:self.assertLess(distance,.001)
            previous=distance
        self.assertLess(previous,.00001)

    def test_project_roundtrip_and_csv(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'design.json';Design().save(path)
            self.assertEqual(Design.load(path),Design())
            sim=Simulation(Mechanism());sim.record(sim.state());sim.step();sim.record(sim.state())
            sim.export_csv(Path(temp)/'trace.csv')
            data=np.loadtxt(Path(temp)/'trace.csv',delimiter=',',skiprows=1)
            self.assertEqual(data.shape,(2,24));self.assertGreater(data[1,0],data[0,0])

    def test_mouse_spring_limit_motion_release_and_reset(self):
        sim=Simulation(Mechanism());sim.mode='none';sim.gravity_enabled=False
        sim.set_mouse_target([.01,0,0])
        np.testing.assert_allclose(sim.state()['mouse_force'],[2.2,0,0])
        self.assertEqual(sim.time,0.)
        np.testing.assert_allclose(sim.p,0)
        for _ in range(12):sim.step()
        self.assertGreater(sim.p[0],0)
        sim.set_mouse_target([10,20,30])
        self.assertAlmostEqual(np.linalg.norm(sim.mouse_force()),12,12)
        previous_velocity=sim.v.copy()
        sim.set_mouse_target(None)
        np.testing.assert_allclose(sim.state()['mouse_force'],0)
        np.testing.assert_allclose(sim.v,previous_velocity)
        sim.set_mouse_target([.01,0,0]);sim.reset()
        self.assertIsNone(sim.mouse_target)
        with self.assertRaises(ValueError):sim.set_mouse_target([float('nan'),0,0])

    def test_ropes_remain_constant_and_mass_follows_arm_position(self):
        m=Mechanism();lengths=[]
        for p in [np.zeros(3),np.array([.005,-.009,.008]),np.array([-.004,.007,-.011])]:
            scene=m.assembly(p)
            lengths.append(np.linalg.norm(scene['c']-scene['pulley'],axis=1)
                           +np.linalg.norm(scene['pulley']-scene['top'],axis=1)
                           +np.linalg.norm(scene['top']-scene['hanging_pos'],axis=1))
        np.testing.assert_allclose(lengths[1:],np.tile(lengths[0],(2,1)),atol=1e-13)
        d=Design();d.legs[0].distal_arm*=2
        changed=Mechanism(d)
        self.assertAlmostEqual(changed.w3[0],m.w3[0]/2)
        self.assertLess(changed.w2[0],m.w2[0])
        self.assertLess(changed.hanging[0],m.hanging[0])

    def test_invalid_parameters_and_continuous_end_stops(self):
        for d in [Design(alpha=0),Design(legs=[Leg()]),Design(legs=[Leg(.01,.01),Leg(),Leg()]),Design(platform_mass=float('nan'))]:
            with self.assertRaises(ValueError):Mechanism(d)
        sim=Simulation(Mechanism())
        with self.assertRaises(ValueError):sim.reset([1,0,0])
        sim.mode='none';sim.gravity_enabled=False;sim.force=np.array([200.,0,0])
        contacts=0
        for _ in range(100):
            sim.step();contacts+=sim.contact_active
        self.assertGreater(contacts,0)
        self.assertAlmostEqual(sim.time,100/240)
        self.assertTrue(sim.mechanism.reachable(sim.p))

    def test_fast_large_drag_can_reverse_and_release(self):
        for mode,gravity in [('full',True),('none',False),('full',False)]:
            for axis in range(3):
                sim=Simulation(Mechanism());sim.mode=mode;sim.gravity_enabled=gravity
                target=np.zeros(3);target[axis]=.25
                sim.set_mouse_target(target)
                hit=False
                for _ in range(100):
                    sim.step();hit|=sim.contact_active
                    self.assertTrue(sim.mechanism.reachable(sim.p))
                    self.assertTrue(np.all(np.isfinite(sim.v)))
                self.assertTrue(hit)
                edge=sim.p[axis]
                sim.set_mouse_target(-target)
                for _ in range(100):sim.step()
                self.assertLess(sim.p[axis],edge-.005)
                sim.set_mouse_target(None)
                for _ in range(60):sim.step()
                self.assertAlmostEqual(sim.time,260/240)
                np.testing.assert_allclose(sim.mouse_force(),0)

    def test_tension_sign_slack_free_fall_and_reconnection(self):
        sim=Simulation(Mechanism())
        sim.set_mouse_target([.012,-.01,.014])
        result=sim.state()
        expected=result['scene']['hanging']*(sim.mechanism.design.gravity-sim.mechanism.sign*(sim.mechanism.G@result['acceleration']))
        np.testing.assert_allclose(result['tension'],expected,atol=1e-9)
        # A raised counterweight starts with excess rope, falls freely, then
        # reconnects. It must never exert a compressive force on the platform.
        sim.reset();sim.state();sim.hanging_z[0]+=.015
        previous=sim.hanging_z[0];saw_taut=False
        for i in range(120):
            result=sim.step()
            self.assertGreaterEqual(result['tension'].min(),0.)
            if i==0:
                self.assertEqual(result['tension'][0],0.)
                self.assertLess(sim.hanging_z[0],previous)
                self.assertAlmostEqual(sim.hanging_v[0],-9.81/240)
            if result['tension'][0]>.01:saw_taut=True
            taut=result['scene']['top'][:,2]-(sim.mechanism.design.travel+.10)-sim.mechanism.sign*(sim.mechanism.G@sim.p)
            self.assertGreaterEqual(float(np.min(sim.hanging_z-taut)),-1e-12)
        self.assertTrue(saw_taut)

    def test_end_stop_impulse_does_not_add_kinetic_energy(self):
        sim=Simulation(Mechanism());sim.mode='none';sim.gravity_enabled=False;sim.damping=0;sim.stiffness=0
        sim.reset(sim.mechanism.J@np.array([.02749,0,0]));sim.v=sim.mechanism.G[0]*.5
        before=sim.state()['kinetic'];sim.step()
        self.assertTrue(sim.contact_active)
        self.assertLessEqual(sim.state()['kinetic'],before+1e-8)

    def test_small_step_energy_conservation(self):
        sim=Simulation(Mechanism());sim.damping=0;sim.stiffness=0;sim.reset([.004,-.003,.006]);sim.v=np.array([.005,-.003,.004])
        initial=sim.state();e0=initial['kinetic']+initial['potential']
        for _ in range(80):sim.step(1/1000)
        final=sim.state();e1=final['kinetic']+final['potential']
        self.assertLess(abs(e1-e0),1e-7)


if __name__=='__main__':unittest.main()
