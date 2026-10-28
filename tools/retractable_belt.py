"""MuJoCo deformable belt operations. Reel feed is prescribed, not spring dynamics.

The gripper and latch use explicit constraints after geometric/contact checks.
No claim of friction-only grasping, vendor latch mechanics or force calibration.
"""
import numpy as np
import mujoco

class RetractableBelt:
    N=82
    W=3
    DS=.02
    LENGTH=1.62
    RADIUS=.08

    def __init__(self, model, data, prefix='', origin=(0,0,0)):
        self.m,self.d,self.prefix=model,data,prefix
        self.origin=np.asarray(origin,dtype=float)
        self.outlet=self.origin+[-.82,0,.98]
        self.seat=self.origin+[.78,0,.98]
        self.park=self.origin+[-.67,0,.98]
        self.stored=.02
        self.phase='latched'
        self.events=[]
        self.open_authorized=False
        self.close_authorized=False
        self.handle=self.id('body','belt_handle')
        self.gripper=int(model.body_mocapid[self.id('body','belt_gripper')])
        self.guides=np.array([model.body_mocapid[self.id('body',f'reel_guide_{i}')] for i in range(self.N)])
        self.feed=np.array([[self.id('equality',f'feed_{i}_{j}') for j in range(self.W)] for i in range(self.N)])
        self.latch_eq=self.id('equality','handle_latch')
        self.grasp_eq=self.id('equality','handle_grasp')
        self.dock_eq=self.id('equality','handle_dock')
        self.lift_q=int(model.jnt_qposadr[self.id('joint','latch_lift')])
        self.flex=self.id('flex','webbing')

    def id(self, kind, name):
        value=mujoco.mj_name2id(self.m,getattr(mujoco.mjtObj,'mjOBJ_'+kind.upper()),self.prefix+name)
        if value<0:raise ValueError('Missing '+self.prefix+name)
        return value

    def event(self, name):
        self.phase=name
        self.events.append({'time_s':float(self.d.time),'event':name,'handle_position_m':self.d.xpos[self.handle].tolist(),'stored_length_m':float(self.stored)})

    def begin_open(self, *, release_authorized, alarm=False, other=None):
        if alarm or not release_authorized:raise PermissionError('Opening requires authorization and cleared alarm')
        if other is not None and not bool(other.d.eq_active[other.latch_eq]):raise RuntimeError('Other passage must remain latched')
        self.open_authorized=True
        self.close_authorized=False

    def begin_close(self, *, occupied=False):
        if occupied:raise RuntimeError('Cannot extend belt through occupied passage')
        self.close_authorized=True

    def grasp(self):
        if np.linalg.norm(self.d.xpos[self.handle]-self.d.mocap_pos[self.gripper])>.004:raise RuntimeError('Gripper is not aligned with handle')
        if abs(float(np.dot(self.d.xquat[self.handle],self.d.mocap_quat[self.gripper])))<np.cos(np.deg2rad(5)/2):raise RuntimeError('Gripper orientation is not aligned')
        handle=self.id('geom','handle_grip')
        pairs=[{int(c.geom1),int(c.geom2)} for c in self.d.contact]
        if not all({handle,self.id('geom','gripper_pad_'+str(sign))} in pairs for sign in [-1,1]):raise RuntimeError('Both gripper pads must contact the handle')
        self.d.eq_active[self.grasp_eq]=1
        self.event('grasped')

    def close_gripper(self):
        for sign in [-1,1]:self.m.geom_pos[self.id('geom','gripper_pad_'+str(sign)),1]=sign*.013
        mujoco.mj_forward(self.m,self.d)

    def unlatch(self):
        if not self.open_authorized:raise PermissionError('Opening was not authorized')
        if not self.d.eq_active[self.grasp_eq]:raise RuntimeError('Hold the handle before unlatching')
        if self.d.qpos[self.lift_q]<.032:raise RuntimeError('Lift the handle at least 32 mm out of the receiver')
        self.d.eq_active[self.latch_eq]=0
        self.event('unlatched')

    def undock(self):
        if not self.close_authorized:raise PermissionError('Closing was not authorized')
        if not self.d.eq_active[self.grasp_eq]:raise RuntimeError('Hold the docked handle first')
        self.d.eq_active[self.dock_eq]=0
        self.event('undocked')

    def dock(self):
        if not self.d.eq_active[self.grasp_eq] or self.d.eq_active[self.latch_eq]:raise RuntimeError('Dock requires a held and unlatched handle')
        if np.linalg.norm(self.d.xpos[self.handle]-self.park)>.005:raise RuntimeError('Handle is not seated at reel parking fixture')
        self.d.eq_active[self.dock_eq]=1
        self.event('docked')

    def latch(self, *, occupied=False):
        if occupied or not self.close_authorized:raise RuntimeError('Passage must be clear and closing authorized')
        if not self.d.eq_active[self.grasp_eq] or self.d.eq_active[self.dock_eq]:raise RuntimeError('Latch requires a held, undocked handle')
        if np.linalg.norm(self.d.xpos[self.handle]-self.seat)>.004 or self.d.qpos[self.lift_q]>.004:raise RuntimeError('Handle is not seated in receiver')
        if abs(self.d.xquat[self.handle,0])<np.cos(np.deg2rad(5)/2):raise RuntimeError('Handle orientation is not seated')
        self.d.eq_active[self.latch_eq]=1
        self.open_authorized=False
        self.close_authorized=False
        self.event('latched')

    def release(self):
        if not (self.d.eq_active[self.latch_eq] or self.d.eq_active[self.dock_eq]):raise RuntimeError('Cannot release an unsecured handle')
        self.d.eq_active[self.grasp_eq]=0
        for sign in [-1,1]:self.m.geom_pos[self.id('geom','gripper_pad_'+str(sign)),1]=sign*.035
        self.event('released_at_latch' if self.d.eq_active[self.latch_eq] else 'released_at_dock')

    def feed_to(self, stored_length, dt):
        if not np.isfinite(stored_length) or not np.isfinite(dt) or dt<=0:raise ValueError('Finite length and positive timestep required')
        if not 0<=stored_length<=1.48:raise ValueError('Stored length outside modeled range')
        if not self.d.eq_active[self.grasp_eq] and abs(stored_length-self.stored)>1e-9:raise RuntimeError('Controlled feed requires a held handle')
        if self.d.eq_active[self.latch_eq] and stored_length>self.stored+1e-6:raise RuntimeError('Unlatch before retracting the belt')
        if self.d.eq_active[self.dock_eq] and abs(stored_length-self.stored)>1e-6:raise RuntimeError('Undock before feeding the belt')
        # Rate-limited boundary feed: all material vertices persist; none are hidden/deleted.
        self.stored+=float(np.clip(stored_length-self.stored,-.5*dt,.5*dt))
        s=np.arange(self.N)*self.DS-self.stored
        pitch=.0012/(2*np.pi)
        theta=-(np.sqrt(self.RADIUS**2+2*pitch*np.maximum(0,-s))-self.RADIUS)/pitch
        radius=self.RADIUS-pitch*theta
        positions=self.outlet+np.column_stack((radius*np.sin(theta),self.RADIUS-radius*np.cos(theta),np.zeros(self.N)))
        active=s<=0
        self.d.mocap_pos[self.guides[active]]=positions[active]
        self.d.eq_active[self.feed]=active[:,None]

    def contacts(self):
        result=[]
        handle_geoms={self.id('geom','handle_grip'),self.id('geom','handle_tongue')}
        for i,c in enumerate(self.d.contact):
            if self.flex not in c.flex and not handle_geoms.intersection((c.geom1,c.geom2)):continue
            force=np.zeros(6);mujoco.mj_contactForce(self.m,self.d,i,force)
            result.append({'time_s':float(self.d.time),'component':'webbing' if self.flex in c.flex else 'handle','position_m':c.pos.tolist(),'normal_force_N':float(abs(force[0])),'force_is_model_estimate':True})
        return result
