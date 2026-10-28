"""Runtime controls for rigid belt proxies and a constraint-held sampling strip.

No cloth mechanics, motor calibration or chemical detector response is modeled.
Caller supplies occupancy and release authorization from its own policy/sensors.
"""
import math
import numpy as np

class BeltControl:
    def __init__(self, apply, duration=1.2):
        self.apply=apply;self.duration=duration
        self.fraction={'entry':0.,'exit':0.};self.target=dict(self.fraction)
        for name in self.fraction:self.apply(name,0.)
    def command(self,name,opened,*,occupied=False,release_authorized=False,alarm=False):
        if name not in self.fraction:raise KeyError(name)
        if opened and (alarm or (name=='exit' and not release_authorized)):
            raise PermissionError('Opening requires cleared alarm and exit release authorization')
        other='exit' if name=='entry' else 'entry'
        if opened and (self.fraction[other]>1e-9 or self.target[other]>0):raise RuntimeError('Entry and exit cannot be open together')
        if not opened and occupied:raise RuntimeError('Cannot close an occupied passage')
        self.target[name]=1. if opened else 0.
    def step(self,dt,occupied=None):
        if not math.isfinite(dt) or dt<0:raise ValueError('dt must be finite and nonnegative')
        occupied=occupied or {}
        for name,x in self.fraction.items():
            if self.target[name]<x and occupied.get(name,False):self.target[name]=1.
            delta=np.clip(self.target[name]-x,-dt/self.duration,dt/self.duration)
            self.fraction[name]=float(np.clip(x+delta,0,1));self.apply(name,self.fraction[name])
    def state(self,name):
        x=self.fraction[name]
        return 'open' if x>=1-1e-9 else 'closed' if x<=1e-9 else 'opening' if self.target[name]>x else 'closing'

def mujoco_belts(model):
    import mujoco
    records={}
    for n,g in [('entry','EntryBelt'),('exit','ReleaseBelt')]:
        i=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,g)
        if i<0:raise ValueError('Missing '+g)
        records[n]=(i,model.geom_pos[i].copy(),model.geom_size[i].copy(),model.geom_rgba[i].copy(),int(model.geom_contype[i]),int(model.geom_conaffinity[i]))
    def apply(n,f):
        i,p,s,c,ct,ca=records[n];opened=f>=1-1e-9
        model.geom_size[i,0]=max(.001,s[0]*(1-f));model.geom_pos[i]=p;model.geom_pos[i,0]=p[0]-s[0]+model.geom_size[i,0]
        model.geom_rgba[i]=c;model.geom_rgba[i,3]=0 if opened else c[3]
        model.geom_contype[i]=0 if opened else ct;model.geom_conaffinity[i]=0 if opened else ca
    return BeltControl(apply)

def usd_belts(stage,config):
    from pxr import UsdGeom,UsdPhysics,Gf
    records={}
    for n in ['entry','exit']:
        p=stage.GetPrimAtPath(config[n]['usd_path']);x=UsdGeom.Xformable(p)
        if not p:raise ValueError('Missing belt prim')
        x.ClearXformOpOrder();t=x.AddTranslateOp(opSuffix='runtime');s=x.AddScaleOp(opSuffix='runtime');records[n]=(p,t,s)
    def apply(n,f):
        p,t,s=records[n];pos=list(config[n]['center']);half=max(.001,.8*(1-f));pos[0]=-.8+half
        t.Set(Gf.Vec3d(*pos));s.Set(Gf.Vec3f(half,.007,.0325));opened=f>=1-1e-9
        UsdGeom.Imageable(p).GetVisibilityAttr().Set('invisible' if opened else 'inherited')
        UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(not opened)
    return BeltControl(apply)

class SwabControl:
    """MuJoCo grasp latch, requiring physical proximity before enabling weld."""
    def __init__(self,m,d):
        import mujoco
        self.m=m;self.d=d;self.state='fresh';self.used=False
        self.body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'sampling_swab')
        self.tool=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'gripper_interface')
        self.eq=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,'swab_grasp')
        self.grip=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'swab_grip')
        self.tip=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'swab_tip')
        if min(self.body,self.tool,self.eq,self.grip,self.tip)<0:raise ValueError('Incomplete operational scene')
    def grasp(self):
        if self.used or self.state!='fresh':raise RuntimeError('Sample is not fresh or already held')
        distance=np.linalg.norm(self.d.site_xpos[self.grip]-self.d.xpos[self.tool])
        alignment=np.linalg.norm(self.d.xmat[self.body]-self.d.xmat[self.tool])
        if distance>.004 or alignment>.05:raise RuntimeError('Gripper not aligned with sample grip region')
        import mujoco
        handle=mujoco.mj_name2id(self.m,mujoco.mjtObj.mjOBJ_GEOM,'swab_handle')
        pads=[mujoco.mj_name2id(self.m,mujoco.mjtObj.mjOBJ_GEOM,'gripper_pad_'+n) for n in ['left','right']]
        pairs=[{c.geom1,c.geom2} for c in self.d.contact]
        if not all({handle,pad} in pairs for pad in pads):raise RuntimeError('Both gripper pads must contact the handle before latching')
        self.d.eq_active[self.eq]=1;self.state='held'
    def accept_insertion(self):
        p=self.d.site_xpos[self.tip];axis=self.d.xmat[self.body].reshape(3,3)[:,0]
        if self.state!='held' or not bool(self.d.eq_active[self.eq]):raise RuntimeError('Sample is not held')
        if not (1.433<=p[0]<=1.54 and abs(p[1]-1.58)<.009 and abs(p[2]-.8825)<.008 and axis[0]>.995):raise RuntimeError('Sample has not entered the slot in the permitted pose')
        self.state='inserted';self.used=True
    def release(self):
        if self.state not in ['held','inserted']:raise RuntimeError('Nothing held')
        self.d.eq_active[self.eq]=0;self.state='released'
