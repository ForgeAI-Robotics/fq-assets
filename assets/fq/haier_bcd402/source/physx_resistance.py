"""Author passive PhysX 4.5 drives; units are Nm and degrees in USD.

Raw USD contains fixed dry resistance. SIMPLE PhysXFridge supplies the
per-step viscous cap and episode randomization; never use dimensionless
physxJoint:jointFriction as a torque.
"""
import math,json
from pathlib import Path
from pxr import UsdPhysics,Sdf
SLOPE=10000.0

def author(stage, root='/Refrigerator', friction=None):
    if friction is None:friction=float(json.loads((Path(__file__).resolve().parent.parent/'simulation_resistance.json').read_text())['frictionloss_Nm'])
    p=stage.GetPrimAtPath(root)
    p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    base=stage.GetPrimAtPath(root+'/Joints/fixed_base')
    UsdPhysics.ArticulationRootAPI.Apply(base)
    base.AddAppliedSchema('PhysxArticulationAPI')
    base.CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(True)
    for side in ('left','right','mullion'):
        joint=stage.GetPrimAtPath(root+'/Joints/refrigerator_'+side+'_hinge')
        joint.AddAppliedSchema('PhysxJointAPI')
        joint.CreateAttribute('physxJoint:jointFriction',Sdf.ValueTypeNames.Float).Set(0.)
        joint.CreateAttribute('physxJoint:armature',Sdf.ValueTypeNames.Float).Set(.002)
        joint.AddAppliedSchema('PhysxLimitAPI:angular')
        joint.CreateAttribute('physxLimit:angular:contactDistance',Sdf.ValueTypeNames.Float).Set(.01)
        if side!='mullion':UsdPhysics.Joint(joint).CreateCollisionEnabledAttr(True)
        drive=UsdPhysics.DriveAPI.Apply(joint,'angular')
        drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0);drive.CreateTargetVelocityAttr(0)
        drive.CreateDampingAttr(((SLOPE if friction else 0.) if side=='right' else (.15 if side=='left' else .01))*math.pi/180)
        drive.CreateMaxForceAttr(friction if side=='right' else 1e6)
    j=stage.GetPrimAtPath(root+'/Joints/refrigerator_right_hinge')
    j.SetCustomDataByKey('fq:resistanceRuntimeSupported',True)
    j.SetCustomDataByKey('fq:resistanceRuntime','SIMPLE PhysXFridge: CPU Isaac Sim 4.5; zero-velocity capped drive')
    j.SetCustomDataByKey('fq:resistanceAdapterRequired',True)
