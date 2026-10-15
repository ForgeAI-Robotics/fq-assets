"""Reapply the versioned native resistance profile to the refrigerator USD."""
from pathlib import Path
import importlib.util
from pxr import Usd,UsdPhysics
root=Path(__file__).resolve().parents[1]
folder=root/'assets/fq/haier_bcd402'
spec=importlib.util.spec_from_file_location('fridge_physx',folder/'source/physx_resistance.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
stage=Usd.Stage.Open(str(folder/'refrigerator.usda'));module.author(stage);stage.GetRootLayer().Save()
j=stage.GetPrimAtPath('/Refrigerator/Joints/refrigerator_right_hinge');drive=UsdPhysics.DriveAPI(j,'angular');assert drive.GetMaxForceAttr().Get()==0 and drive.GetDampingAttr().Get()==0
j.SetCustomDataByKey('fq:simulationFrictionlossNm',0.0)
j.SetCustomDataByKey('fq:simulationDampingNmSRad',0.0)
j.SetCustomDataByKey('fq:parameterStatus','simulation_parameter_not_measured_calibration')
stage.GetRootLayer().Save()
print('Authored zero right-door resistance; other drives unchanged')
