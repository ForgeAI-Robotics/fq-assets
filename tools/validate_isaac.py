"""Bounded Isaac Sim baseline load and PhysX startup smoke check."""
import faulthandler
faulthandler.dump_traceback_later(120,repeat=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480})
import json
from pathlib import Path
import omni.usd,omni.timeline
from pxr import UsdGeom,UsdPhysics
root=Path(__file__).resolve().parents[1]
report={'isaac_sim':'4.5.0','assets':[],'physics_calibrated':False}
try:
    ctx=omni.usd.get_context()
    for item in json.loads((root/'catalog/index.json').read_text())['assets']:
        a=json.loads((root/item['manifest']).read_text())
        print('FQ_LOADING',a['id'],flush=True)
        assert ctx.open_stage(str(root/a['representations']['usd']))
        for _ in range(15):app.update()
        s=ctx.get_stage(); assert s and s.GetDefaultPrim()
        count=sum(1 for p in s.Traverse() if p.IsA(UsdGeom.Gprim))
        assert count>0
        report['assets'].append({'id':a['id'],'stage_loaded':True,'geometry_prims':count})
    assert ctx.open_stage(str(root/'scenes/fq/office_baseline/scene.usda'))
    for _ in range(20):app.update()
    timeline=omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(60):app.update()
    report['baseline_timeline_time']=timeline.get_current_time()
    assert report['baseline_timeline_time']>0
    timeline.stop()
    report['baseline_startup_passed']=True
    report['scope']='All USD stages opened in Isaac; fixed baseline advanced 60 app updates. No articulated contact-policy validation.'
    (root/'validation/isaac_load.json').write_text(json.dumps(report,indent=2)+'\n')
    print('FQ_ISAAC_CHECK_PASSED',json.dumps(report))
finally:
    faulthandler.cancel_dump_traceback_later()
    app.close()
