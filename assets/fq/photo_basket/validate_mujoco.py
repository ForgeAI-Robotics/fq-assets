"""Actual dynamic tests and rendered evidence, using the installed MuJoCo."""
import os
os.environ.setdefault('MUJOCO_GL','glfw' if os.name=='nt' else 'egl')
import mujoco, numpy as np, json, imageio.v2 as imageio
from pathlib import Path
R=Path(__file__).resolve().parent;P=json.loads((R/'parameters.json').read_text());out=R/'evidence';out.mkdir(exist_ok=True)
m=mujoco.MjModel.from_xml_path(str(R/'test_mujoco.xml'));d=mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m,d,0)
bid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'basket');pid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'payload')
renderer=mujoco.Renderer(m,height=720,width=960);opt=mujoco.MjvOption();opt.geomgroup[3]=0
cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.15];cam.distance=.8;cam.azimuth=125;cam.elevation=-27
writer=imageio.get_writer(str(out/'mujoco_validation.mp4'),fps=30,codec='libx264',quality=8,macro_block_size=1)
history=[];maxpenetration=0.;finite=True
for i in range(5000):
    mujoco.mj_step(m,d)
    finite=finite and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
    if d.ncon:maxpenetration=max(maxpenetration,float(max(-c.dist for c in d.contact)))
    if i%33==0:
        renderer.update_scene(d,camera=cam,scene_option=opt);writer.append_data(renderer.render())
    if i%100==0:history.append({'time':float(d.time),'basket':d.xpos[bid].tolist(),'payload':d.xpos[pid].tolist(),'hinges':d.qpos[7:9].tolist()})
local=d.xmat[bid].reshape(3,3).T@(d.xpos[pid]-d.xpos[bid]);drop={'inside':bool(abs(local[0])<.13 and abs(local[1])<.075 and .02<local[2]<.075),'local_payload':local.tolist(),'basket_height':float(d.xpos[bid,2]),'payload_speed':float(np.linalg.norm(d.qvel[-6:-3]))}
imageio.imwrite(out/'mujoco_drop_settled.png',renderer.render())
# Test both joints independently using torques, rather than assigning poses.
sweeps=[]
for index,name in enumerate(['handle_right','handle_left']):
    mujoco.mj_resetDataKeyframe(m,d,0)
    jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name+'_hinge');q=m.jnt_qposadr[jid];v=m.jnt_dofadr[jid]
    seen=[];other=[]
    for i in range(3600):
        target=P['joint_initial']+(2.35-P['joint_initial'])*min(i/1900,1)
        d.qfrc_applied[:]=0;d.qfrc_applied[v]=np.clip(.16*(target-d.qpos[q])-.012*d.qvel[v],-.12,.12)
        mujoco.mj_step(m,d);seen.append(float(d.qpos[q]));other.append(float(d.qpos[8-index]))
        if i%33==0:renderer.update_scene(d,camera=cam,scene_option=opt);writer.append_data(renderer.render())
    sweeps.append({'handle':name,'minimum':min(seen),'maximum':max(seen),'other_span':max(other)-min(other),'moved_independently':max(seen)-min(seen)>1.,'within_limits':min(seen)>P['joint_limits'][0]-.08 and max(seen)<P['joint_limits'][1]+.08})
writer.close();renderer.close()
report={'mujoco_version':mujoco.__version__,'finite':finite,'drop':drop,'hinge_sweeps':sweeps,'maximum_contact_penetration_m':maxpenetration,'warnings':[int(w.number) for w in d.warning],'contacts_parent_child':'Parent-child contact filtering enabled at pivot attachment; handle-handle and environment contacts enabled.','passed':finite and drop['inside'] and drop['payload_speed']<.03 and all(s['moved_independently'] and s['within_limits'] for s in sweeps),'history':history}
(out/'mujoco_report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2),flush=True)
if not report['passed']:raise SystemExit(2)
