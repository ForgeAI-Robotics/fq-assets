"""Shared simulation-only G1 O7 startup. No DDS/CAN or hardware commands."""
from __future__ import annotations

import hashlib
import json
from collections import deque
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np


@lru_cache(maxsize=1)
def load_profile():
    profile = json.loads(Path(__file__).with_suffix('.json').read_text())
    for key, size in [('motion_token', 64), ('body_joint_names', 29),
                      ('seed_body_qpos', 29), ('standing_body_qpos', 29)]:
        if len(profile[key]) != size:
            raise ValueError(f'O7 startup: invalid {key}')
    return profile


def initial_body_qpos():
    return list(load_profile()['standing_body_qpos'])


def is_task_command(command):
    """Only an active body controller takes ownership from the startup hold."""
    if command is None:
        return False
    if command.type == 'wbc_torque':
        low = command['low_cmd']
        return low is not None and any(float(x.kp) > 1.0 for x in low.motor_cmd[:29])
    return command.type in {'decoupled_wbc', 'move_qpos', 'move_eef', 'joint_pos',
                            'joint_torque', 'loco_command'}


class StartupController:
    def __init__(self, model, data, decoder_path, profile=None, prefix=''):
        import onnxruntime as ort
        self.profile = profile or load_profile()
        p = self.profile
        path = Path(decoder_path)
        if hashlib.sha256(path.read_bytes()).hexdigest() != p['decoder_sha256']:
            raise ValueError('O7 startup requires the verified SONIC v1.1 decoder')
        self.model, self.data = model, data
        self.root_id = model.body(prefix + 'pelvis').id
        joints = [model.joint(prefix + n).id for n in p['body_joint_names']]
        self.qa = model.jnt_qposadr[joints]
        self.va = model.jnt_dofadr[joints]
        self.acts = np.array([next(a for a in range(model.nu)
                                  if model.actuator_trnid[a, 0] == j
                                  and model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT)
                              for j in joints])
        if not np.allclose(model.actuator_gear[self.acts, 0], 1):
            raise ValueError('O7 startup requires unit-gear body torque actuators')
        if np.any(model.actuator_biastype[self.acts] != 0):
            raise ValueError('O7 startup requires body torque actuators')
        self.to_mj = np.asarray(p['isaaclab_to_mujoco'], dtype=int)
        self.to_lab = np.asarray(p['mujoco_to_isaaclab'], dtype=int)
        self.default = np.asarray(p['default_angles'])
        self.kp, self.kd = np.asarray(p['kp']), np.asarray(p['kd'])
        self.scale = np.asarray(p['action_scale'])
        self.token = np.asarray(p['motion_token'], dtype=np.float32)
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = opts.inter_op_num_threads = 1
        self.decoder = ort.InferenceSession(str(path), opts, providers=['CPUExecutionProvider'])
        if self.decoder.get_inputs()[0].shape[-1] != 994:
            raise ValueError('SONIC startup observation dimension must be 994')
        self.history = deque(maxlen=10)
        self.last = np.zeros(29)
        self.next_control_time = float(data.time)
        self.target = self.default.copy()
        self.hand_actuators = []
        for a in range(model.nu):
            if model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT:
                continue
            j = int(model.actuator_trnid[a, 0])
            if '_o7_' in (model.joint(j).name or ''):
                qa, va = model.jnt_qposadr[j], model.jnt_dofadr[j]
                self.hand_actuators.append((a, qa, va, float(data.qpos[qa])))

    def _clip(self, values, actuators):
        m = self.model
        limited = m.actuator_ctrllimited[actuators].astype(bool)
        bounds = m.actuator_ctrlrange[actuators]
        return np.where(limited, np.clip(values, bounds[:, 0], bounds[:, 1]), values)

    def apply(self):
        m, d = self.model, self.data
        if d.time + 1e-9 >= self.next_control_time:
            mujoco.mj_forward(m, d)
            root = d.body(self.root_id)
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.root_id, velocity, 1)
            entry = (velocity[:3].copy(), (d.qpos[self.qa] - self.default)[self.to_lab].copy(),
                     d.qvel[self.va][self.to_lab].copy(), self.last.copy(),
                     root.xmat.reshape(3, 3).T @ np.array([0., 0., -1.]))
            if not self.history:
                self.history.extend([entry] * 10)
            else:
                self.history.append(entry)
            obs = np.concatenate([self.token] + [np.array([h[k] for h in self.history]).ravel()
                                                 for k in range(5)]).astype(np.float32)
            raw = self.decoder.run(None, {'obs_dict': obs[None]})[0][0]
            if not np.isfinite(raw).all():
                raise RuntimeError('Nonfinite O7 startup decoder output')
            self.target = self.default + raw[self.to_mj] * self.scale
            self.last = raw.copy()
            self.next_control_time = float(d.time) + self.profile['control_dt']
        d.xfrc_applied[self.root_id] = 0
        d.ctrl[self.acts] = self._clip(self.kp * (self.target - d.qpos[self.qa])
                                      - self.kd * d.qvel[self.va], self.acts)
        for a, qa, va, target in self.hand_actuators:
            d.ctrl[a] = self._clip(np.array([.4 * (target - d.qpos[qa]) - .04 * d.qvel[va]]),
                                   np.array([a]))[0]


def initialize(model, data, decoder_path, *, profile=None, prefix='', ground_z=None):
    """Seed free robots then settle under real physics; fixed robots use saved standing angles.

    Preserves task spawn XY/yaw. Warmup is outside the episode clock. Returns a
    live controller that must continue holding until the task takes ownership.
    """
    p = profile or load_profile()
    root = model.body(prefix + 'pelvis').id
    free = [j for j in range(model.njnt) if model.jnt_bodyid[j] == root
            and model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    qa = [model.jnt_qposadr[model.joint(prefix + n).id] for n in p['body_joint_names']]
    va = [model.jnt_dofadr[model.joint(prefix + n).id] for n in p['body_joint_names']]
    if not free or np.linalg.norm(model.opt.gravity) < 1:
        data.qpos[qa] = p['standing_body_qpos']
        data.qvel[va] = 0
        mujoco.mj_forward(model, data)
        return None, {'mode': 'standing_pose', 'ready': True, 'dynamic_warmup': False}
    if np.any(model.eq_type == mujoco.mjtEq.mjEQ_WELD):
        for e in range(model.neq):
            if model.eq_type[e] == mujoco.mjtEq.mjEQ_WELD and data.eq_active[e] and root in model.eq_obj1id[e:e+1].tolist() + model.eq_obj2id[e:e+1].tolist():
                raise RuntimeError('O7 startup cannot validate a welded pelvis')
    rq, rv = int(model.jnt_qposadr[free[0]]), int(model.jnt_dofadr[free[0]])
    data.qpos[qa] = p['seed_body_qpos']
    data.qvel[va] = 0
    data.qvel[rv:rv + 6] = 0
    mujoco.mj_forward(model, data)
    R = data.body(root).xmat.reshape(3, 3)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    data.qpos[rq+3:rq+7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
    mujoco.mj_forward(model, data)
    if ground_z is None:
        planes = [float(data.geom_xpos[g, 2]) for g in range(model.ngeom)
                  if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE
                  and data.geom_xmat[g].reshape(3, 3)[2, 2] > .999]
        if not planes:
            raise RuntimeError('O7 startup needs an explicit ground_z for a scene without a horizontal floor plane')
        ground_z = max(planes)
    low = float('inf')
    for g in range(model.ngeom):
        if not (model.body(model.geom_bodyid[g]).name or '').startswith(prefix):
            continue
        if 'ankle' not in (model.body(model.geom_bodyid[g]).name or ''):
            continue
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = model.geom_dataid[g]
            vertices = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]]
            low = min(low, float((vertices @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g])[:, 2].min()))
    if not np.isfinite(low):
        raise RuntimeError('O7 startup could not locate foot geometry')
    data.qpos[rq+2] += ground_z + .003 - low
    mujoco.mj_forward(model, data)
    controller = StartupController(model, data, decoder_path, p, prefix)
    start_time = float(data.time)
    stable = 0.
    max_tilt = 0.
    start_xy = data.qpos[rq:rq+2].copy()
    while data.time - start_time < p['maximum_warmup_seconds']:
        controller.apply()
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        tilt = float(np.degrees(np.arccos(np.clip(data.body(root).xmat.reshape(3, 3)[2, 2], -1, 1))))
        max_tilt = max(max_tilt, tilt)
        if not np.isfinite(data.qpos).all() or data.qpos[rq+2] < ground_z + .45 or tilt > 45:
            raise RuntimeError(f'O7 initial token failed to stand: tilt={tilt:.1f}deg')
        speed = float(np.linalg.norm(data.qvel[rv:rv+3]))
        stable = stable + model.opt.timestep if tilt < p['max_ready_tilt_deg'] and speed < p['max_ready_base_speed'] else 0.
        if data.time-start_time >= p['minimum_warmup_seconds'] and stable >= p['stable_window_seconds']:
            break
    else:
        raise RuntimeError('O7 initial token did not stabilize before the startup timeout')
    report = {'mode': 'sonic_initial_token', 'ready': True, 'dynamic_warmup': True,
              'warmup_seconds': float(data.time-start_time), 'max_tilt_deg': max_tilt,
              'final_tilt_deg': tilt, 'horizontal_drift_m': float(np.linalg.norm(data.qpos[rq:rq+2]-start_xy))}
    data.time = start_time
    controller.next_control_time = start_time
    return controller, report
