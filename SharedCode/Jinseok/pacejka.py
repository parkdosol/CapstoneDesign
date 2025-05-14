# pacejka.py
"""
목적 : Pacejka 타이어 모델 적용
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-03
"""

import numpy as np
import mujoco

def magic_formula(slip, Fz, B, C, E, mu, max_D=None):
    D = mu * Fz
    if max_D is not None:
        D = np.minimum(D, max_D)
    Bs = B * slip
    return D * np.sin(C * np.arctan(Bs - E * (Bs - np.arctan(Bs))))

class Pacejka:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.wheels = [
            ('front_left_wheel',  'fl_wheel', 'fl_steer'),
            ('front_right_wheel', 'fr_wheel', 'fr_steer'),
            ('rear_left_wheel',   'rl_wheel', None),
            ('rear_right_wheel',  'rr_wheel', None),
        ]
        self.ids = []
        for body, wj, sj in self.wheels:
            bid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     body)
            jid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    wj)
            sjid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, sj) if sj else None
            aid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, wj.replace('_wheel', '_motor'))
            self.ids.append((body, bid, jid, sjid, aid))
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
        self.m      = float(model.body_mass[cid])
        self.h_cog  = float(model.body_pos[cid, 2])
        self.wbase  = 2.5
        self.track  = 3.0
        self.Bx, self.Cx, self.Ex, self.mu_x = 12.0, 1.9, 0.97, 0.9
        self.By, self.Cy, self.Ey, self.mu_y = 8.5, 1.3, 0.97, 1.0
        self.max_load = 3000.0
        self.max_Fx   = self.mu_x * self.max_load
        self.max_Fy   = self.mu_y * self.max_load
        self.r        = 0.35
        self.damp     = 0.1

    def apply(self, data: mujoco.MjData):
        ax, ay = float(data.qacc[0]), float(data.qacc[1])
        Fz_stat = self.m * 9.81 / 4.0
        for body, bid, jid, sjid, aid in self.ids:
            data.xfrc_applied[bid] = 0
            data.qfrc_applied[jid] = 0
            front = 'front' in body
            left  = 'left'  in body
            dF_long = self.m * ax * self.h_cog / self.wbase
            dF_lat  = self.m * ay * self.h_cog / self.track
            Fz_calc = Fz_stat + 0.5*(dF_long*(1 if front else -1) + dF_lat*(1 if left else -1))
            Fz = float(np.clip(Fz_calc, 10.0, self.max_load))
            vel6 = np.zeros(6, float)
            mujoco.mj_objectVelocity(self.model, data,
                                     mujoco.mjtObj.mjOBJ_BODY, bid,
                                     vel6, 1)
            vx, vy = float(vel6[3]), float(vel6[4])
            omega = float(data.qvel[jid])
            vw    = omega * self.r
            vn    = max(abs(vx), 0.5)
            ctrl_torque = float(data.ctrl[aid])
            if abs(omega) < 1e-3 and abs(ctrl_torque) > 10.0:
                kappa = np.sign(ctrl_torque) * 0.1
            else:
                kappa = float(np.clip((vw - vx) / vn, -0.2, 0.2))
            yaw       = float(data.qvel[5])
            dy        = (self.wbase/2 if front else -self.wbase/2)
            steer_ang = float(data.qpos[sjid]) if sjid is not None else 0.0
            beta      = float(np.clip(np.arctan2(vy + dy*yaw, vn) - steer_ang, -0.26, 0.26))
            Fx = float(np.clip(magic_formula(kappa, Fz, self.Bx, self.Cx, self.Ex, self.mu_x, self.max_Fx), -self.max_Fx, self.max_Fx))
            Fy = float(np.clip(magic_formula(beta,  Fz, self.By, self.Cy, self.Ey, self.mu_y, self.max_Fy), -self.max_Fy, self.max_Fy))
            data.xfrc_applied[bid, 0] = Fx
            data.xfrc_applied[bid, 1] = Fy
            torque = np.sign(omega) * Fx * self.r + self.damp * omega
            data.qfrc_applied[jid]   -= torque

def apply_pacejka_forces(model, data):
    inst = getattr(apply_pacejka_forces, '_inst', None)
    if inst is None or inst.model is not model:
        inst = Pacejka(model)
        apply_pacejka_forces._inst = inst
    inst.apply(data)
