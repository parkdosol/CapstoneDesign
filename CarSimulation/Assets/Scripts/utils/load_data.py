"""
목표 : 파라미터 로드 유틸리티 함수 제공
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-25
"""

import os
import json

import mujoco

def _read_file_binary(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.read()

def _read_file_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

# JSON 파일에서 파라미터 로드 함수
def load_json(json_path=None):
    if json_path is None: # 기본 경로는 physics_param.json 
        json_path = os.path.join(os.path.dirname(__file__), 'physics_param.json')
    with open(json_path, 'r') as f:
        params = json.load(f)
    return params['car_params'], params['force_params'], params['suspension_params'] 

# MuJoCo 모델 로드 함수 - params 이용해 xml 치환 
def load_model(params):
    base_dir = os.path.dirname(__file__)
    xml_path = os.path.join(base_dir, params["CAR_MODEL"])
    stl_path = os.path.join(base_dir, params["STL_NAME"])

    # STL asset dictionary 생성
    stl_name = params["STL_NAME"]
    stl_dict = {stl_name: _read_file_binary(stl_path)}

    # XML 템플릿 파싱 및 파라미터 치환 
    xml_template = _read_file_text(xml_path)
    try: 
        xml_filled = xml_template.format(**params)
        model = mujoco.MjModel.from_xml_string(xml_filled, asset_dict)
        data = mujoco.MjData(model)
        return model, data
    except Exception as e:
        raise RuntimeError(f"MuJoCo 모델 로드 실패: {e}")

# ────────────────────────────────────────────────────────────────────────────
# 차체 ID 캐싱 함수
def get_chassis_id(model: mujoco.MjModel):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

# 바퀴·조향·모터 ID 캐싱 함수
def get_wheel_ids(model: mujoco.MjModel):
    wheel_defs = [
        ("FL", "front_left_wheel",  "fl_wheel", "fl_steer"),
        ("FR", "front_right_wheel", "fr_wheel", "fr_steer"),
        ("RL", "rear_left_wheel",   "rl_wheel", None),
        ("RR", "rear_right_wheel",  "rr_wheel", None),
    ]

    ids: list[tuple[str,int,int,int|None,int]] = []
    for label, body, wj, sj in wheel_defs:
            bid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,  body)                    # body id
            jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, wj)                      # wheel joint id
            sjid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, sj) if sj else None      # steer joint id
            aid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, wj.replace("_wheel", "_motor")) # motor id
            ids.append((label, bid, jid, sjid, aid))
    
    return ids
