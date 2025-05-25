# racecar_model.py
"""
목적   : 브레이크 기능 포함 차량 시뮬 루프
작성자 : 김준호
최초 작성일자 : 2025‑05‑22
수정자 : 김준호
최종 수정일자 : 2025‑05‑25
"""

import os
import mujoco

# 1. 파라미터 입력 (직접 입력 또는 함수 인자 등)
PARAMS = {
    # 차량 모델 관련
    "CAR_MODEL": "closed_truck_ell.xml",     # xml
    "STL_NAME" : "closed_truck.stl",      # stl 실행전 두 파일 변경 체크
    # 시뮬레이션 환경 설정
    "TIMESTEP": 0.005,
    "GRAVITY_Z": -9.81,
    "INTEGRATOR": "RK4",
    # 차량 물리적 파라미터 설정 (closed truck 기준, 대형트럭)
    # 실제 트럭 타이어규격 300/70R20 = 두게300mm, 지름718mm
    "MASS_CHASSIS": 2500,
    "MASS_SUSP": 300,
    "INERTIA_SUSP": 50,
    "MASS_WHEEL": 100,
    "WHEEL_DIAMETER": 0.718,                                 # 바퀴반지름            
    "WHEEL_WIDTH": 0.300,                                    # 바퀴두께
    "SPRING_K_F": 300_000,
    "SPRING_K_R": 300_000,
    "DAMP_SUSP": 10_000,
    "GEAR_DRIVE": 6,
    "GEAR_STEER": 40,
    "GEAR_BRAKE": 1,
}

def load_model(params=PARAMS):
    """XML 파일을 읽고 파라미터를 치환하여 MuJoCo 모델 생성"""
    xml_path = os.path.join(os.path.dirname(__file__), params["CAR_MODEL"])
    stl_path = os.path.join(os.path.dirname(__file__), params["STL_NAME"])
    asset_dict = {}
    with open(stl_path, "rb") as f:
        asset_dict[params["STL_NAME"]] = f.read()
    with open(xml_path, "r", encoding="utf-8") as f:
        xml_template = f.read()
    xml_filled = xml_template.format(**params)
    model = mujoco.MjModel.from_xml_string(xml_filled, asset_dict)
    data = mujoco.MjData(model)
    return model, data

# 사용 예시
# model, data = load_model()