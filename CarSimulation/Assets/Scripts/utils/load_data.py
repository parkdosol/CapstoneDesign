"""
목표 : 파라미터 로드 유틸리티 함수 제공
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-14

참고 : physics_param.json 등 파라미터 로드
"""

import json
import os

# JSON 파일에서 파라미터 로드 함수

def load_params_from_json(json_path=None):
    if json_path is None:
        # 현재 파일 기준 상대경로
        json_path = os.path.join(os.path.dirname(__file__), 'physics_param.json')
    with open(json_path, 'r') as f:
        params = json.load(f)
    return params['car_params'], params['force_params'], params['suspension_params'] 