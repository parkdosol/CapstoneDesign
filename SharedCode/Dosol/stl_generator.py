import numpy as np
from stl import mesh
import random
import os

def format_float_for_filename(value: float) -> str:
    return f"{value:.3f}".replace('.', '_')

def generate_elliptical_cylinder_stl(
    a_range=(2.5, 3.5),
    b_range=(0.03, 0.045),
    segments= 80,
    save_dir="models/speedbumps"
) -> dict:
    os.makedirs(save_dir, exist_ok=True)

    # 랜덤 파라미터
    a = round(random.uniform(*a_range), 3)  # 장반경 (y)
    b = round(random.uniform(*b_range), 3)  # 단반경 (z)
    h = 5.0  # x축 방향 길이

    # 파일명 구성
    a_str = format_float_for_filename(a)
    b_str = format_float_for_filename(b)
    h_str = format_float_for_filename(h)
    filename = f"speedbump-{a_str}-{b_str}-{h_str}.stl"
    path = os.path.join(save_dir, filename)

    # 타원형 단면을 YZ 평면에 생성하고, X축 방향으로 길이 생성
    theta = np.linspace(0, 2 * np.pi, segments)
    y = a * np.cos(theta)
    z = b * np.sin(theta)
    x_top = np.full_like(y, h / 2)
    x_bottom = np.full_like(y, -h / 2)

    top = np.stack([x_top, y, z], axis=1)
    bottom = np.stack([x_bottom, y, z], axis=1)

    # 삼각형 면 정의
    faces = []
    for i in range(segments - 1):
        p1, p2 = bottom[i], bottom[i + 1]
        p3, p4 = top[i], top[i + 1]
        faces.extend([[p1, p2, p3], [p2, p4, p3]])

    center_top = np.array([h / 2, 0, 0])
    center_bottom = np.array([-h / 2, 0, 0])
    for i in range(segments - 1):
        faces.append([center_top, top[i], top[i + 1]])
        faces.append([center_bottom, bottom[i + 1], bottom[i]])

    # STL 저장
    data = np.zeros(len(faces), dtype=mesh.Mesh.dtype)
    for i, f in enumerate(faces):
        data['vectors'][i] = np.array(f)
    m = mesh.Mesh(data)
    m.save(path)

    print(f"✅ STL 저장: {path}")

    return {
        "filename": filename,
        "filepath": path,
        "a": a,
        "b": b,
        "h": h
    }

def generate_sample():
    result = generate_elliptical_cylinder_stl()
    print(f"📦 생성 정보: {result}")

if __name__ == "__main__":
    generate_sample()
