import os
import random
import xml.etree.ElementTree as ET

def insert_stl_to_mjcf(
    base_xml_path="models/base_scene.xml",
    output_xml_path="models/generated_scene.xml",
    n_bump=5,                    # 소환할 bump 개수
    x_start=5.0,                 # 첫 bump 시작점
    min_gap=5.0,                 # 최소 거리
    max_gap=15.0,                # 최대 거리
    base_pos_x=0.0,
    base_pos_y=0.0,
    base_pos_z=0,
    random_seed=None
):
    if random_seed is not None:
        random.seed(random_seed)

    stl_dir = "models/speedbumps"
    stl_files = sorted([
        fname for fname in os.listdir(stl_dir)
        if fname.endswith(".stl")
    ])
    if not stl_files:
        raise RuntimeError("No STL files found in speedbumps folder!")

    # bump 개수와 STL 개수 맞추기 (모자라면 처음부터 재사용)
    stl_list = [os.path.join(stl_dir, stl_files[i % len(stl_files)]) for i in range(n_bump)]

    tree = ET.parse(base_xml_path)
    root = tree.getroot()

    asset_tag = root.find("asset")
    if asset_tag is None:
        print("⚠️  <asset> 태그가 없어서 자동 생성합니다.")
        asset_tag = ET.SubElement(root, "asset")
    worldbody_tag = root.find("worldbody")
    if worldbody_tag is None:
        print("⚠️  <worldbody> 태그가 없어서 자동 생성합니다.")
        worldbody_tag = ET.SubElement(root, "worldbody")

    x = x_start
    for i, path in enumerate(stl_list):
        fname = os.path.basename(path)
        mesh_name = os.path.splitext(fname)[0]
        rel_path = os.path.relpath(path, os.path.dirname(output_xml_path))

        # asset 등록 (중복 방지)
        if not any(m.get("name") == mesh_name for m in asset_tag.findall("mesh")):
            ET.SubElement(asset_tag, "mesh", name=mesh_name, file=rel_path)

        pos_str = f"{x:.3f} {base_pos_y:.3f} {base_pos_z:.3f}"
        ET.SubElement(worldbody_tag, "geom",
                      type="mesh", mesh=mesh_name,
                      pos=pos_str,
                      euler="0 0 -90",
                      rgba="0.9 0.5 0.2 1", contype="1", conaffinity="1")
        if i < n_bump - 1:
            gap = random.uniform(min_gap, max_gap)
            x += gap

    os.makedirs(os.path.dirname(output_xml_path), exist_ok=True)
    tree.write(output_xml_path)
    print(f"✅ 랜덤 gap으로 {n_bump}개 bump 소환 완료 → {output_xml_path}")

if __name__ == "__main__":
    insert_stl_to_mjcf(
        n_bump=5,          # 원하는 bump 개수
        x_start=5.0,       # 시작점
        min_gap=6.0,       # 최소 거리 (원하는 범위)
        max_gap=13.0,      # 최대 거리
        random_seed=42     # (옵션) 실행마다 같게 하려면
    )
