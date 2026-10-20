"""碰撞掩码修复的单元测试（纯文本层）。

背景：USD 关闭整机自碰撞时，Newton 会把所有可碰撞 shape 的 `contype/conaffinity`
算成 0 —— 那等于"连地面都不碰"。工具只把**真正的碰撞体**恢复成 `mask/mask`，
视觉体保持 0。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from usd2mjcf import mjcf_edit

BASE = """
<mujoco>
  <worldbody>
    <body name="base">
      <geom name="/Robot/base/collisions/base/mesh_0" type="mesh" contype="0" conaffinity="0"/>
      <geom name="/Robot/base/visuals/base/mesh/red" type="mesh" contype="0" conaffinity="0"/>
      <body name="arm">
        <geom name="/Robot/arm/collisions/arm/cylinder_5" type="cylinder" contype="0" conaffinity="0"/>
        <geom name="/Robot/arm/visuals/arm/mesh_1" type="mesh" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

# Newton 的 shape label（无去重后缀）
COLLIDERS = [
    "/Robot/base/collisions/base/mesh",
    "/Robot/arm/collisions/arm/cylinder",
]


def _geoms(root: ET.Element) -> dict[str, ET.Element]:
    return {g.get("name"): g for g in root.iter("geom")}


def test_restores_masks_for_colliders_only():
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.fix_zero_collision_masks(root, COLLIDERS)
    geoms = _geoms(root)

    # 碰撞体（带 _N 去重后缀）恢复成可碰撞
    assert geoms["/Robot/base/collisions/base/mesh_0"].get("contype") == "1"
    assert geoms["/Robot/base/collisions/base/mesh_0"].get("conaffinity") == "1"
    assert geoms["/Robot/arm/collisions/arm/cylinder_5"].get("contype") == "1"

    # 视觉体不能被误改
    assert geoms["/Robot/base/visuals/base/mesh/red"].get("contype") == "0"
    assert geoms["/Robot/arm/visuals/arm/mesh_1"].get("contype") == "0"

    assert len(stats["fixed"]) == 2
    assert stats["candidates"] == 2


def test_exact_name_also_matches():
    """没有去重后缀时也要能匹配。"""
    root = ET.fromstring(BASE.replace("mesh_0", "mesh").strip())
    stats = mjcf_edit.fix_zero_collision_masks(root, COLLIDERS)
    assert "/Robot/base/collisions/base/mesh" in stats["fixed"]


def test_nonzero_mask_left_alone():
    """已经可碰撞的不能动（W2 这种情况本来就没问题）。"""
    xml = BASE.replace(
        'name="/Robot/base/collisions/base/mesh_0" type="mesh" contype="0" conaffinity="0"',
        'name="/Robot/base/collisions/base/mesh_0" type="mesh" contype="4" conaffinity="4"',
    )
    root = ET.fromstring(xml.strip())
    stats = mjcf_edit.fix_zero_collision_masks(root, COLLIDERS)
    assert stats["fixed"] == ["/Robot/arm/collisions/arm/cylinder_5"]
    assert _geoms(root)["/Robot/base/collisions/base/mesh_0"].get("contype") == "4"


def test_no_colliders_is_noop():
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.fix_zero_collision_masks(root, [])
    assert stats == {"fixed": [], "candidates": 0}
    assert all(g.get("contype") == "0" for g in root.iter("geom"))


def test_custom_mask():
    root = ET.fromstring(BASE.strip())
    mjcf_edit.fix_zero_collision_masks(root, COLLIDERS, mask=3)
    assert _geoms(root)["/Robot/base/collisions/base/mesh_0"].get("contype") == "3"
