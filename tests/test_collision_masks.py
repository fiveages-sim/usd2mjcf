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


NORMALIZE_BASE = """
<mujoco>
  <worldbody>
    <body name="base">
      <geom name="base_col" contype="2" conaffinity="-3"/>
      <body name="arm">
        <geom name="arm_col" contype="4" conaffinity="-5"/>
      </body>
    </body>
    <body name="other">
      <geom name="other_col" contype="8" conaffinity="-9"/>
    </body>
  </worldbody>
</mujoco>
"""


def test_normalizes_equivalent_masks_to_one():
    """位掩码只是冗余编码时，统一写成 1/1。"""
    root = ET.fromstring(NORMALIZE_BASE.strip())
    stats = mjcf_edit.normalize_collision_masks(root)

    assert stats["normalized"] is True
    assert stats["geoms"] == 3
    assert _geoms(root)["base_col"].get("contype") == "1"
    assert _geoms(root)["base_col"].get("conaffinity") == "1"
    assert _geoms(root)["other_col"].get("contype") == "1"


PARTIAL_FILTER = """
<mujoco>
  <worldbody>
    <body name="a">
      <geom name="a1" contype="1" conaffinity="1"/>
      <geom name="a2" contype="1" conaffinity="0"/>
    </body>
    <body name="b">
      <geom name="b1" contype="1" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""


def test_keeps_masks_when_shape_filter_changes_graph():
    """只有部分 geom 组合允许接触时，统一成 1/1 会改变语义，必须保留原掩码。"""
    root = ET.fromstring(PARTIAL_FILTER.strip())
    stats = mjcf_edit.normalize_collision_masks(root)

    assert stats["normalized"] is False
    assert stats["mismatches"] == [{"body1": "a", "body2": "b", "allowed": 1, "total": 2}]
    assert _geoms(root)["a2"].get("conaffinity") == "0"


def test_excluded_body_pair_does_not_block_normalization():
    """已经在 <contact><exclude> 里的 body 对不再影响掩码统一。"""
    xml = PARTIAL_FILTER.replace(
        "</worldbody>",
        "</worldbody><contact><exclude body1=\"a\" body2=\"b\"/></contact>",
    )
    root = ET.fromstring(xml.strip())
    stats = mjcf_edit.normalize_collision_masks(root)

    assert stats["normalized"] is True
    assert _geoms(root)["a2"].get("conaffinity") == "1"


PARENT_CHILD_FILTER = """
<mujoco>
  <worldbody>
    <body name="a">
      <geom name="a1" contype="1" conaffinity="1"/>
      <geom name="a2" contype="1" conaffinity="0"/>
      <body name="b">
        <geom name="b1" contype="1" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def test_parent_child_pair_is_ignored_when_filterparent_enabled():
    """父子对由 filterparent 处理，不参与掩码等价性比较。"""
    root = ET.fromstring(PARENT_CHILD_FILTER.strip())
    stats = mjcf_edit.normalize_collision_masks(root)

    assert stats["normalized"] is True
    assert _geoms(root)["a2"].get("conaffinity") == "1"


def test_parent_child_pair_blocks_normalization_when_filterparent_disabled():
    """filterparent 关闭时，父子对的形状级过滤必须保留。"""
    xml = PARENT_CHILD_FILTER.replace(
        "<mujoco>",
        '<mujoco><option><flag filterparent="false"/></option>',
    )
    root = ET.fromstring(xml.strip())
    stats = mjcf_edit.normalize_collision_masks(root)

    assert stats["normalized"] is False
    assert stats["mismatches"] == [{"body1": "a", "body2": "b", "allowed": 1, "total": 2}]
