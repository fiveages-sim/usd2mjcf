"""视觉体 / 碰撞体分组（group）的单元测试。"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from usd2mjcf import mjcf_edit

BASE = """
<mujoco>
  <worldbody>
    <body name="base">
      <geom name="/Robot/base/collisions/base/mesh" type="mesh" contype="1" conaffinity="1"/>
      <geom name="/Robot/base/visuals/base/mesh/red" type="mesh" contype="0" conaffinity="0" material="m"/>
      <geom name="/Robot/base/collisions/dead/mesh" type="mesh" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <asset><material name="m" rgba="1 0 0 1"/></asset>
</mujoco>
"""


def _geoms(root: ET.Element) -> dict[str, ET.Element]:
    return {g.get("name"): g for g in root.iter("geom")}


def test_colliders_go_to_group3_and_visuals_to_group0():
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.separate_visual_collision(root)
    geoms = _geoms(root)

    assert geoms["/Robot/base/collisions/base/mesh"].get("group") == "3"
    # 掩码 0/0 的几何不产生接触，即使名字带 /collisions/ 也按视觉体处理
    assert geoms["/Robot/base/collisions/dead/mesh"].get("group") is None
    assert geoms["/Robot/base/visuals/base/mesh/red"].get("group") is None

    assert stats["colliders"] == 1
    assert stats["visuals"] == 2


def test_collider_without_material_gets_neutral_rgba():
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.separate_visual_collision(root)
    geoms = _geoms(root)
    assert geoms["/Robot/base/collisions/base/mesh"].get("rgba") is not None
    # 有材质的视觉体，以及 0/0 的视觉几何，都不该被改颜色
    assert geoms["/Robot/base/visuals/base/mesh/red"].get("rgba") is None
    assert geoms["/Robot/base/collisions/dead/mesh"].get("rgba") is None
    assert stats["recolored"] == 1


def test_disabled_keeps_everything_visible():
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.separate_visual_collision(root, hide_colliders=False)
    assert all(g.get("group") is None for g in root.iter("geom"))
    assert stats["hidden"] is False


def test_visual_with_wrong_group_is_reset():
    """视觉体如果被之前步骤写进 group=3，这里要拉回 0，否则它会莫名其妙消失。"""
    root = ET.fromstring(BASE.strip())
    _geoms(root)["/Robot/base/visuals/base/mesh/red"].set("group", "3")
    mjcf_edit.separate_visual_collision(root)
    assert _geoms(root)["/Robot/base/visuals/base/mesh/red"].get("group") == "0"


def test_companion_geoms_not_confused():
    """配对几何（conttype/conaffinity 非零）也算碰撞体。"""
    xml = BASE.replace(
        'name="/Robot/base/collisions/dead/mesh" type="mesh" contype="0" conaffinity="0"',
        'name="/Robot/base/collisions/dead/mesh" type="mesh" contype="0" conaffinity="5"',
    )
    root = ET.fromstring(xml.strip())
    mjcf_edit.separate_visual_collision(root)
    assert _geoms(root)["/Robot/base/collisions/dead/mesh"].get("group") == "3"
