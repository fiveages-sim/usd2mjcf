"""geom 统一命名（`mjcf.geom_names`）的单元测试。

命名规则：`VISUAL_<body>_<序号>` / `COLLISION_<body>_<序号>`，序号按同一个
body 和同一类几何各自从 0 开始。mesh 名字与 `mesh=` 引用必须保持不变。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from usd2mjcf import mjcf_edit

BASE = """
<mujoco>
  <default>
    <geom rgba="0.5 0.5 0.5 1"/>
  </default>
  <worldbody>
    <geom name="/floor" type="plane" contype="1" conaffinity="1"/>
    <body name="base_footprint">
      <geom name="/Robot/base_footprint/collisions/base/mesh" type="mesh"
            contype="1" conaffinity="1" mesh="/m_base"/>
      <geom name="/Robot/base_footprint/visuals/base/mesh/red" type="mesh"
            contype="0" conaffinity="0" mesh="/m_red"/>
      <geom name="/Robot/base_footprint/collisions/dead/mesh" type="mesh"
            contype="0" conaffinity="0"/>
    </body>
    <body name="arm">
      <geom name="/Robot/arm/visuals/mesh" type="mesh" contype="0" conaffinity="0"/>
      <geom name="/Robot/arm/collisions/mesh" type="mesh" contype="2" conaffinity="2"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="/Robot/base_footprint/collisions/base/mesh"
          geom2="/Robot/arm/collisions/mesh"/>
  </contact>
  <asset>
    <mesh name="/m_base" file="base.obj"/>
    <mesh name="/m_red" file="red.obj"/>
  </asset>
</mujoco>
"""


def _root() -> ET.Element:
    return ET.fromstring(BASE.strip())


def test_names_follow_body_and_kind():
    root = _root()
    stats = mjcf_edit.normalize_geom_names(root)
    names = [geom.get("name") for geom in root.iter("geom") if geom.get("name")]

    assert names == [
        "COLLISION_world_0",
        "COLLISION_base_footprint_0",
        "VISUAL_base_footprint_0",
        "COLLISION_base_footprint_1",
        "VISUAL_arm_0",
        "COLLISION_arm_0",
    ]
    assert stats == {"mode": "kind", "visual": 2, "collision": 4, "renamed": 6}


def test_mesh_names_and_references_unchanged():
    root = _root()
    mjcf_edit.normalize_geom_names(root)
    meshes = {mesh.get("name") for mesh in root.iter("mesh")}
    assert meshes == {"/m_base", "/m_red"}

    mesh_refs = {
        geom.get("name"): geom.get("mesh")
        for geom in root.iter("geom")
        if geom.get("name")
    }
    assert mesh_refs["COLLISION_base_footprint_0"] == "/m_base"
    assert mesh_refs["VISUAL_base_footprint_0"] == "/m_red"
    assert "COLLISION_arm_0" in mesh_refs


def test_contact_pair_references_are_rewritten():
    root = _root()
    mjcf_edit.normalize_geom_names(root)
    pair = root.find("contact/pair")
    assert pair is not None
    assert pair.get("geom1") == "COLLISION_base_footprint_0"
    assert pair.get("geom2") == "COLLISION_arm_0"


def test_default_geom_template_is_not_touched():
    root = _root()
    mjcf_edit.normalize_geom_names(root)
    default_geom = root.find("default/geom")
    assert default_geom is not None
    assert default_geom.get("name") is None


def test_usd_mode_keeps_names():
    root = _root()
    before = [geom.get("name") for geom in root.iter("geom")]
    stats = mjcf_edit.normalize_geom_names(root, mode="usd")

    assert [geom.get("name") for geom in root.iter("geom")] == before
    assert stats["mode"] == "usd"
    assert stats["renamed"] == 0
    assert stats["visual"] == 2
    assert stats["collision"] == 4


def test_collider_ids_take_priority_over_name_heuristic():
    """掩码为 0 且名字被改过时，仍能靠分组阶段记下的碰撞体集合正确分类。"""
    root = ET.fromstring(
        """
        <mujoco><worldbody>
          <body name="base_link">
            <geom name="mesh_1" type="mesh" contype="0" conaffinity="0"/>
          </body>
        </worldbody></mujoco>
        """.strip()
    )
    geom = root.find("worldbody/body/geom")
    assert geom is not None
    assert mjcf_edit.collision_geom_ids(root) == set()

    stats = mjcf_edit.normalize_geom_names(root, collider_ids={id(geom)})

    assert geom.get("name") == "COLLISION_base_link_0"
    assert stats["collision"] == 1
    assert stats["visual"] == 0


def test_geom_added_after_snapshot_uses_current_masks():
    """补丁新加的几何不在快照里，按当前掩码判断。"""
    root = _root()
    collider_ids = mjcf_edit.collision_geom_ids(root)
    body = root.find("worldbody/body[@name='arm']")
    assert body is not None
    added = ET.SubElement(
        body, "geom", {"name": "extra", "type": "box", "contype": "1", "conaffinity": "1"}
    )

    mjcf_edit.normalize_geom_names(root, collider_ids=collider_ids)

    assert added.get("name") == "COLLISION_arm_1"
