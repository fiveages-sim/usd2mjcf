"""`contact.excludes`（额外碰撞排除）的单元测试。"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from usd2mjcf import mjcf_edit

BASE = """
<mujoco>
  <worldbody>
    <body name="chassis">
      <body name="waist"><joint name="j1"/></body>
      <body name="wheel_fl"><joint name="j2"/></body>
      <body name="wheel_fr"><joint name="j3"/></body>
      <body name="steer_fl"><joint name="j4"/></body>
    </body>
  </worldbody>
  <contact>
    <exclude body1="chassis" body2="steer_fl"/>
  </contact>
</mujoco>
"""


def _excludes(root: ET.Element) -> set[frozenset[str]]:
    contact = root.find("contact")
    if contact is None:
        return set()
    return {
        frozenset((e.get("body1"), e.get("body2")))
        for e in contact
        if e.tag == "exclude"
    }


def test_adds_glob_pairs():
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.add_contact_excludes(root, [["chassis", "*wheel*"]])
    pairs = _excludes(root)
    assert frozenset(("chassis", "wheel_fl")) in pairs
    assert frozenset(("chassis", "wheel_fr")) in pairs
    assert len(stats["added"]) == 2


def test_skips_existing_pairs():
    """USD/Newton 已经生成的排除不能重复添加。"""
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.add_contact_excludes(root, [["chassis", "steer_fl"]])
    assert stats["added"] == []
    assert stats["skipped"] == ["chassis × steer_fl"]
    assert len(_excludes(root)) == 1


def test_direction_does_not_matter():
    """exclude 是无向的，反向写也不该重复。"""
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.add_contact_excludes(root, [["steer_fl", "chassis"]])
    assert stats["added"] == []
    assert stats["skipped"]


def test_unknown_body_is_reported():
    root = ET.fromstring(BASE.strip())
    notes: list[str] = []
    stats = mjcf_edit.add_contact_excludes(root, [["chassis", "nope*"]], notes=notes)
    assert stats["unknown"] == ["nope*"]
    assert any("不存在" in line for line in notes)


def test_strict_raises_on_unknown():
    root = ET.fromstring(BASE.strip())
    with pytest.raises(LookupError):
        mjcf_edit.add_contact_excludes(root, [["chassis", "nope"]], strict=True)


def test_self_pair_ignored():
    """同一个 body 与自己不需要 exclude。"""
    root = ET.fromstring(BASE.strip())
    stats = mjcf_edit.add_contact_excludes(root, [["chassis", "chassis"]])
    assert stats["added"] == []


def test_creates_contact_node_when_absent():
    xml = """
    <mujoco>
      <worldbody><body name="a"/><body name="b"/></worldbody>
    </mujoco>
    """
    root = ET.fromstring(xml.strip())
    assert root.find("contact") is None
    mjcf_edit.add_contact_excludes(root, [["a", "b"]])
    assert root.find("contact") is not None
    assert frozenset(("a", "b")) in _excludes(root)


def test_invalid_pair_shape_raises():
    root = ET.fromstring(BASE.strip())
    with pytest.raises(ValueError):
        mjcf_edit.add_contact_excludes(root, [["only-one"]])


PRUNE_BASE = """
<mujoco>
  <option><flag filterparent="true"/></option>
  <worldbody>
    <body name="torso">
      <geom name="torso_col" contype="1" conaffinity="1"/>
      <body name="shoulder">
        <geom name="shoulder_col" contype="1" conaffinity="1"/>
        <body name="elbow">
          <geom name="elbow_col" contype="1" conaffinity="1"/>
        </body>
      </body>
      <body name="visual_only">
        <geom name="visual" contype="0" conaffinity="0"/>
      </body>
    </body>
    <body name="default_col">
      <geom name="default_geom"/>
    </body>
  </worldbody>
  <contact>
    <exclude body1="torso" body2="shoulder"/>
    <exclude body1="torso" body2="elbow"/>
    <exclude body1="torso" body2="visual_only"/>
    <exclude body1="torso" body2="default_col"/>
    <exclude body1="shoulder" body2="elbow"/>
  </contact>
</mujoco>
"""


def test_prunes_parent_child_and_vacuous_excludes():
    root = ET.fromstring(PRUNE_BASE.strip())
    stats = mjcf_edit.prune_redundant_excludes(root)

    # torso-shoulder、shoulder-elbow 是父子；torso-visual_only 一方没有碰撞体。
    assert stats["parent"] == 2
    assert stats["vacuous"] == 1
    assert stats["kept"] == 2
    assert _excludes(root) == {
        frozenset(("torso", "elbow")),
        frozenset(("torso", "default_col")),
    }


def test_prune_keeps_parent_excludes_when_filterparent_disabled():
    root = ET.fromstring(PRUNE_BASE.strip())
    root.find("option/flag").set("filterparent", "false")
    stats = mjcf_edit.prune_redundant_excludes(root)

    assert stats["parent"] == 0
    assert stats["vacuous"] == 1
    assert stats["kept"] == 4


def test_prune_without_contact_node():
    root = ET.fromstring("<mujoco><worldbody/></mujoco>")
    stats = mjcf_edit.prune_redundant_excludes(root)
    assert stats == {"removed": 0, "parent": 0, "vacuous": 0, "kept": 0}

