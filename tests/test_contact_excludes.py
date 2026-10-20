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
