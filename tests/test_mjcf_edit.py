"""纯文本层的单元测试：不依赖 GPU / Newton / USD。"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from usd2mjcf import mjcf_edit


def _root(xml: str) -> ET.Element:
    return ET.fromstring(xml.strip())


BASE = """
<mujoco>
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="euler"/>
  <worldbody>
    <body name="base">
      <joint name="root" type="free"/>
      <geom name="base_vis" type="sphere" size="0.1" contype="0" conaffinity="0"/>
      <geom name="base_col" type="sphere" size="0.1"/>
      <body name="wheel_l">
        <joint name="_Robot_wheel_joint" axis="0 1 0"/>
        <geom name="wheel_l_col" type="cylinder" size="0.07 0.02"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general joint="_Robot_wheel_joint" gainprm="100" biasprm="0 0 -100" biastype="affine"
             forcerange="-1000 1000" ctrlrange="-3 3"/>
    <general joint="root" gainprm="5" biasprm="0 -5 -1"/>
  </actuator>
</mujoco>
"""


def test_scene_settings_applied():
    root = _root(BASE)
    notes: list[str] = []
    mjcf_edit.apply_scene_settings(
        root, timestep=0.005, iterations=42, gravity=(0, 0, -9, ), flags={"refsafe": False}, notes=notes
    )
    option = root.find("option")
    assert option is not None
    assert option.get("timestep") == "0.005"
    assert option.get("iterations") == "42"
    assert option.get("gravity") == "0 0 -9"
    flag = option.find("flag")
    assert flag is not None and flag.get("refsafe") == "disable"
    assert any("timestep" in line for line in notes)


def test_actuator_type_conversion_drops_general_attrs():
    root = _root(BASE)
    mjcf_edit.apply_patch(
        root,
        {
            "actuators": [
                {
                    "match": {"joint": "*_wheel_joint"},
                    "type": "velocity",
                    "kv": 20,
                    "forcerange": [-100, 100],
                }
            ]
        },
    )
    actuators = list(root.find("actuator"))
    assert [a.tag for a in actuators] == ["velocity", "general"]
    wheel = actuators[0]
    assert wheel.get("kv") == "20"
    assert wheel.get("forcerange") == "-100 100"
    # 缩写形式不接受 general 专有属性
    for attr in mjcf_edit._GENERAL_ONLY_ATTRS:
        assert wheel.get(attr) is None


def test_unmatched_rule_is_reported():
    root = _root(BASE)
    notes: list[str] = []
    mjcf_edit.apply_patch(root, {"joints": [{"match": {"name": "*nope*"}, "damping": 1}]}, notes=notes)
    assert any("未命中" in line for line in notes)


def test_strict_patch_raises():
    root = _root(BASE)
    with pytest.raises(LookupError):
        mjcf_edit.apply_patch(
            root, {"joints": [{"match": {"name": "*nope*"}, "damping": 1}]}, strict=True
        )


def test_rename_updates_references():
    root = _root(BASE)
    mjcf_edit.apply_patch(
        root,
        {"rename": {"joint": {"pattern": "^_Robot_", "replacement": ""}}},
    )
    names = {j.get("name") for j in root.iter("joint")}
    assert "_Robot_wheel_joint" not in names
    actuator = root.find("actuator")[0]
    assert actuator.get("joint") in names


def test_rename_conflict_raises():
    """重命名后撞上已有名字 → 必须报错；静默会产生两个同名 joint。"""
    conflict = BASE.replace(
        '<body name="wheel_l">',
        '<body name="wheel_r"><joint name="wheel_joint" axis="0 1 0"/></body>\n'
        '      <body name="wheel_l">',
    )
    root = _root(conflict)
    with pytest.raises(ValueError):
        mjcf_edit.apply_patch(
            root,
            {"rename": {"joint": {"pattern": "^_Robot_", "replacement": ""}}},
        )


def test_container_targets():
    """<actuator> 是容器，规则必须命中它的子元素而不是容器本身。"""
    root = _root(BASE)
    targets = mjcf_edit._iter_targets(root, "actuator")
    assert [e.tag for e in targets] == ["general", "general"]


def test_summary_counts():
    stats = mjcf_edit.summarize(_root(BASE))
    assert stats["joints"] == 2
    assert stats["actuators"] == 2
    assert stats["geoms"] == 3
