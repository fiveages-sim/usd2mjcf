"""执行器 ctrlrange 修正的单元测试（纯文本层）。"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from usd2mjcf import mjcf_edit

DEG = 180.0 / 3.141592653589793


def _root(xml: str) -> ET.Element:
    return ET.fromstring(xml.strip())


BASE = """
<mujoco>
  <worldbody>
    <body name="b">
      <joint name="deg_joint" range="-1.35263 0.567232" limited="true"/>
      <joint name="rad_joint" range="-3.14159 3.14159" limited="true"/>
      <joint name="slide_joint" type="slide" range="0 0.04" limited="true"/>
    </body>
  </worldbody>
  <actuator>
    <general joint="deg_joint" biastype="affine" gainprm="5000" biasprm="0 -5000 -500"
             ctrlrange="-77.49999 32.5" ctrllimited="true"/>
    <general joint="rad_joint" biastype="affine" gainprm="625" biasprm="0 -625 -80"
             ctrlrange="-3.14159 3.14159" ctrllimited="true"/>
    <general joint="slide_joint" biastype="affine" gainprm="1" biasprm="0 -1 -0.2"
             ctrllimited="true"/>
    <velocity joint="rad_joint" kv="20" ctrlrange="-15 15"/>
  </actuator>
</mujoco>
"""


def test_degree_ctrlrange_converted_to_radian():
    """跨度比 ≈ 180/π → 判定为「度」，换算成弧度。"""
    root = _root(BASE)
    stats = mjcf_edit.fix_actuator_ctrlrange(root)
    actuators = list(root.find("actuator"))
    lo, hi = (float(v) for v in actuators[0].get("ctrlrange").split())
    assert lo == pytest.approx(-1.35263, abs=1e-4)
    assert hi == pytest.approx(0.567232, abs=1e-4)
    assert len(stats["converted"]) == 1
    assert "deg_joint" in stats["converted"][0]


def test_radian_ctrlrange_left_alone():
    """本来就是弧度的不能动（否则会把 ±π 变成 ±0.055）。"""
    root = _root(BASE)
    mjcf_edit.fix_actuator_ctrlrange(root)
    assert list(root.find("actuator"))[1].get("ctrlrange") == "-3.14159 3.14159"


def test_missing_ctrlrange_filled_from_joint():
    """ctrllimited=true 但没写 ctrlrange（夹爪就是这种）→ 用关节限位补。"""
    root = _root(BASE)
    stats = mjcf_edit.fix_actuator_ctrlrange(root)
    slide = list(root.find("actuator"))[2]
    assert slide.get("ctrlrange") == "0 0.04"
    assert any("slide_joint" in line for line in stats["filled"])


def test_velocity_actuator_not_touched():
    """速度伺服的控制量是角速度，不该被关节限位限幅。"""
    root = _root(BASE)
    mjcf_edit.fix_actuator_ctrlrange(root)
    velocity = list(root.find("actuator"))[3]
    assert velocity.get("ctrlrange") == "-15 15"


def test_auto_degrees_can_be_disabled():
    root = _root(BASE)
    mjcf_edit.fix_actuator_ctrlrange(root, auto_convert_degrees=False)
    assert list(root.find("actuator"))[0].get("ctrlrange") == "-77.49999 32.5"


def test_equality_joint_not_treated_as_kinematic():
    """<equality><joint> 与运动学 <joint> 同名，不能互相污染。"""
    xml = """
    <mujoco>
      <worldbody><body name="b"><joint name="j" range="-1 1" limited="true"/></body></worldbody>
      <equality><joint joint1="j" joint2="j"/></equality>
      <actuator>
        <general joint="j" biastype="affine" gainprm="1" biasprm="0 -1 0" ctrllimited="true"/>
      </actuator>
    </mujoco>
    """
    root = _root(xml)
    stats = mjcf_edit.fix_actuator_ctrlrange(root)
    assert list(root.find("actuator"))[0].get("ctrlrange") == "-1 1"
    assert len(stats["filled"]) == 1
    # equality 那条不该被当成运动学关节参与统计
    assert mjcf_edit.summarize(root)["joints"] == 1
