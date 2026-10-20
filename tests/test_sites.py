"""`mjcf.sites`（补充 TCP / EEF 等 `<site>`）的单元测试。"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from usd2mjcf import config as config_mod
from usd2mjcf import mjcf_edit

BASE = """
<mujoco>
  <worldbody>
    <body name="base">
      <body name="left_link7">
        <geom name="left_link7_vis" type="sphere" size="0.05"/>
        <body name="left_gripper_gripper_base">
          <geom name="gripper_vis" type="sphere" size="0.02"/>
        </body>
      </body>
      <body name="right_link7">
        <geom name="right_link7_vis" type="sphere" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

SITE = {
    "name": "LEFT_TCP_SITE",
    "body": "left_link7",
    "pos": [0, -0.095, 0],
    "quat": [0.5, 0.5, -0.5, 0.5],
    "size": 0.005,
    "rgba": [0.2, 0.9, 0.2, 0.5],
}


def _root(xml: str = BASE) -> ET.Element:
    return ET.fromstring(xml.strip())


def _body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    raise AssertionError(f"缺少 body {name}")


def test_adds_site_with_attributes():
    root = _root()
    stats = mjcf_edit.add_sites(root, [SITE])
    site = _body(root, "left_link7").find("site")
    assert site is not None
    assert site.get("name") == "LEFT_TCP_SITE"
    assert site.get("pos") == "0 -0.095 0"
    assert site.get("quat") == "0.5 0.5 -0.5 0.5"
    assert site.get("size") == "0.005"
    assert site.get("rgba") == "0.2 0.9 0.2 0.5"
    assert stats["added"] == ["LEFT_TCP_SITE @ left_link7"]


def test_site_is_inserted_before_child_bodies():
    """site 放在父 body 自有元素之后、子 body 之前，与常见 MJCF 排版一致。"""
    root = _root()
    mjcf_edit.add_sites(root, [SITE])
    tags = [child.tag for child in _body(root, "left_link7")]
    assert tags.index("site") < tags.index("body")


def test_body_glob_matches():
    root = _root()
    mjcf_edit.add_sites(root, [{**SITE, "body": "left_*"}])
    assert _body(root, "left_link7").find("site") is not None


def test_existing_site_is_skipped():
    """重复运行必须幂等：产物里已有同名 site 就不再加。"""
    root = _root()
    mjcf_edit.add_sites(root, [SITE])
    stats = mjcf_edit.add_sites(root, [SITE])
    assert stats["added"] == []
    assert stats["skipped"] == ["LEFT_TCP_SITE"]
    assert len(list(root.iter("site"))) == 1


def test_unknown_body_is_reported():
    root = _root()
    notes: list[str] = []
    stats = mjcf_edit.add_sites(root, [{**SITE, "body": "nope*"}], notes=notes)
    assert stats["unknownBodies"] == ["LEFT_TCP_SITE @ nope*"]
    assert any("不存在" in line for line in notes)


def test_optional_missing_body_is_quiet():
    """`optional: true` 用于随 variant 消失的部件（例如 EE=None 时的夹爪 base）。"""
    root = _root()
    notes: list[str] = []
    stats = mjcf_edit.add_sites(
        root,
        [{**SITE, "name": "LEFT_EEF_SITE", "body": "missing_gripper_*", "optional": True}],
        notes=notes,
    )
    assert stats["unknownBodies"] == []
    assert not any("⚠️" in line for line in notes)
    assert list(root.iter("site")) == []


def test_ambiguous_body_warns_and_uses_first():
    root = _root()
    notes: list[str] = []
    stats = mjcf_edit.add_sites(root, [{**SITE, "body": "*_link7"}], notes=notes)
    assert stats["added"] == ["LEFT_TCP_SITE @ left_link7"]
    assert stats["ambiguous"]
    assert _body(root, "left_link7").find("site") is not None
    assert _body(root, "right_link7").find("site") is None
    assert any("匹配到 2 个 body" in line for line in notes)


def test_strict_raises_on_unknown_body():
    root = _root()
    with pytest.raises(LookupError):
        mjcf_edit.add_sites(root, [{**SITE, "body": "nope"}], strict=True)


def test_strict_raises_on_ambiguous_body():
    root = _root()
    with pytest.raises(LookupError):
        mjcf_edit.add_sites(root, [{**SITE, "body": "*_link7"}], strict=True)


def test_missing_body_or_name_raises():
    root = _root()
    with pytest.raises(ValueError):
        mjcf_edit.add_sites(root, [{"name": "X"}])
    with pytest.raises(ValueError):
        mjcf_edit.add_sites(root, [{"body": "left_link7"}])


def test_comment_keys_are_ignored():
    root = _root()
    mjcf_edit.add_sites(root, [{**SITE, "_comment": "注释"}])
    site = _body(root, "left_link7").find("site")
    assert site is not None
    assert site.get("_comment") is None


def test_config_validation_catches_unknown_key():
    cfg = config_mod.from_dict(
        {"mjcf": {"sites": [{"name": "X", "body": "b", "rpga": [1, 1, 1, 1]}]}}
    )
    problems = getattr(cfg, "_problems", [])
    assert any("rpga" in problem for problem in problems)


def test_config_validation_catches_short_vector():
    cfg = config_mod.from_dict(
        {"mjcf": {"sites": [{"name": "X", "body": "b", "quat": [1, 0, 0]}]}}
    )
    problems = getattr(cfg, "_problems", [])
    assert any("quat" in problem for problem in problems)


def test_config_validation_accepts_scalar_size():
    cfg = config_mod.from_dict({"mjcf": {"sites": [SITE]}})
    assert getattr(cfg, "_problems", []) == []
