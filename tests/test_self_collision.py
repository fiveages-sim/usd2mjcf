"""整机自碰撞策略：把粗粒度的"整机自碰撞关闭"改成父子过滤。

资产常用 `newton:selfCollisionEnabled = 0` 或
`physxArticulation:enabledSelfCollisions = 0` 一次性关闭整机自碰撞。
转换器默认在 Newton 解析前把开关改成开启，让 MuJoCo 的 `filterparent`
只过滤父子 body 对，其余 body 对正常碰撞。
"""

from __future__ import annotations

import pytest

from usd2mjcf import usd_scene

pxr = pytest.importorskip("pxr")


def test_enables_all_authored_switches():
    Sdf, Usd, UsdGeom, _, _ = usd_scene.import_usd()
    stage = Usd.Stage.CreateInMemory()

    root = UsdGeom.Xform.Define(stage, "/World/root").GetPrim()
    newton_attr = root.CreateAttribute(
        "newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, custom=True
    )
    newton_attr.Set(False)

    component = UsdGeom.Xform.Define(stage, "/World/root/component").GetPrim()
    physx_attr = component.CreateAttribute(
        "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, custom=True
    )
    physx_attr.Set(False)

    already_on = UsdGeom.Xform.Define(stage, "/World/root/on").GetPrim()
    on_attr = already_on.CreateAttribute(
        "newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, custom=True
    )
    on_attr.Set(True)

    changed = usd_scene.apply_self_collision_mode(stage, "parent_filtered")

    assert {entry["attribute"] for entry in changed} == {
        "newton:selfCollisionEnabled",
        "physxArticulation:enabledSelfCollisions",
    }
    assert newton_attr.Get() is True
    assert physx_attr.Get() is True
    assert on_attr.Get() is True


def test_only_shallowest_articulation_root_is_changed():
    """整机和部件各有一层 root 时，只覆盖整机那一层。"""
    Sdf, Usd, UsdGeom, UsdPhysics, _ = usd_scene.import_usd()
    stage = Usd.Stage.CreateInMemory()

    root = UsdGeom.Xform.Define(stage, "/World/robot").GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root_attr = root.CreateAttribute(
        "newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, custom=True
    )
    root_attr.Set(False)

    arm = UsdGeom.Xform.Define(stage, "/World/robot/arm/root_joint").GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(arm)
    arm_attr = arm.CreateAttribute(
        "newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, custom=True
    )
    arm_attr.Set(False)

    changed = usd_scene.apply_self_collision_mode(stage, "parent_filtered")

    assert [entry["prim"] for entry in changed] == ["/World/robot"]
    assert root_attr.Get() is True
    assert arm_attr.Get() is False


def test_respect_keeps_authored_value():
    Sdf, Usd, UsdGeom, _, _ = usd_scene.import_usd()
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, "/World/root").GetPrim()
    attr = prim.CreateAttribute(
        "newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, custom=True
    )
    attr.Set(False)

    assert usd_scene.apply_self_collision_mode(stage, "respect") == []
    assert attr.Get() is False


def test_invalid_mode_raises():
    Sdf, Usd, UsdGeom, _, _ = usd_scene.import_usd()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/root")
    with pytest.raises(ValueError):
        usd_scene.apply_self_collision_mode(stage, "nope")
