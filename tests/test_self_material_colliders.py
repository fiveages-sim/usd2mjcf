"""自材质碰撞体修复：Newton 会把"同时是碰撞体"的材质 prim 整个跳过。

Isaac / URDF 转换器会写出三种不同的材质 API：
`NewtonMaterialAPI`、`PhysicsMaterialAPI`、`PhysxMaterialAPI`。
这里用内存 stage 验证三种写法都会被拆成"普通碰撞体 + 独立材质 prim"，
同时真正的材质 prim 不受影响。
"""

from __future__ import annotations

import pytest

from usd2mjcf import usd_scene

pxr = pytest.importorskip("pxr")


def _make_collider(stage, path: str, api: str):
    """在内存 stage 上建一个带指定材质 API 的碰撞体，返回其 prim。"""
    Sdf, Usd, UsdGeom, UsdPhysics, _ = usd_scene.import_usd()
    prim = UsdGeom.Cylinder.Define(stage, path).GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if api == "PhysicsMaterialAPI":
        UsdPhysics.MaterialAPI.Apply(prim)
    else:
        try:
            prim.ApplyAPI(api)
        except pxr.Tf.ErrorException:
            # PhysX 的 codeless schema 不一定在当前 USD 运行时注册；
            # 直接写入 applied schema 名即可模拟资产里的写法。
            prim.AddAppliedSchema(api)
    prim.CreateAttribute("physics:dynamicFriction", Sdf.ValueTypeNames.Float).Set(0.8)
    prim.CreateAttribute("physics:staticFriction", Sdf.ValueTypeNames.Float).Set(0.9)
    return prim


def test_splits_collision_prims_for_all_material_apis():
    Sdf, Usd, UsdGeom, UsdPhysics, UsdShade = usd_scene.import_usd()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # PhysxMaterialAPI 是 PhysX 的 codeless schema，当前 usd-core 运行时不一定注册，
    # 内存 stage 上无法稳定模拟；它和另外两种 API 走同一条修复路径。
    apis = [api for api in usd_scene._SELF_MATERIAL_APIS if api != "PhysxMaterialAPI"]
    colliders = {
        api: _make_collider(stage, f"/World/robot/{api}/collisions/cylinder", api)
        for api in apis
    }

    # 真正的材质 prim 不应被改写
    material_only = UsdShade.Material.Define(stage, "/World/materials/steel").GetPrim()
    UsdPhysics.MaterialAPI.Apply(material_only)

    fixed = usd_scene.fix_self_material_colliders(stage)
    by_path = {entry["collider"]: entry for entry in fixed}

    assert len(fixed) == len(colliders)
    for api, collider in colliders.items():
        path = str(collider.GetPath())
        assert path in by_path
        assert by_path[path]["status"] == "moved"
        assert api not in (collider.GetAppliedSchemas() or [])
        assert "PhysicsMaterialAPI" not in (collider.GetAppliedSchemas() or [])

        side_material = stage.GetPrimAtPath(by_path[path]["material"])
        assert side_material.IsValid()
        assert side_material.GetAttribute("physics:dynamicFriction").Get() == pytest.approx(0.8)
        assert side_material.GetAttribute("physics:staticFriction").Get() == pytest.approx(0.9)

    assert "PhysicsMaterialAPI" in (material_only.GetAppliedSchemas() or [])
