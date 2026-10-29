"""`usd.max_hull_vertices`：覆盖 USD 上作者授权的凸包顶点上限。

Newton 的 schema resolver 把**作者授权值**排在调用方默认值之前，所以只传
`add_usd(mesh_maxhullvert=...)` 会被资产里写死的
`newton:maxHullVertices` / `mjc:maxhullvert` /
`physxConvexHullCollision:hullVertexLimit` 压回去。这里验证 stage 级覆盖：

* 碰撞体上已授权的三个属性都被改写；
* 非碰撞（视觉）几何上即使同名字段存在也**不动**；
* 重复调用幂等（值已等于目标时不再记录改动）。
"""

from __future__ import annotations

import pytest

from usd2mjcf import usd_scene

pxr = pytest.importorskip("pxr")

HULL_ATTRS = {
    "mjc:maxhullvert": 32,
    "newton:maxHullVertices": 64,
    "physxConvexHullCollision:hullVertexLimit": 48,
}
TARGET = 512


def _stage_with_collider():
    Sdf, Usd, UsdGeom, UsdPhysics, _ = usd_scene.import_usd()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    collider = UsdGeom.Mesh.Define(stage, "/World/robot/base/collisions/mesh").GetPrim()
    UsdPhysics.CollisionAPI.Apply(collider)
    for name, value in HULL_ATTRS.items():
        attr = collider.CreateAttribute(name, Sdf.ValueTypeNames.Int, custom=True)
        attr.Set(value)

    visual = UsdGeom.Mesh.Define(stage, "/World/robot/base/visuals/mesh").GetPrim()
    attr = visual.CreateAttribute("newton:maxHullVertices", Sdf.ValueTypeNames.Int, custom=True)
    attr.Set(HULL_ATTRS["newton:maxHullVertices"])
    return stage, collider, visual


def test_overrides_authored_hull_limits():
    stage, collider, visual = _stage_with_collider()

    changes = usd_scene.override_max_hull_vertices(stage, TARGET)

    assert len(changes) == len(HULL_ATTRS)
    assert {c["attribute"] for c in changes} == set(HULL_ATTRS)
    for name, old in HULL_ATTRS.items():
        assert collider.GetAttribute(name).Get() == TARGET
        assert any(c["attribute"] == name and c["from"] == old for c in changes)

    # 视觉 prim 不受影响
    assert visual.GetAttribute("newton:maxHullVertices").Get() == HULL_ATTRS[
        "newton:maxHullVertices"
    ]


def test_override_is_idempotent_and_ignores_unauthored():
    stage, _collider, _visual = _stage_with_collider()
    usd_scene.override_max_hull_vertices(stage, TARGET)

    # 第二次调用：值已经是目标值，不再产生改动
    assert usd_scene.override_max_hull_vertices(stage, TARGET) == []

    # 没有授权上限的碰撞体不会被新增属性（由 add_usd.mesh_maxhullvert 兜底）
    Sdf, Usd, UsdGeom, UsdPhysics, _ = usd_scene.import_usd()
    extra = UsdGeom.Mesh.Define(stage, "/World/robot/base/collisions/plain").GetPrim()
    UsdPhysics.CollisionAPI.Apply(extra)
    assert usd_scene.override_max_hull_vertices(stage, TARGET) == []
    assert not extra.GetAttribute("newton:maxHullVertices").HasAuthoredValue()
