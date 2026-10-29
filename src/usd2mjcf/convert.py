"""核心转换：USD stage → Newton 模型 → MJCF 文本 → 场景同步 / 网格外置 / 材质 / 命名 / 补丁。

执行顺序（每一步都是**确定性后处理**，改动可审计）：

1. 打开 USD、按配置选 variant
2. 修掉"自材质碰撞体"（否则会被 Newton 丢弃，例如驱动轮）
3. 重开被资产禁用的碰撞体（资产用它表达自碰撞排除）
4. 读场景层参数（newton:/mjc:）与材质（必须在 Newton 解析之前）
5. Newton `add_usd` → `SolverMuJoCo(save_to_mjcf=...)`
6. 场景同步（配置兜底 + USD 覆盖）→ 材质 → 执行器命名 → 网格外置 → 通用补丁
7. 写产物与报告
"""

from __future__ import annotations

import json
import re
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from . import config as config_mod
from . import mjcf_edit, usd_scene
from . import materials as materials_mod


# ---------------------------------------------------------------------------
# 结果
# ---------------------------------------------------------------------------
@dataclass
class ConvertResult:
    input: Path
    output: Path
    variants: list[dict[str, Any]]
    scene: dict[str, Any]
    model_stats: dict[str, Any]
    mjcf_stats: dict[str, Any]
    material_stats: dict[str, Any]
    mesh_stats: dict[str, Any]
    actuator_stats: dict[str, Any]
    ctrlrange_stats: dict[str, Any]
    contact_stats: dict[str, Any]
    site_stats: dict[str, Any]
    mask_stats: dict[str, Any]
    separation_stats: dict[str, Any]
    geom_name_stats: dict[str, Any]
    naming_stats: dict[str, Any]
    collision: dict[str, Any]
    changes: list[str]
    warnings: list[str]
    external_meshes: list[tuple[str, bool]]
    report_path: Path | None
    seconds: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "input": str(self.input),
            "output": str(self.output),
            "report": str(self.report_path) if self.report_path else None,
            "variants": self.variants,
            "scene": self.scene,
            "collision": self.collision,
            "newtonModel": self.model_stats,
            "mjcf": self.mjcf_stats,
            "materials": self.material_stats,
            "meshes": {k: (len(v) if isinstance(v, list) else v) for k, v in self.mesh_stats.items()},
            "actuators": self.actuator_stats,
            "ctrlrange": self.ctrlrange_stats,
            "contactExcludes": self.contact_stats,
            "sites": self.site_stats,
            "collisionMasks": {k: (len(v) if isinstance(v, list) else v) for k, v in self.mask_stats.items()},
            "visualCollisionSplit": self.separation_stats,
            "geomNames": self.geom_name_stats,
            "naming": self.naming_stats,
            "changes": self.changes,
            "warnings": self.warnings,
            "externalMeshes": [{"ref": r, "exists": e} for r, e in self.external_meshes],
            "seconds": round(self.seconds, 2),
        }


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
def _schema_resolvers(priority: Sequence[str]) -> list[Any]:
    from newton.usd import SchemaResolverMjc, SchemaResolverNewton, SchemaResolverPhysx

    table = {
        "mjc": SchemaResolverMjc,
        "newton": SchemaResolverNewton,
        "physx": SchemaResolverPhysx,
    }
    resolvers = []
    for name in priority:
        key = str(name).lower()
        if key not in table:
            raise ValueError(f"未知 schema resolver：{name}（可选 {sorted(table)}）")
        resolvers.append(table[key]())
    if not resolvers:
        raise ValueError("usd.schema_priority 不能为空")
    return resolvers


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if hasattr(value, "numpy"):  # warp / torch
        try:
            return _jsonable(value.numpy().tolist())
        except Exception:
            return str(value)
    if hasattr(value, "tolist"):
        try:
            return _jsonable(value.tolist())
        except Exception:
            return str(value)
    return str(value)


def resolve_paths(cfg: config_mod.Config) -> tuple[Path, Path]:
    """解析输入/输出路径。输出给目录时自动命名 `<stem>.xml`。"""
    if not cfg.usd.path:
        raise config_mod.ConfigError("缺少输入 USD：请用第一个位置参数或 usd.path 指定")
    source = Path(cfg.usd.path).expanduser()
    if source.is_dir():
        source, _ = usd_scene.resolve_input_path(source)

    raw_out = cfg.mjcf.path
    if not raw_out:
        output = Path.cwd() / f"{source.stem}.xml"
    else:
        output = Path(raw_out).expanduser()
        if output.is_dir() or str(raw_out).endswith(("/", "\\")):
            output = output / f"{source.stem}.xml"
    return source, output


# ---------------------------------------------------------------------------
# Newton
# ---------------------------------------------------------------------------
def build_newton_model(
    stage: Any,
    *,
    schema_priority: Sequence[str],
    verbose: bool = False,
    add_usd_kwargs: Mapping[str, Any] | None = None,
) -> tuple[Any, Any, dict[str, Any]]:
    """用 Newton 解析 USD，返回 (model, builder, import_result)。"""
    import newton
    from newton.solvers import SolverMuJoCo

    builder = newton.ModelBuilder()
    # 必须在 add_usd 之前注册：否则 model.mujoco.*（tendon / site / mjc: 命名空间）不会被分配
    SolverMuJoCo.register_custom_attributes(builder)
    kwargs = dict(add_usd_kwargs or {})
    result = builder.add_usd(
        stage,
        schema_resolvers=_schema_resolvers(schema_priority),
        verbose=verbose,
        **kwargs,
    )
    model = builder.finalize()
    return model, builder, result or {}


def collider_labels(model: Any) -> set[str]:
    """返回 Newton 模型里**真正参与碰撞**的 shape label 集合。

    用来在导出后识别"本应碰撞但掩码被算成 0"的 geom（见
    :func:`mjcf_edit.fix_zero_collision_masks`）。
    """
    try:
        import numpy as np
        from newton import ShapeFlags
    except ImportError:  # pragma: no cover
        return set()
    labels = getattr(model, "shape_label", None)
    if labels is None:
        return set()
    flags = np.asarray(model.shape_flags.numpy())
    indices = np.flatnonzero(flags & int(ShapeFlags.COLLIDE_SHAPES))
    return {str(labels[int(i)]) for i in indices}


def model_stats(model: Any) -> dict[str, Any]:
    def count(name: str) -> int:
        value = getattr(model, name, None)
        try:
            return int(len(value))
        except TypeError:
            return int(value) if isinstance(value, int) else -1

    return {
        "bodies": count("body_count"),
        "joints": count("joint_count"),
        "dofs": count("joint_dof_count"),
        "shapes": count("shape_count"),
        "articulations": count("articulation_count"),
        "worlds": count("world_count"),
        "device": str(getattr(model, "device", "?")),
    }


def _export_via_solver(model: Any, tmp_xml: Path, *, cpu: bool, solver_kwargs: Mapping[str, Any]) -> None:
    """`SolverMuJoCo(save_to_mjcf=...)` 是 Newton 官方唯一的 MJCF 出口。"""
    from newton.solvers import SolverMuJoCo

    kwargs = dict(solver_kwargs)
    kwargs.setdefault("use_mujoco_cpu", cpu)
    SolverMuJoCo(model, save_to_mjcf=str(tmp_xml), **kwargs)
    if not tmp_xml.is_file():
        raise RuntimeError("Newton 未生成 MJCF（save_to_mjcf 没有落盘）")


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def convert(cfg: config_mod.Config) -> ConvertResult:
    t0 = time.time()
    changes: list[str] = []
    warnings: list[str] = list(getattr(cfg, "_problems", []) or [])

    source, output = resolve_paths(cfg)
    output.parent.mkdir(parents=True, exist_ok=True)

    # 0) 远程资产：先把闭包下载到本地（Isaac 官方资产库就是这种形态）
    if usd_scene.is_remote(cfg.usd.path or ""):
        local = usd_scene.localize_remote(str(cfg.usd.path), cfg.usd.download_dir)
        changes.append(f"远程资产已本地化：{cfg.usd.path} → {local}")
        source = local

    # 1) 打开 USD + 选 variant -------------------------------------------------
    stage = usd_scene.open_stage(source)
    choices = usd_scene.apply_solver_variants(
        stage, overrides=cfg.usd.variants, prefer=cfg.usd.solver_variants
    )
    for choice in choices:
        flag = "" if choice.selection else "  ⚠️ 未选中（该 variant set 不贡献内容）"
        changes.append(
            f"variant {choice.prim_path}.{choice.variant_set} = {choice.selection!r}"
            f"（{choice.reason}，可选 {choice.candidates}）{flag}"
        )
        if choice.selection is None and choice.variant_set.lower() in ("physics", "solver"):
            warnings.append(
                f"{choice.prim_path}: variant set '{choice.variant_set}' 未选中，"
                "求解器专属层（mjc:/*newton:*）可能读不到；"
                f'请在配置里写 "variants": {{"{choice.variant_set}": "mujoco"}}'
            )

    # 2) 自材质碰撞体 ----------------------------------------------------------
    self_material = (
        usd_scene.fix_self_material_colliders(stage)
        if cfg.usd.fix_self_material_colliders
        else []
    )
    if self_material:
        changes.append(
            f"自材质碰撞体修复：{len(self_material)} 个碰撞体自带 NewtonMaterialAPI"
            "（Newton 会因此整块跳过），已转成独立材质 prim"
        )

    # 3) 碰撞体使能 ------------------------------------------------------------
    muted = usd_scene.muted_colliders(stage)
    forced = (
        usd_scene.apply_collision_mode(stage, cfg.usd.collision_mode)
        if cfg.usd.collision_mode == "enable"
        else []
    )
    if forced:
        changes.append(
            f"重开 {len(forced)} 个被资产禁用的碰撞体（collision_mode=enable）："
            f"{forced[:4]}{' …' if len(forced) > 4 else ''}"
        )
        warnings.append(
            f"USD 中有 {len(muted)} 个碰撞体被显式禁用（作者用它表达自碰撞排除）。"
            "重开后这些部位会真正参与接触；若出现自碰撞炸裂，用配置里的 patch "
            "<contact><exclude> 规则按需排除（报告里列出了全部路径）。"
        )
    elif cfg.usd.collision_mode == "respect" and muted:
        warnings.append(
            f"collision_mode=respect：{len(muted)} 个被禁用的碰撞体不参与接触，模型可能无碰撞。"
        )

    # 3b) 碰撞网格凸包精度（必须在 Newton 解析之前改 stage）---------------------
    hull_overrides: list[dict[str, Any]] = []
    if cfg.usd.max_hull_vertices is not None:
        hull_overrides = usd_scene.override_max_hull_vertices(
            stage, cfg.usd.max_hull_vertices
        )
        changes.append(
            f"碰撞网格凸包顶点上限 = {cfg.usd.max_hull_vertices}"
            f"（覆盖 {len(hull_overrides)} 个 USD 授权值；未授权的碰撞体走 "
            "add_usd.mesh_maxhullvert）"
        )

    # 4) 场景参数 + 材质（必须在 Newton 解析之前）-------------------------------
    settings, scene_prims = usd_scene.find_scene_settings(stage)
    scene_report = settings.as_dict()
    scene_report["prims"] = [
        {"path": p, "attrs": {k: _jsonable(v) for k, v in a.items()}} for p, a in scene_prims
    ]
    if settings.prim_path is None:
        changes.append(
            "USD 里没有场景 prim（PhysicsScene / NewtonSceneAPI / MjcSceneAPI）：场景参数用配置兜底值"
        )

    material_report = None
    if cfg.mjcf.materials.enabled:
        material_report = materials_mod.collect_for_stage(
            stage,
            usd_module=usd_scene.import_usd(),
            texture_dir=output.parent / "textures",
        )

    # 4b) 名字还原表（同样必须在 Newton 解析之前建好）
    prim_names = usd_scene.collect_prim_names(stage) if cfg.mjcf.name_from_usd else {}

    # 4c) add_usd 参数：显式透传优先，`usd.max_hull_vertices` 只做兜底
    add_usd_kwargs: dict[str, Any] = dict(cfg.usd.add_usd)
    if cfg.usd.max_hull_vertices is not None:
        add_usd_kwargs.setdefault("mesh_maxhullvert", cfg.usd.max_hull_vertices)

    # 5) Newton 解析 + 导出 ----------------------------------------------------
    try:
        model, _builder, import_result = build_newton_model(
            stage,
            schema_priority=cfg.usd.schema_priority,
            verbose=cfg.verbose,
            add_usd_kwargs=add_usd_kwargs,
        )
    except RuntimeError as exc:
        text = str(exc)
        if "Could not open asset" in text or "composition errors" in text:
            missing = sorted(set(re.findall(r"Could not open asset @([^@]+)@", text)))
            hint = (
                "Newton 无法加载该 stage：USD 的组合弧引用了**缺失的文件**。\n"
                "常见原因：远程资产没下全，或某个 variant 的依赖在原作者机器上是绝对路径。\n"
                f"缺失文件（前 5 个）：{missing[:5]}\n"
                "处理：换一个 usd.variants 取值（例如 Gripper/Physics 换别的候选值），"
                "或先把缺失文件补齐再转换。"
            )
            raise RuntimeError(hint) from exc
        raise
    stats = model_stats(model)
    if stats["joints"] <= 0:
        raise RuntimeError(
            "Newton 解析后的模型没有关节；MuJoCo 需要至少 1 个关节。"
            "请检查 usd.variants 里的 Physics 是否选对。"
        )

    solver_kwargs: dict[str, Any] = {"skip_visual_only_geoms": not cfg.mjcf.visuals}
    with tempfile.TemporaryDirectory(prefix="usd2mjcf-") as tmpdir:
        tmp_xml = Path(tmpdir) / "raw.xml"
        _export_via_solver(model, tmp_xml, cpu=cfg.use_cpu(), solver_kwargs=solver_kwargs)
        tree = mjcf_edit.load_xml(tmp_xml)
        root = tree.getroot()

        # 导出器可能把外部网格写到临时目录，搬过来
        for ref, exists in mjcf_edit.external_mesh_refs(root, tmp_xml.parent):
            if exists:
                src = (tmp_xml.parent / ref).resolve()
                dst = output.parent / ref
                dst.parent.mkdir(parents=True, exist_ok=True)
                dst.write_bytes(src.read_bytes())
                changes.append(f"复制外部网格 {ref} → {dst}")

        # 6a) 场景同步：配置兜底 → USD 覆盖 ------------------------------------
        scene_cfg = cfg.mjcf.scene
        effective: dict[str, Any] = {
            "timestep": scene_cfg.timestep,
            "iterations": scene_cfg.iterations,
            "gravity": scene_cfg.gravity,
            "flags": dict(scene_cfg.flags),
            "option": dict(scene_cfg.option),
            "compiler": dict(scene_cfg.compiler),
        }
        sources = {key: "config" for key in effective}
        if scene_cfg.sync_from_usd:
            for key in ("timestep", "iterations", "gravity"):
                value = getattr(settings, key)
                if value is not None:
                    effective[key] = value
                    sources[key] = "usd"
            effective["flags"].update(settings.flags)
            effective["option"].update(settings.option)
            effective["compiler"].update(settings.compiler)
        changes.append(
            "场景同步（配置兜底 + USD 覆盖）："
            + ", ".join(f"{k}={effective[k]!r}({sources[k]})" for k in ("timestep", "iterations"))
        )
        mjcf_edit.apply_scene_settings(
            root,
            timestep=effective["timestep"],
            iterations=effective["iterations"],
            gravity=effective["gravity"],
            flags=effective["flags"],
            option=effective["option"],
            compiler=effective["compiler"],
            sources=sources,
            notes=changes,
        )
        scene_report["effective"] = {k: _jsonable(v) for k, v in effective.items()}
        scene_report["sources"] = sources

        # 6b) 材质 --------------------------------------------------------------
        material_stats: dict[str, Any] = {
            "materials": 0, "textures": 0, "geomsMatched": 0, "primsScanned": 0,
        }
        if material_report is not None:
            changes.append("材质传递（USD → MJCF）：")
            mjcf_edit.apply_materials(
                root,
                material_report,
                pbr=cfg.mjcf.materials.pbr,
                notes=changes,
            )
            matched = sum(
                1
                for g in root.iter("geom")
                if g.get("material") and g.get("material") in material_report.materials
            )
            material_stats = {
                "materials": len(material_report.materials),
                "textures": len(material_report.textures_copied),
                "geomsMatched": matched,
                "primsScanned": material_report.prims_scanned,
            }

        # 6c) 碰撞掩码修复：USD 关闭自碰撞时 Newton 会把掩码全算成 0（连地面都不碰）
        mask_stats: dict[str, Any] = {"fixed": [], "candidates": 0}
        if cfg.mjcf.fix_zero_collision_masks:
            mask_stats = mjcf_edit.fix_zero_collision_masks(
                root, collider_labels(model), notes=changes
            )

        # 6c2) 视觉体 / 碰撞体分组（必须在掩码定稿之后：判断“谁是碰撞体”依赖掩码）
        separation_stats: dict[str, Any] = {}
        separation_stats = mjcf_edit.separate_visual_collision(
            root,
            hide_colliders=cfg.mjcf.materials.hide_colliders,
            notes=changes,
        )
        # 记下此刻的碰撞体，供最后的统一命名使用：后续的名字还原或补丁可能改写
        # geom 名字，届时再靠名字判断视觉/碰撞会出错。
        collider_ids = mjcf_edit.collision_geom_ids(root)

        # 6d) 执行器 ctrlrange：单位修正 + 缺失补齐（必须在改名之前，按原 joint 名匹配）
        ctrlrange_stats: dict[str, Any] = {"converted": [], "filled": [], "unlimited": []}
        if cfg.mjcf.fix_ctrlrange:
            ctrlrange_stats = mjcf_edit.fix_actuator_ctrlrange(
                root,
                auto_convert_degrees=cfg.mjcf.ctrlrange_auto_degrees,
                fill_missing_from_joint=cfg.mjcf.ctrlrange_from_joint,
                notes=changes,
            )

        # 6d) 名字还原（下划线路径名 → USD prim 叶子名），再据此命名执行器
        naming_stats: dict[str, Any] = {"renamed": {}, "kept": {}, "freeJoints": []}
        if cfg.mjcf.name_from_usd:
            changes.append("名字还原（Newton 下划线路径名 → USD prim 叶子名）：")
            naming_stats = mjcf_edit.apply_name_map(
                root, prim_names, cfg.mjcf.name_from_usd, notes=changes
            )
        named = mjcf_edit.name_actuators(root, cfg.mjcf.actuator_names, notes=changes)
        actuator_stats = {"named": named, "mode": cfg.mjcf.actuator_names}

        # 6d) 网格外置 ----------------------------------------------------------
        mesh_stats = mjcf_edit.externalize_meshes(
            root,
            output.parent,
            mode=cfg.mjcf.mesh_mode,
            inline_max_vertices=cfg.mjcf.inline_max_vertices,
            notes=changes,
        )

        # 6e) 额外碰撞排除（配置指定）
        contact_stats: dict[str, Any] = {"added": [], "skipped": [], "unknown": []}
        if cfg.mjcf.contact.excludes:
            contact_stats = mjcf_edit.add_contact_excludes(
                root, cfg.mjcf.contact.excludes, notes=changes, strict=cfg.strict_patch
            )

        # 6f) 通用补丁（优先级最高，可能改名/删元素）-------------------------------
        if cfg.patch:
            changes.append("应用补丁（配置内联 patch）：")
            mjcf_edit.apply_patch(root, cfg.patch, notes=changes, strict=cfg.strict_patch)

        # 6g) 补充 <site>（必须在补丁改名之后：配置里写的是产物最终 body 名）
        site_stats: dict[str, Any] = {
            "added": [], "skipped": [], "unknownBodies": [], "ambiguous": []
        }
        if cfg.mjcf.sites:
            site_stats = mjcf_edit.add_sites(
                root, cfg.mjcf.sites, notes=changes, strict=cfg.strict_patch
            )

        # 6h) geom 统一命名 -----------------------------------------------------
        # 必须放在补丁之后：补丁可能重命名 body，geom 名要使用最终的 body 名；
        # 同时必须晚于材质匹配、掩码修复与视觉/碰撞分组（它们都按旧名字工作）。
        geom_name_stats = mjcf_edit.normalize_geom_names(
            root,
            mode=cfg.mjcf.geom_names,
            collider_ids=collider_ids,
            notes=changes,
        )

        mjcf_edit.dump_xml(tree, output)

    # 7) 复核产物 --------------------------------------------------------------
    tree = mjcf_edit.load_xml(output)
    mjcf_stats = mjcf_edit.summarize(tree.getroot())
    externals = mjcf_edit.external_mesh_refs(tree.getroot(), output.parent)
    missing = [ref for ref, exists in externals if not exists]
    if missing:
        warnings.append(f"产物引用了 {len(missing)} 个缺失的外部文件：{missing[:5]}")

    report_path = _report_path(cfg, output)
    result = ConvertResult(
        input=source,
        output=output,
        variants=[c.as_dict() for c in choices],
        scene=scene_report,
        model_stats=stats,
        mjcf_stats=mjcf_stats,
        material_stats=material_stats,
        mesh_stats=mesh_stats,
        actuator_stats=actuator_stats,
        ctrlrange_stats=ctrlrange_stats,
        contact_stats=contact_stats,
        site_stats=site_stats,
        mask_stats=mask_stats,
        separation_stats=separation_stats,
        geom_name_stats=geom_name_stats,
        naming_stats=naming_stats,
        collision={
            "mode": cfg.usd.collision_mode,
            "mutedInUsd": muted,
            "reopened": forced,
            "selfMaterialFixed": self_material,
            "maxHullVertices": cfg.usd.max_hull_vertices,
            "hullOverrides": hull_overrides,
        },
        changes=changes,
        warnings=warnings,
        external_meshes=externals,
        report_path=report_path,
        seconds=time.time() - t0,
    )
    if report_path is not None:
        payload = result.as_dict()
        payload["config"] = _jsonable(config_mod.to_dict(cfg))
        payload["schemaAttrs"] = _jsonable(import_result.get("schema_attrs", {}))
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def _report_path(cfg: config_mod.Config, output: Path) -> Path | None:
    if not cfg.report_enabled:
        return None
    if cfg.report:
        return Path(cfg.report).expanduser()
    return output.with_suffix(".report.json")
