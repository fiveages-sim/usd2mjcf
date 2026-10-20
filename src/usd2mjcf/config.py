"""配置文件：`usd2mjcf` 的唯一"旋钮面"。

设计原则：**CLI 只负责三个参数（输入 USD、输出 MJCF、配置文件）**，其余全部在这里。
配置文件是纯 JSON，所有键都有明确默认值；未出现的键取默认值，未识别的键会警告
（拼错键导致静默不生效是本项目最忌讳的失败模式）。

    usd2mjcf robot.usda out/robot.xml out/config.json

也可以把 `usd` / `output` 写在配置里，CLI 传的位置参数优先级更高。
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

DEFAULT_SCHEMA_PRIORITY: tuple[str, ...] = ("mjc", "newton", "physx")
DEFAULT_VIEWS: tuple[tuple[float, float], ...] = ((135.0, -20.0), (45.0, -10.0), (225.0, 0.0))


# ---------------------------------------------------------------------------
# 子配置
# ---------------------------------------------------------------------------
@dataclass
class UsdConfig:
    path: str | None = None
    #: variant set 名 -> 取值。**只写你想覆盖的**，其余沿用 USD 里作者授权的默认选择
    #: （`Physics` 这类没授权默认值的，会按 `solverVariants` 的顺序自动挑）。
    variants: dict[str, str] = field(default_factory=dict)
    #: solver-like variant set 的自动优先顺序（没有 authored 值时生效）
    solver_variants: list[str] = field(default_factory=lambda: ["mujoco", "newton"])
    #: enable=重开被资产禁用的碰撞体（默认）；respect=完全按 USD
    collision_mode: str = "enable"
    #: 这些碰撞体自带 NewtonMaterialAPI 时会被 Newton 丢弃（轮子就是这种情况），
    #: 这里把它们转成普通碰撞体 + 独立材质 prim
    fix_self_material_colliders: bool = True
    #: 透传给 newton.ModelBuilder.add_usd 的参数
    add_usd: dict[str, Any] = field(default_factory=dict)
    #: 远程（https）USD 的下载目录；None 表示 ./.usd_cache
    download_dir: str | None = None
    schema_priority: list[str] = field(default_factory=lambda: list(DEFAULT_SCHEMA_PRIORITY))


@dataclass
class SceneConfig:
    #: 是否把 USD 场景层（PhysicsScene / NewtonSceneAPI / MjcSceneAPI）的值同步到 MJCF
    sync_from_usd: bool = True
    #: 下面这些是**兜底值**：USD 里有就用 USD 的，没有就用这里的
    timestep: float | None = 0.002
    iterations: int | None = None
    gravity: list[float] | None = None
    flags: dict[str, bool] = field(default_factory=dict)
    option: dict[str, Any] = field(default_factory=dict)
    compiler: dict[str, Any] = field(default_factory=dict)


@dataclass
class MaterialConfig:
    enabled: bool = True
    #: 同时写 metallic/roughness。默认关：OmniPBR 常见 metallic=1，MuJoCo 默认灯光下发黑
    pbr: bool = False
    #: 碰撞体放进 group=3（MuJoCo 约定：碰撞专用，默认不渲染）
    hide_colliders: bool = True


@dataclass
class ContactConfig:
    """额外碰撞排除（补 USD 没表达成形状级过滤的部位）。

    每项是 ``[body1, body2]``，两侧支持 glob。已经存在的 exclude 会跳过，
    body 名不存在会显式警告。
    """

    excludes: list[list[str]] = field(default_factory=list)


@dataclass
class MjcfConfig:
    path: str | None = None
    #: obj=网格外置成 .obj 文件（XML 小、与旧流程一致）；inline=写在 XML 里（单文件但极大）
    mesh_mode: str = "obj"
    #: inline 模式下超过该顶点数的网格仍然外置（防止 XML 爆到上百 MB）
    inline_max_vertices: int = 2000
    #: joint=执行器用所驱动 joint 的名字命名；none=不命名
    actuator_names: str = "joint"
    #: 把 Newton 生成的下划线路径名换回 USD prim 叶子名（如 fl_steer_joint）。
    #: 可用项：joint / body / geom / site / mesh；空列表=不改名
    name_from_usd: list[str] = field(default_factory=lambda: ["joint", "body"])
    #: 修正执行器 ctrlrange：单位混用（度/弧度）+ 缺失时从关节限位补
    fix_ctrlrange: bool = True
    #: 跨度比 ≈ 180/π 时把 ctrlrange 由「度」换算为弧度
    ctrlrange_auto_degrees: bool = True
    #: ctrlrange 缺失/为零时，用关节限位补（仅位置类执行器）
    ctrlrange_from_joint: bool = True
    #: USD 关闭自碰撞时 Newton 会把碰撞掩码全算成 0（连地面都不碰）→ 自动恢复
    fix_zero_collision_masks: bool = True
    #: 是否导出仅用于可视化的几何（关掉可显著减小产物，适合纯 RL 训练）
    visuals: bool = True
    scene: SceneConfig = field(default_factory=SceneConfig)
    materials: MaterialConfig = field(default_factory=MaterialConfig)
    contact: ContactConfig = field(default_factory=ContactConfig)


@dataclass
class VerifyConfig:
    enabled: bool = True
    render: bool = True
    gl: str | None = None
    smoke_steps: int = 200
    views: list[list[float]] = field(default_factory=lambda: [list(v) for v in DEFAULT_VIEWS])
    render_dir: str | None = None


@dataclass
class Config:
    usd: UsdConfig = field(default_factory=UsdConfig)
    mjcf: MjcfConfig = field(default_factory=MjcfConfig)
    #: 通用补丁（结构见 README §6），写在配置里就不用额外文件
    patch: dict[str, Any] = field(default_factory=dict)
    strict_patch: bool = False
    #: cuda | cpu —— 决定 SolverMuJoCo 用 mujoco-warp 还是 MuJoCo-C 后端
    backend: str = "cuda"
    #: 完整转换报告（JSON）；None 表示写到 <output>.report.json 旁边
    report: str | None = None
    report_enabled: bool = True
    verbose: bool = False
    verify: VerifyConfig = field(default_factory=VerifyConfig)

    # -- 便捷访问 ----------------------------------------------------------
    @property
    def input_path(self) -> Path | None:
        return Path(self.usd.path).expanduser() if self.usd.path else None

    @property
    def output_path(self) -> Path | None:
        return Path(self.mjcf.path).expanduser() if self.mjcf.path else None

    def use_cpu(self) -> bool:
        return str(self.backend).lower() in ("cpu", "mujoco_cpu", "mujoco-c")


# ---------------------------------------------------------------------------
# 加载 / 保存
# ---------------------------------------------------------------------------
class ConfigError(ValueError):
    """配置文件本身有问题（键名拼错、取值非法等）。"""


def _fill_dataclass(cls: type, raw: Mapping[str, Any], path: str, problems: list[str]) -> Any:
    known = {f.name: f for f in fields(cls)}
    kwargs: dict[str, Any] = {}
    for key, value in raw.items():
        if key.startswith("_"):
            continue
        if key not in known:
            problems.append(f"{path}.{key}: 未知配置项（已忽略）")
            continue
        ftype = known[key].type
        if is_dataclass(ftype) and isinstance(value, Mapping):
            kwargs[key] = _fill_dataclass(ftype, value, f"{path}.{key}", problems)
        elif isinstance(ftype, str) and ftype in _DATACLASS_REGISTRY and isinstance(value, Mapping):
            kwargs[key] = _fill_dataclass(_DATACLASS_REGISTRY[ftype], value, f"{path}.{key}", problems)
        else:
            kwargs[key] = value
    return cls(**kwargs)


def load(path: str | Path, *, strict: bool = False) -> tuple[Config, list[str]]:
    """读取配置文件，返回 (配置, 警告列表)。

    键名拼错、取值非法只会给出**显式警告**（`strict=True` 时直接报错），
    绝不静默忽略。
    """
    file = Path(path).expanduser()
    if not file.is_file():
        raise ConfigError(f"配置文件不存在：{file}")
    try:
        raw = json.loads(file.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ConfigError(f"配置文件不是合法 JSON：{file}: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise ConfigError(f"配置文件顶层必须是对象：{file}")
    cfg = from_dict(raw, strict=strict)
    return cfg, list(getattr(cfg, "_problems", []) or [])


def from_dict(raw: Mapping[str, Any], *, strict: bool = False) -> Config:
    problems: list[str] = []
    cfg = _fill_dataclass(Config, raw, "config", problems)
    validation = validate(cfg)
    problems.extend(validation)
    if problems and strict:
        raise ConfigError("配置有问题：\n  - " + "\n  - ".join(problems))
    cfg._problems = problems  # type: ignore[attr-defined]
    return cfg


_DATACLASS_REGISTRY = {
    "UsdConfig": UsdConfig,
    "SceneConfig": SceneConfig,
    "MaterialConfig": MaterialConfig,
    "ContactConfig": ContactConfig,
    "MjcfConfig": MjcfConfig,
    "VerifyConfig": VerifyConfig,
}


def validate(cfg: Config) -> list[str]:
    problems: list[str] = []
    if cfg.usd.collision_mode not in ("enable", "respect"):
        problems.append(f"usd.collision_mode: 只能是 enable/respect，收到 {cfg.usd.collision_mode!r}")
    if cfg.mjcf.mesh_mode not in ("obj", "inline"):
        problems.append(f"mjcf.mesh_mode: 只能是 obj/inline，收到 {cfg.mjcf.mesh_mode!r}")
    if cfg.mjcf.actuator_names not in ("joint", "none"):
        problems.append(f"mjcf.actuator_names: 只能是 joint/none，收到 {cfg.mjcf.actuator_names!r}")
    allowed_tags = {"joint", "body", "geom", "site", "mesh", "tendon", "actuator"}
    unknown = [t for t in cfg.mjcf.name_from_usd if t not in allowed_tags]
    if unknown:
        problems.append(f"mjcf.name_from_usd: 不支持的实体 {unknown}（可选 {sorted(allowed_tags)}）")
    if str(cfg.backend).lower() not in ("cuda", "cpu", "mujoco_cpu", "mujoco-c"):
        problems.append(f"backend: 只能是 cuda/cpu，收到 {cfg.backend!r}")
    if cfg.usd.schema_priority and not set(cfg.usd.schema_priority) <= {"mjc", "newton", "physx"}:
        problems.append(f"usd.schema_priority: 只能包含 mjc/newton/physx，收到 {cfg.usd.schema_priority}")
    if cfg.mjcf.scene.timestep is not None and cfg.mjcf.scene.timestep <= 0:
        problems.append(f"mjcf.scene.timestep: 必须为正数，收到 {cfg.mjcf.scene.timestep}")
    return problems


def to_dict(cfg: Config) -> dict[str, Any]:
    return asdict(cfg)


def save(cfg: Config, path: str | Path, *, comment: str | None = None) -> Path:
    """把配置（含默认值）写成可读的 JSON —— 用作"示例配置文件"。"""
    payload = to_dict(cfg)
    _strip_internal(payload)
    if comment:
        payload = {"_comment": comment.splitlines(), **payload}
    file = Path(path).expanduser()
    file.parent.mkdir(parents=True, exist_ok=True)
    file.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return file


def _strip_internal(payload: dict[str, Any]) -> None:
    for key in list(payload):
        if key.startswith("_"):
            del payload[key]
        elif isinstance(payload[key], dict):
            _strip_internal(payload[key])


# ---------------------------------------------------------------------------
# 示例
# ---------------------------------------------------------------------------
EXAMPLE_COMMENT = """\
usd2mjcf 配置文件（唯一旋钮面）。
CLI 只有三个参数：usd2mjcf <输入.usda> <输出.xml> [本文件]
未出现的键取默认值；拼错的键会给出显式警告。
先不带配置跑一次，看 <输出>.report.json 里的 variants / scene 段，
再把要覆盖的值抄到下面。"""


def example_config() -> Config:
    """给 `config.example.json` 用的参考配置：**全部默认值**。

    刻意不塞机型相关的覆盖项 —— 具体怎么换变体 / 加排除 / 写补丁，见
    `examples/fiveages_w2/config.json`。这样示例文件只承担"键表"职责，
    不会因为换机型而误导。
    """
    return Config()


def describe() -> list[tuple[str, str]]:
    """(配置键, 说明) 列表 —— 用于 README / 文档生成。"""
    return [
        ("usd.path", "输入 USD（也可由 CLI 第一个参数给出）"),
        ("usd.variants", "variant set → 取值；只写覆盖项，其余沿用 USD 的授权默认值"),
        ("usd.solver_variants", "无授权默认值时的自动优先顺序（默认 mujoco/newton）"),
        ("usd.collision_mode", "enable=重开被禁用的碰撞体；respect=完全按 USD"),
        ("usd.fix_self_material_colliders", "修复自带 NewtonMaterialAPI 被丢弃的碰撞体"),
        ("usd.add_usd", "透传给 newton ModelBuilder.add_usd 的参数"),
        ("usd.download_dir", "远程 https USD 的下载目录（默认 ./.usd_cache）"),
        ("usd.schema_priority", "schema resolver 优先级（默认 mjc,newton,physx）"),
        ("mjcf.path", "输出 MJCF（也可由 CLI 第二个参数给出；给目录则自动命名）"),
        ("mjcf.mesh_mode", "obj=网格外置为 .obj；inline=写进 XML"),
        ("mjcf.inline_max_vertices", "inline 模式下超过该顶点数的网格仍然外置"),
        ("mjcf.actuator_names", "joint=用驱动的 joint 名命名执行器；none=不命名"),
        ("mjcf.name_from_usd", "把下划线路径名还原为 USD prim 叶子名（joint/body/geom/...）"),
        ("mjcf.fix_ctrlrange", "修正执行器 ctrlrange（单位混用 / 缺失）"),
        ("mjcf.ctrlrange_auto_degrees", "跨度比≈180/π 时把 ctrlrange 由度换算为弧度"),
        ("mjcf.ctrlrange_from_joint", "ctrlrange 缺失时用关节限位补"),
        ("mjcf.fix_zero_collision_masks", "修复“USD 关闭自碰撞导致掩码全 0”的碰撞体"),
        ("mjcf.visuals", "是否导出仅可视化的几何"),
        ("mjcf.scene.timestep", "兜底 timestep（USD 场景里有时以 USD 为准）"),
        ("mjcf.scene.sync_from_usd", "是否把 USD 场景层参数同步到 option/compiler/flag"),
        ("mjcf.materials.enabled", "是否传递 USD 颜色/材质"),
        ("mjcf.materials.pbr", "是否同时写 metallic/roughness"),
        ("mjcf.materials.hide_colliders", "碰撞体写入 group=3（默认不渲染）"),
        ("mjcf.contact.excludes", "额外碰撞排除 [[body1, body2], ...]，两侧支持 glob"),
        ("patch", "通用补丁（match 用 glob；见 README §6）"),
        ("strict_patch", "补丁规则未命中任何元素时直接报错"),
        ("backend", "cuda | cpu"),
        ("report / report_enabled", "转换报告 JSON 路径 / 是否生成"),
        ("verbose", "打印 Newton/USD 详细解析日志"),
        ("verify.enabled", "转换后编译 + 步进 + 渲染"),
        ("verify.render", "是否离屏渲染"),
        ("verify.gl", "egl | osmesa | glfw"),
        ("verify.smoke_steps", "零控制步进步数"),
        ("verify.views", "渲染视角 [[方位角, 俯仰角], ...]"),
    ]


def load_or_default(path: str | Path | None, *, strict: bool = False) -> Config:
    if path is None:
        return Config()
    cfg, _ = load(path, strict=strict)
    return cfg


def apply_cli_overrides(cfg: Config, *, usd: str | None, output: str | None) -> Config:
    """CLI 的位置参数优先级高于配置文件。"""
    if usd:
        cfg.usd.path = usd
    if output:
        cfg.mjcf.path = output
    return cfg


def views_tuples(cfg: VerifyConfig) -> list[tuple[float, float]]:
    views: Sequence[Sequence[float]] = cfg.views or DEFAULT_VIEWS
    out: list[tuple[float, float]] = []
    for view in views:
        if len(view) != 2:
            continue
        out.append((float(view[0]), float(view[1])))
    return out or list(DEFAULT_VIEWS)
