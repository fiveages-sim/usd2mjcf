"""OpenUSD 侧的处理：打开 stage、选择 variant、提取场景级仿真参数、自动发现转换入口。

这一层的职责只有一个：**把 USD 里"作者授权过的真值"忠实地读出来**，不做任何机型特判。
"""

from __future__ import annotations

import hashlib
import os
import posixpath
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping, Sequence

USD_EXTENSIONS = (".usda", ".usdc", ".usd", ".usdz")

# 这些是 Omniverse/MDL 侧的辅助文件：MuJoCo 用不到，且常不在资产压缩包内（404 是常态）
_SKIP_SUFFIXES = (".mdl", ".mdr")

# 我们会认的场景级属性前缀。mjc: 是 MuJoCo 官方 schema（优先级最高），
# newton: 是 Newton/Isaac 的扩展 schema。
_SCENE_PREFIXES = ("mjc:", "newton:", "physics:")

# 只有出现下列任一属性（或 typeName == PhysicsScene）的 prim 才被认为是"场景"，
# 避免把机器人根 prim 上的 newton:selfCollisionEnabled 当成场景设置。
_SCENE_KEYS = (
    "newton:timeStepsPerSecond",
    "newton:maxSolverIterations",
    "newton:gravityEnabled",
    "physics:gravityMagnitude",
    "physics:gravityDirection",
)

# variant 选择时的偏好顺序：先找求解器专属层，再退回其它。
_PREFERRED_VARIANTS = ("mujoco", "newton")


# ---------------------------------------------------------------------------
# USD 导入（newton_usd_schemas 必须早于第一次 pxr 使用，否则 schema 注册不上）
# ---------------------------------------------------------------------------
def import_usd():
    """返回 (Sdf, Usd, UsdGeom, UsdPhysics, UsdShade)，并保证 newton schema 已注册。"""
    import newton_usd_schemas  # noqa: F401  # 注册 newton:* 的 codeless schema

    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    return Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


# ---------------------------------------------------------------------------
# 入口发现
# ---------------------------------------------------------------------------
_NAME_PENALTY = ("payload", "instance", "geometry", "material", "texture", "side/")
_NAME_BONUS = ("robot", "env", "scene", "assembly")

# 自动发现时要跳过的目录：虚拟环境/依赖包里也会带 .usda 测试资产，
# 不排掉就会出现"在仓库里自动选中了某个包的测试文件"这种荒谬结果。
_IGNORED_DIRS = frozenset(
    {
        ".git", ".hg", ".svn", ".venv", "venv", "env", ".env", "site-packages",
        "dist-packages", "node_modules", "__pycache__", "build", "dist", ".cache",
        ".uv-cache", ".mypy_cache", ".pytest_cache", ".ruff_cache", "install",
    }
)


def _is_ignored(path: Path, root: Path) -> bool:
    try:
        parts = path.relative_to(root).parts[:-1]
    except ValueError:
        return False
    return any(part.startswith(".") or part in _IGNORED_DIRS for part in parts)


def discover_candidates(root: Path, max_files: int = 400) -> list[tuple[float, Path, str]]:
    """在目录里找出"像入口"的 USD 文件，返回 (分数, 路径, 理由) 降序列表。

    纯文本启发式：不打开 stage（USD 打开有成本，且大库很多文件打不开）。
    """
    root = root.resolve()
    found: list[tuple[float, Path, str]] = []
    for path in sorted(root.rglob("*")):
        if len(found) >= max_files:
            break
        if path.suffix.lower() not in USD_EXTENSIONS or not path.is_file():
            continue
        if _is_ignored(path, root):
            continue
        try:
            head = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if not head.lstrip().startswith("#usda"):
            continue  # 二进制 .usdc 不参与启发式打分

        score, reasons = 0.0, []
        rel = path.relative_to(root).as_posix()
        if "defaultPrim" in head:
            score += 1.0
            reasons.append("defaultPrim")
        if re.search(r"payloads?/", head) or "prepend references" in head:
            score += 3.0
            reasons.append("references payload")
        if "variants" in head:
            score += 2.0
            reasons.append("variants")
        if re.search(r"PhysicsArticulationRootAPI|ArticulationRootAPI", head):
            score += 2.0
            reasons.append("articulation root")
        if re.search(r"NewtonSceneAPI|MjcSceneAPI|PhysicsScene", head):
            score += 1.5
            reasons.append("physics scene")
        if re.search(r"\bdef\s+\w+\s*\"", head):
            score += 0.5
            reasons.append("defines prims")
        if head.count('over "') > head.count('def "'):
            score -= 2.0
            reasons.append("mostly overrides")
        for bad in _NAME_PENALTY:
            if bad in rel.lower():
                score -= 3.0
                reasons.append(f"name:-{bad}")
        for good in _NAME_BONUS:
            if good in rel.lower():
                score += 0.5
                reasons.append(f"name:+{good}")
        if path.parent.name == "env" or "/env/" in rel:
            score -= 1.0
            reasons.append("env layer")
        found.append((score, path, ", ".join(reasons)))

    found.sort(key=lambda item: (-item[0], len(str(item[1]))))
    return found


# ---------------------------------------------------------------------------
# variant 选择
# ---------------------------------------------------------------------------
@dataclass
class VariantChoice:
    prim_path: str
    variant_set: str
    candidates: list[str]
    selection: str | None
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "prim": self.prim_path,
            "variantSet": self.variant_set,
            "candidates": self.candidates,
            "selection": self.selection,
            "reason": self.reason,
        }


def _variant_targets(stage: Any, Usd: Any) -> list[Any]:
    """挑出需要做 variant 选择的 prim：defaultPrim、articulation root，及其带 variant 的后代。"""
    targets: list[Any] = []
    seen: set[str] = set()

    def push(prim: Any) -> None:
        if prim and prim.IsValid() and prim.GetPath().pathString not in seen:
            seen.add(prim.GetPath().pathString)
            targets.append(prim)

    root = stage.GetPseudoRoot()
    push(stage.GetDefaultPrim())
    for prim in stage.Traverse():
        if "ArticulationRootAPI" in (prim.GetAppliedSchemas() or []):
            push(prim)
    # 顶层 prim 里带 variant 的（例如只有 scene 层包裹的情况）
    for prim in root.GetChildren():
        push(prim)
    for prim in stage.TraverseAll():
        types = prim.GetTypeName()
        if types in ("PhysicsScene",) or "SceneAPI" in "".join(prim.GetAppliedSchemas() or []):
            push(prim)

    # 再补上 defaultPrim / root 的后代（限定深度，避免遍历整个资产库）
    expanded: list[Any] = []
    for prim in targets:
        expanded.append(prim)
        depth = len(prim.GetPath().pathString.strip("/").split("/"))
        for child in stage.Traverse():
            child_path = child.GetPath().pathString
            if not child_path.startswith(prim.GetPath().pathString + "/"):
                continue
            if (
                depth < len(child_path.strip("/").split("/")) <= depth + 3
                and list(child.GetVariantSets().GetNames())
            ):
                expanded.append(child)
    out, seen2 = [], set()
    for prim in expanded:
        key = prim.GetPath().pathString
        if key not in seen2:
            seen2.add(key)
            out.append(prim)
    return out


def apply_solver_variants(
    stage: Any,
    *,
    overrides: Mapping[str, str] | None = None,
    prefer: Sequence[str] = _PREFERRED_VARIANTS,
    select_unselected: bool = True,
) -> list[VariantChoice]:
    """把 Physics 之类的 variant 显式选到求解器专属层（默认 mujoco）。

    Isaac 导出的资产常把 `Physics` variant 的默认值留空，由 VariantSwitcher 在运行时按引擎
    选择。若不显式选中，组合出来的 stage 里**读不到** `mjc:*` 授权值 —— 转换结果就只剩几何。
    """
    overrides = dict(overrides or {})
    choices: list[VariantChoice] = []
    for prim in _variant_targets(stage, None):
        try:
            vset_names = list(prim.GetVariantSets().GetNames())
        except Exception:  # pragma: no cover - 防御性
            continue
        for vs_name in vset_names:
            vset = prim.GetVariantSet(vs_name)
            names = list(vset.GetVariantNames())
            if not names:
                continue
            authored = vset.GetVariantSelection() or None
            if vs_name in overrides:
                chosen, reason = overrides[vs_name], "cli"
            elif authored:
                chosen, reason = authored, "authored"
            elif select_unselected:
                chosen = next((n for n in names if n.lower() in prefer), None)
                reason = "prefer:" + chosen if chosen else "none"
            else:
                chosen, reason = None, "skipped"
            if chosen is not None and chosen not in names:
                raise ValueError(
                    f"{prim.GetPath()}: variant set '{vs_name}' 没有取值 '{chosen}'，可选 {names}"
                )
            if chosen is not None:
                vset.SetVariantSelection(chosen)
            choices.append(
                VariantChoice(prim.GetPath().pathString, vs_name, names, chosen, reason)
            )
    return choices


# ---------------------------------------------------------------------------
# 场景级仿真参数
# ---------------------------------------------------------------------------
def _as_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_vec3(value: Any) -> tuple[float, float, float] | None:
    try:
        seq = list(value)
    except TypeError:
        return None
    if len(seq) != 3:
        return None
    try:
        return (float(seq[0]), float(seq[1]), float(seq[2]))
    except (TypeError, ValueError):
        return None


def read_prim_attrs(prim: Any, prefixes: Sequence[str] = _SCENE_PREFIXES) -> dict[str, Any]:
    """读出 prim 上所有已授权、且命名空间匹配的属性（不依赖 schema 是否注册）。"""
    out: dict[str, Any] = {}
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if not name.startswith(tuple(prefixes)):
            continue
        if not attr.HasAuthoredValue():
            continue
        try:
            value = attr.Get()
        except Exception:  # pragma: no cover - 坏 asset 容错
            continue
        out[name] = value
    return out


@dataclass
class SceneSettings:
    """从 USD 场景层抽出来、需要落到 MJCF `<option>` / `<compiler>` 的设置。"""

    prim_path: str | None = None
    timestep: float | None = None
    iterations: int | None = None
    gravity: tuple[float, float, float] | None = None
    flags: dict[str, bool] = field(default_factory=dict)
    option: dict[str, Any] = field(default_factory=dict)
    compiler: dict[str, Any] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "prim": self.prim_path,
            "timestep": self.timestep,
            "iterations": self.iterations,
            "gravity": self.gravity,
            "flags": self.flags,
            "option": {k: _jsonable(v) for k, v in self.option.items()},
            "compiler": {k: _jsonable(v) for k, v in self.compiler.items()},
            "raw": {k: _jsonable(v) for k, v in self.raw.items()},
        }


def _jsonable(value: Any) -> Any:
    if isinstance(value, (str, bool, int, float)) or value is None:
        return value
    try:
        return [float(x) for x in value]
    except TypeError:
        return str(value)


# MJCF <option> 里合法的属性名（用于直通 mjc:option:*）
_MJCF_OPTION_ATTRS = frozenset(
    {
        "timestep", "apiratio", "impratio", "tolerance", "ls_tolerance", "ccd_tolerance",
        "sleep_tolerance", "sdf_tolerance", "sdf_iterations", "sdf_initpoints",
        "integrator", "solver", "iterations", "ls_iterations", "ccd_iterations",
        "cone", "jacobian", "density", "viscosity", "wind", "magnetic",
        "o_margin", "o_solref", "o_solimp", "o_friction",
        "noslip_iterations", "noslip_tolerance", "mpr_iterations", "mpr_tolerance",
        "slip_friction", "gravity",
    }
)


def _read_scene_from_prim(attrs: Mapping[str, Any]) -> SceneSettings:
    s = SceneSettings(raw=dict(attrs))
    # timestep：mjc:option:timestep 是秒；newton:timeStepsPerSecond 是 Hz。mjc 优先。
    dt = _as_float(attrs.get("mjc:option:timestep"))
    if dt and dt > 0:
        s.timestep = dt
    else:
        tps = _as_float(attrs.get("newton:timeStepsPerSecond"))
        if tps and tps > 0:
            s.timestep = 1.0 / tps
    # iterations
    it = attrs.get("mjc:option:iterations", attrs.get("newton:maxSolverIterations"))
    if it is not None:
        try:
            it_int = int(it)
        except (TypeError, ValueError):
            it_int = None
        if it_int is not None and it_int >= 0:
            s.iterations = it_int
    # gravity
    enabled = attrs.get("newton:gravityEnabled", attrs.get("mjc:flag:gravity"))
    if enabled is not None and not bool(enabled):
        s.gravity = (0.0, 0.0, 0.0)
    else:
        mag = _as_float(attrs.get("physics:gravityMagnitude"))
        direction = _as_vec3(attrs.get("physics:gravityDirection"))
        if mag is not None and direction is not None:
            norm = sum(c * c for c in direction) ** 0.5
            if norm > 0:
                s.gravity = tuple(mag * c / norm for c in direction)
    # 直通
    for name, value in attrs.items():
        if name.startswith("mjc:option:"):
            key = name[len("mjc:option:") :]
            if key in _MJCF_OPTION_ATTRS and value is not None:
                s.option[key] = value
        elif name.startswith("mjc:flag:"):
            s.flags[name[len("mjc:flag:") :]] = bool(value)
        elif name.startswith("mjc:compiler:"):
            s.compiler[name[len("mjc:compiler:") :]] = value
    return s


def find_scene_settings(stage: Any) -> tuple[SceneSettings, list[tuple[str, dict[str, Any]]]]:
    """在整个 stage 上找场景 prim 并抽取设置。

    返回 (最佳场景设置, [(prim_path, 该 prim 的所有场景属性), ...])。
    """
    candidates: list[tuple[int, str, dict[str, Any]]] = []
    for prim in stage.TraverseAll():
        attrs = read_prim_attrs(prim)
        is_scene_type = prim.GetTypeName() == "PhysicsScene"
        has_scene_key = any(k in attrs for k in _SCENE_KEYS) or any(
            k.startswith(("mjc:option:", "mjc:flag:")) for k in attrs
        )
        schemas = "".join(prim.GetAppliedSchemas() or [])
        has_scene_schema = "NewtonSceneAPI" in schemas or "MjcSceneAPI" in schemas
        if not (is_scene_type or has_scene_key or has_scene_schema):
            continue
        if not attrs:
            continue
        candidates.append((len(attrs), prim.GetPath().pathString, attrs))

    if not candidates:
        return SceneSettings(), []
    candidates.sort(key=lambda item: -item[0])
    _, path, attrs = candidates[0]
    settings = _read_scene_from_prim(attrs)
    settings.prim_path = path
    return settings, [(p, a) for _, p, a in candidates]


# ---------------------------------------------------------------------------
# 打开 stage
# ---------------------------------------------------------------------------
def resolve_input_path(raw: str | os.PathLike[str]) -> tuple[Path, list[tuple[float, Path, str]]]:
    """把用户输入解析成一个具体的 USD 文件（目录/省略时走自动发现）。"""
    path = Path(raw).expanduser()
    if path.is_file():
        return path, []
    if path.is_dir():
        ranked = discover_candidates(path)
        if not ranked:
            raise FileNotFoundError(f"{path} 下没有找到可用的 USD 入口文件")
        return ranked[0][1], ranked
    raise FileNotFoundError(f"输入不存在：{path}")


def is_remote(raw: str | os.PathLike[str]) -> bool:
    return str(raw).lower().startswith(("http://", "https://"))


def localize_remote(
    url: str, dest_dir: Path | str | None = None, *, max_files: int = 4000, quiet: bool = True
) -> Path:
    """把远程 USD 及其**完整依赖闭包**镜像到本地，返回本地入口文件路径。

    Isaac Sim 官方资产库（``omniverse-content-*``）的入口 `.usd` 通过
    ``payload`` / ``reference`` 拉一堆子文件。两个坑：

    * **payload 是按需加载的合成弧**：只顺着已打开的 layer 找引用会漏掉整个
      ``payloads/`` 目录（Newton 自带的下载器就是这样，Franka 因此打不开）；
    * 变体里的 payload 也要一起下，否则以后换 ``Gripper``/``Mesh`` 变体还要联网。

    所以这里用 OpenUSD 自己的依赖枚举（``UsdUtils.ComputeAllDependencies``）递归
    收集，再按**相对路径镜像**到本地目录，保证引用关系原样成立。
    """
    from urllib.parse import urljoin, urlparse

    if not str(url).lower().startswith("https://"):
        raise ValueError(f"远程 USD 只支持 https：{url}")

    root_rel = posixpath.basename(urlparse(url).path) or "root.usda"
    base_url = urljoin(url, "./")
    dest = Path(dest_dir).expanduser().resolve() if dest_dir else Path.cwd() / ".usd_cache"
    dest.mkdir(parents=True, exist_ok=True)

    queue: list[tuple[str, str]] = [(root_rel, str(url))]
    seen: set[str] = set()
    fetched = 0
    missing: list[str] = []
    while queue:
        rel, source = queue.pop(0)
        if rel in seen or len(seen) >= max_files:
            continue
        seen.add(rel)
        if rel.lower().endswith(_SKIP_SUFFIXES):
            continue
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.is_file():
            try:
                _fetch_https(source, target)
            except _RemoteNotFound as exc:
                missing.append(rel)
                if not quiet:
                    print(f"    跳过（远端不存在）{rel}：{exc}")
                continue
            fetched += 1
            if not quiet:
                print(f"    下载 {rel}")
        if target.suffix.lower() not in USD_EXTENSIONS:
            continue
        for dep in _usd_dependency_paths(target):
            child = _rebase_dependency(dep, target, dest, base_url)
            if child is not None:
                queue.append(child)

    entry = dest / root_rel
    if not entry.is_file():
        raise RuntimeError(f"远程资产下载失败（未生成入口文件）：{url}")
    if missing and not quiet:
        usd_missing = [m for m in missing if m.lower().endswith(USD_EXTENSIONS)]
        print(f"    共 {fetched} 个文件 → {dest}；{len(missing)} 个远端缺失（已跳过）")
        if usd_missing:
            print(f"    ⚠️ 其中 USD 文件缺失 {len(usd_missing)} 个，组装时可能报 composition 错误：{usd_missing[:3]}")
    return entry


class _RemoteNotFound(RuntimeError):
    """远端返回 404（多为可选依赖，如 MDL 材质库）。"""


def _fetch_https(url: str, target: Path, *, timeout: float = 60.0) -> None:
    """下载单个文件（校验重定向后仍是 https）。"""
    import requests
    from urllib.parse import urljoin

    current = url
    for _ in range(10):
        if not current.lower().startswith("https://"):
            raise ValueError(f"USD 下载只允许 https：{current}")
        response = requests.get(current, allow_redirects=False, timeout=timeout)
        if int(response.status_code) in (301, 302, 303, 307, 308):
            location = response.headers.get("Location")
            if not location:
                break
            current = urljoin(current, location)
            continue
        if response.status_code == 404:
            raise _RemoteNotFound(current)
        if response.status_code != 200:
            raise RuntimeError(f"下载失败（HTTP {response.status_code}）：{current}")
        target.write_bytes(response.content)
        return
    raise RuntimeError(f"下载失败（重定向过多）：{url}")


def _usd_dependency_paths(file: Path) -> list[str]:
    """列出某个 USD layer 直接依赖的所有资产路径（含 payload 与纹理）。"""
    from pxr import UsdUtils

    try:
        deps = UsdUtils.ComputeAllDependencies(str(file))
    except Exception:  # pragma: no cover - 坏 layer 容错
        return []
    out: list[str] = []
    layers, assets, unresolved = deps[0], deps[1], deps[2]
    for layer in layers:
        if layer is not None and layer.identifier:
            out.append(str(layer.identifier))
    for asset in assets:
        path = getattr(asset, "resolvedPath", None) or getattr(asset, "path", None)
        if path:
            out.append(str(path))
    for entry in unresolved:
        out.append(str(entry))
    return out


def _rebase_dependency(dep: str, referrer: Path, dest: Path, base_url: str) -> tuple[str, str] | None:
    """把一条依赖换算成 (本地相对路径, 下载 URL)；无法镜像到 dest 内则返回 None。"""
    from urllib.parse import urljoin, urlparse

    if dep.lower().startswith(("http://", "https://")):
        digest = hashlib.sha256(dep.encode("utf-8")).hexdigest()[:12]
        rel = posixpath.join("_external", digest, posixpath.basename(urlparse(dep).path) or "asset.usd")
        return rel, dep

    path = Path(dep)
    if not path.is_absolute():
        path = referrer.parent / path
    try:
        rel = path.resolve().relative_to(dest).as_posix()
    except ValueError:
        return None  # 指向 dest 之外（例如系统盘上的共享资产），镜像不了就跳过
    if not rel or rel == ".":
        return None
    return rel, urljoin(base_url, rel)


def autodiscover(cwd: Path | None = None) -> tuple[Path, list[tuple[float, Path, str]]]:
    cwd = Path.cwd() if cwd is None else cwd
    ranked = discover_candidates(cwd)
    if not ranked:
        raise FileNotFoundError(
            f"{cwd} 下没有找到 USD 文件（已跳过 .venv/依赖目录等）。"
            "请显式给出目标 .usda/.usd/.usdz 路径。"
        )
    return ranked[0][1], ranked


def open_stage(path: Path) -> Any:
    _, Usd, _, _, _ = import_usd()
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"无法打开 USD：{path}")
    return stage


# ---------------------------------------------------------------------------
# 自材质碰撞体（NewtonMaterialAPI on a collider）
# ---------------------------------------------------------------------------
#: 从碰撞体复制到独立材质 prim 的属性（材质语义，不含形状语义）
_MATERIAL_ATTR_ALLOW = frozenset(
    {
        "physics:staticFriction",
        "physics:dynamicFriction",
        "physics:restitution",
        "physics:density",
        "physics:frictionCombineMode",
        "physics:restitutionCombineMode",
    }
)
_MATERIAL_SCOPE = "/usd2mjcf_materials"

#: 碰撞体上可能出现的"自带材质"API。不同资产写法不同：
#: Isaac 常见 NewtonMaterialAPI，URDF 转换器常见 PhysicsMaterialAPI，
#: 混用 PhysX 的资产还会附加 PhysxMaterialAPI。三者都会让 Newton 把碰撞体
#: 当成材质定义而跳过碰撞形状。
_SELF_MATERIAL_APIS = ("NewtonMaterialAPI", "PhysicsMaterialAPI", "PhysxMaterialAPI")


def fix_self_material_colliders(stage: Any) -> list[dict[str, str]]:
    """把"自带材质的碰撞体"拆成 **普通碰撞体 + 独立材质 prim**。

    Isaac 的底盘会把驱动轮的碰撞圆柱写成"自材质"（同一个 prim 上既挂
    ``UsdPhysicsCollisionAPI`` 又挂材质 API）。这在 PhysX 里是合法的逐碰撞体
    材质写法，但 **Newton 的 USD 导入器会把这个 prim 当成材质定义，于是完全不导入
    它的碰撞形状** —— 实测底盘四个轮子的碰撞体就是这样消失的，而轮子恰恰是驱动
    轮式机器人最关键的碰撞体。

    材质 API 可能是 ``NewtonMaterialAPI``、``PhysicsMaterialAPI`` 或
    ``PhysxMaterialAPI``，三者都要识别。

    改写方式（语义等价）：
      * 在 ``/usd2mjcf_materials`` 下建一个 ``Material`` prim，把材质类属性复制过去；
      * 原碰撞体挂 ``material:binding`` 指向它，并移除材质 API。

    只针对"同时是碰撞体"的 prim；真正的材质 prim 不受影响。
    """
    _, Usd, _, UsdPhysics, UsdShade = import_usd()
    fixed: list[dict[str, str]] = []
    for prim in iter_prims(stage, include_instance_proxies=False):
        schemas = prim.GetAppliedSchemas() or []
        material_apis = [api for api in _SELF_MATERIAL_APIS if api in schemas]
        if not material_apis:
            continue
        if "PhysicsCollisionAPI" not in schemas:
            continue  # 只处理"自材质碰撞体"，真正的材质 prim 不动
        try:
            material = _make_side_material(stage, prim, Usd, UsdPhysics, UsdShade)
            if material is None:
                continue
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(material)
            prim.RemoveAPI(UsdPhysics.MaterialAPI)
            for api in material_apis:
                prim.RemoveAPI(api)
        except Exception as exc:  # pragma: no cover - 坏 asset 容错
            fixed.append({"collider": str(prim.GetPath()), "status": f"failed: {type(exc).__name__}"})
            continue
        fixed.append(
            {
                "collider": str(prim.GetPath()),
                "material": str(material.GetPath()),
                "status": "moved",
            }
        )
    return fixed


def _make_side_material(stage: Any, collider: Any, Usd: Any, UsdPhysics: Any, UsdShade: Any) -> Any:
    """为自材质碰撞体建一个独立 Material prim 并复制材质属性。"""
    name = re.sub(r"[^0-9A-Za-z_]+", "_", str(collider.GetPath()).strip("/"))
    material = UsdShade.Material.Define(stage, f"{_MATERIAL_SCOPE}/{name}")
    mat_prim = material.GetPrim()
    UsdPhysics.MaterialAPI.Apply(mat_prim)
    try:
        mat_prim.ApplyAPI("NewtonMaterialAPI")
    except Exception:  # pragma: no cover - schema 未注册时也不致命
        pass
    for attr in collider.GetAttributes():
        attr_name = attr.GetName()
        if not attr.HasAuthoredValue():
            continue
        if not (attr_name in _MATERIAL_ATTR_ALLOW or attr_name.startswith("newton:")):
            continue
        value = attr.Get()
        target = mat_prim.CreateAttribute(attr_name, attr.GetTypeName(), custom=False)
        try:
            target.Set(value)
        except Exception:  # pragma: no cover
            continue
    # 即使没有可复制的材质属性，也要返回材质 prim：否则碰撞体会继续带着材质 API，
    # Newton 仍然会跳过它的碰撞形状。默认材质比丢失碰撞体更安全。
    return material


# ---------------------------------------------------------------------------
# 碰撞体使能
# ---------------------------------------------------------------------------
def iter_prims(stage: Any, *, include_instance_proxies: bool = True) -> Iterator[Any]:
    """遍历 stage 上的 prim。

    **必须能走到实例代理**：Isaac 导出的资产常把视觉几何放在 `instanceable=true` 的
    容器下（碰撞容器则显式 `instanceable=false`）；默认遍历看不到这些代理，
    颜色/材质就会全部丢掉。
    """
    _, Usd, _, _, _ = import_usd()
    if not include_instance_proxies:
        for prim in stage.TraverseAll():
            yield prim
        return
    predicate = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), predicate):
        yield prim


# ---------------------------------------------------------------------------
# 碰撞体使能
# ---------------------------------------------------------------------------
COLLISION_MODES = ("enable", "respect")


def apply_collision_mode(stage: Any, mode: str = "enable") -> list[str]:
    """按模式调整 `physics:collisionEnabled`，返回被改动过的 prim 路径。

    Isaac 导出的资产常用"把碰撞体 disable 掉"来表达"自碰撞排除"（例如腰部升降柱在躯干内
    始终重叠）。对 PhysX 这等价于 CollisionGroup 过滤，但对 MuJoCo/Newton 导出而言，
    disable 的碰撞体会被当作 **视觉体** 从而完全不参与接触 —— 整机就没了碰撞。

    * ``enable``  —— 重开所有被禁用的碰撞体（推荐；保住 ``mjc:*`` 参数的同时拿到碰撞）
    * ``respect`` —— 完全按 USD 授权（可能得到零碰撞体的模型）
    """
    if mode not in COLLISION_MODES:
        raise ValueError(f"collision_mode 必须是 {COLLISION_MODES} 之一，收到 {mode!r}")
    if mode == "respect":
        return []
    changed: list[str] = []
    for prim in stage.TraverseAll():
        if "PhysicsCollisionAPI" not in (prim.GetAppliedSchemas() or []):
            continue
        attr = prim.GetAttribute("physics:collisionEnabled")
        try:
            authored = bool(attr and attr.HasAuthoredValue())
            enabled = bool(attr.Get()) if authored else True
        except Exception:  # pragma: no cover - 坏 asset 容错
            continue
        if authored and not enabled:
            attr.Set(True)
            changed.append(prim.GetPath().pathString)
    return changed


def muted_colliders(stage: Any) -> list[str]:
    """列出被资产显式禁用的碰撞体（用于报告：这些是作者表达的自碰撞排除意图）。"""
    out: list[str] = []
    for prim in stage.TraverseAll():
        if "PhysicsCollisionAPI" not in (prim.GetAppliedSchemas() or []):
            continue
        attr = prim.GetAttribute("physics:collisionEnabled")
        try:
            if attr and attr.HasAuthoredValue() and not attr.Get():
                out.append(prim.GetPath().pathString)
        except Exception:  # pragma: no cover
            continue
    return out


# ---------------------------------------------------------------------------
# 整机自碰撞
# ---------------------------------------------------------------------------
SELF_COLLISION_MODES = ("parent_filtered", "respect")

#: 整机自碰撞开关在不同 schema 里的属性名。两者都可能出现，遇到就一起处理。
_SELF_COLLISION_ATTRS = (
    "newton:selfCollisionEnabled",
    "physxArticulation:enabledSelfCollisions",
)


def _path_depth(prim: Any) -> int:
    """返回 prim 路径的层数；`/A` 为 1，`/A/B` 为 2。"""
    return len(prim.GetPath().pathString.strip("/").split("/"))


def _self_collision_targets(stage: Any) -> list[Any]:
    """挑出真正控制整机自碰撞的 prim。

    一个资产里可能有多层 articulation root：整机、左右臂、头部各有一个。
    只有**最浅一层**（离 stage 根最近）的 root 控制整机自碰撞；更深层的开关
    属于部件自己的局部策略，不应被统一覆盖。

    找不到 articulation root API 时退回"所有带自碰撞属性的 prim"，兼容没有
    写 API 的资产。
    """
    prims = list(stage.TraverseAll())
    roots = [
        prim
        for prim in prims
        if any("ArticulationRootAPI" in schema for schema in (prim.GetAppliedSchemas() or []))
    ]
    if roots:
        shallowest = min(_path_depth(prim) for prim in roots)
        return [prim for prim in roots if _path_depth(prim) == shallowest]
    return prims


def apply_self_collision_mode(stage: Any, mode: str = "parent_filtered") -> list[dict[str, Any]]:
    """按模式调整整机自碰撞开关，返回被改动的属性记录。

    资产经常用 ``newton:selfCollisionEnabled = 0`` 或
    ``physxArticulation:enabledSelfCollisions = 0`` 一次性关闭整机自碰撞。
    Newton 会把这种粗粒度开关展开成**所有 body 对**的过滤，导出的 MJCF 就会出现
    "手臂穿过躯干"这类问题 —— 实际语义变成了完全没有自碰撞。

    * ``parent_filtered`` —— 把最浅一层的整机 articulation root 开关改成开启
      （推荐）。MuJoCo 的 ``filterparent`` 默认会过滤父子 body 对，其余 body 对
      （例如手臂与躯干）正常碰撞。部件级 root 的局部开关保持不动。
    * ``respect`` —— 完全按 USD 授权，保留资产的整机自碰撞关闭语义。
    """
    if mode not in SELF_COLLISION_MODES:
        raise ValueError(f"self_collision_mode 必须是 {SELF_COLLISION_MODES} 之一，收到 {mode!r}")
    if mode == "respect":
        return []
    changed: list[dict[str, Any]] = []
    for prim in _self_collision_targets(stage):
        for attr_name in _SELF_COLLISION_ATTRS:
            attr = prim.GetAttribute(attr_name)
            try:
                authored = bool(attr and attr.HasAuthoredValue())
                enabled = bool(attr.Get()) if authored else True
            except Exception:  # pragma: no cover - 坏 asset 容错
                continue
            if not authored or enabled:
                continue
            attr.Set(True)
            changed.append(
                {
                    "prim": prim.GetPath().pathString,
                    "attribute": attr_name,
                    "from": False,
                }
            )
    return changed


#: 凸包顶点上限在三种 schema 里的属性名（顺序 = 优先级）。覆盖时**三个都改**，
#: 免得换了 schema resolver 优先级后又被作者值钉回去。
MAX_HULL_ATTRS = (
    "mjc:maxhullvert",
    "newton:maxHullVertices",
    "physxConvexHullCollision:hullVertexLimit",
)


def override_max_hull_vertices(stage: Any, limit: int) -> list[dict[str, Any]]:
    """把所有碰撞体上**已授权**的凸包顶点上限统一改成 ``limit``。

    为什么不能只传 `add_usd(mesh_maxhullvert=...)`：Newton 的 schema resolver 把
    "作者授权值"排在调用方默认值之前，所以资产里写死的
    ``newton:maxHullVertices`` / ``physxConvexHullCollision:hullVertexLimit``
    会把上限钉在原地。典型症状：几万顶点的底盘碰撞网格被压到 64 顶点后，凸包会
    不对称地切掉底面角部（支撑多边形缺角、支撑面变小）。

    只改**已经存在**的属性；没授权上限的碰撞体由 ``mesh_maxhullvert`` 兜底。
    """
    overridden: list[dict[str, Any]] = []
    for prim in stage.TraverseAll():
        if "PhysicsCollisionAPI" not in (prim.GetAppliedSchemas() or []):
            continue
        for name in MAX_HULL_ATTRS:
            attr = prim.GetAttribute(name)
            try:
                if not (attr and attr.HasAuthoredValue()):
                    continue
            except Exception:  # pragma: no cover - 坏 asset 容错
                continue
            try:
                previous = attr.Get()
            except Exception:  # pragma: no cover
                previous = None
            if previous == limit:
                continue
            attr.Set(limit)
            overridden.append(
                {
                    "prim": prim.GetPath().pathString,
                    "attribute": name,
                    "from": previous,
                    "to": limit,
                }
            )
    return overridden


def save_stage(stage: Any, dest: Path) -> Path:
    """把组合后的 stage 落盘（调试用；同时可给外部工具复用）。"""
    dest.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(dest))
    return dest


def iter_prim_paths(stage: Any) -> Iterator[str]:
    for prim in stage.TraverseAll():
        yield prim.GetPath().pathString


def collect_prim_names(stage: Any) -> dict[str, tuple[str, ...]]:
    """Newton 下划线名 → 该 prim 的**路径分段**（含自身叶子名）。

    Newton 用 ``_`` + prim 路径（``/`` → ``_``）给实体命名，于是 MJCF 里会出现
    ``_Robot_Linkhou_S2_joints_fl_steer_joint``；而 USD 里它就叫 ``fl_steer_joint``。
    保留路径分段是为了在叶子名撞车时能逐步加长（例如左右臂都叫 ``link1`` 时，
    另一侧可以退成 ``Right_Arm_link1``）。

    **必须在 Newton 解析之前调用** —— 导入过程会改写 layer，之后旧 prim 句柄会变成
    过期对象（``UsdExpiredPrimAccessError``）。
    """
    mapping: dict[str, tuple[str, ...]] = {}
    for prim in iter_prims(stage):
        try:
            path = prim.GetPath().pathString
        except Exception:  # pragma: no cover - 坏 asset 容错
            continue
        if not path or path == "/":
            continue
        parts = tuple(p for p in path.split("/") if p and not p.startswith("_Prototype"))
        if not parts:
            continue
        mapping[path.replace("/", "_")] = parts
    return mapping


def collect_leaf_names(stage: Any) -> dict[str, str]:
    """
    兼容入口：只返回叶子名（:func:`collect_prim_names` 的简化形式）。
    """
    return {key: parts[-1] for key, parts in collect_prim_names(stage).items()}
