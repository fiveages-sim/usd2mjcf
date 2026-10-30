"""MJCF 文本层的确定性编辑：`<option>` / `<compiler>` 同步 + 通用补丁引擎。

刻意不用 MjSpec 重新序列化整个模型（那会重写全文、丢失作者排版），
只对 DOM 做"点对点"修改，改动范围可审计。
"""

from __future__ import annotations

import fnmatch
import itertools
import re
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping, Sequence

# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------

_CONTAINER_TAGS = frozenset({"actuator", "equality", "sensor", "contact", "tendon"})

#: 只在 <worldbody> 里找的物理实体（<equality><joint> 与运动学 <joint> 同名）
_WORLDBODY_SCOPED = frozenset({"body", "joint", "geom", "site", "camera", "light"})

#: 只在 <asset> 里找的资产元素（<default><geom> 之类模板不该被当作实体）
_ASSET_SCOPED = frozenset({"mesh", "material", "texture", "hfield", "skin"})

_BOOL_TRUE = {"1", "true", "True", "yes", "on", "enable"}
_BOOL_FALSE = {"0", "false", "False", "no", "off", "disable"}


def fmt_value(value: Any) -> str:
    """把 Python 值写成 MJCF 属性字符串。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return " ".join(fmt_value(v) for v in value)
    if isinstance(value, float):
        return repr(value) if value != int(value) else f"{int(value)}"
    return str(value)


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip()
    if text in _BOOL_TRUE:
        return True
    if text in _BOOL_FALSE:
        return False
    raise ValueError(f"无法解析布尔值：{value!r}")


def load_xml(path: Path) -> ET.ElementTree:
    tree = ET.parse(str(path))
    return tree


def dump_xml(tree: ET.ElementTree, path: Path) -> None:
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(path), encoding="utf-8", xml_declaration=False)


def get_or_create(root: ET.Element, tag: str) -> ET.Element:
    node = root.find(tag)
    if node is None:
        node = ET.SubElement(root, tag)
    return node


# ---------------------------------------------------------------------------
# 场景同步：USD 场景层 → <option> / <compiler> / <flag>
# ---------------------------------------------------------------------------
def apply_scene_settings(
    root: ET.Element,
    *,
    timestep: float | None = None,
    iterations: int | None = None,
    gravity: Sequence[float] | None = None,
    flags: Mapping[str, bool] | None = None,
    option: Mapping[str, Any] | None = None,
    compiler: Mapping[str, Any] | None = None,
    sources: Mapping[str, str] | None = None,
    notes: list[str] | None = None,
) -> list[str]:
    """把场景设置写进 MJCF。

    返回人类可读的改动记录（用于转换报告）。注意 `timestep` 是 Newton 导出时**唯一
    必然丢失**的 option —— `SolverMuJoCo` 从不设置 `spec.option.timestep`。

    ``sources`` 用来在日志里如实标注每个值的来源（``usd`` / ``config``），
    “明明用的是配置兜底值却写成 usd scene”会直接误导排查方向。
    """
    log: list[str] = []
    notes = notes if notes is not None else log
    sources = sources or {}

    def origin(key: str, default: str) -> str:
        src = sources.get(key, default)
        return {"usd": "usd scene", "config": "config fallback"}.get(src, src)

    option_node = get_or_create(root, "option")

    def set_attr(node: ET.Element, key: str, value: Any, src: str) -> None:
        old = node.get(key)
        text = fmt_value(value)
        if old == text:
            notes.append(f"  option.{key} = {text}（已一致，未改动；来源 {src}）")
            return
        node.set(key, text)
        notes.append(f"  option.{key}: {old!r} → {text}（来源 {src}）")

    if timestep is not None and timestep > 0:
        set_attr(option_node, "timestep", timestep, origin("timestep", "config"))
    if iterations is not None and iterations >= 0:
        set_attr(option_node, "iterations", iterations, origin("iterations", "config"))
    if gravity is not None:
        set_attr(option_node, "gravity", [float(g) for g in gravity], origin("gravity", "config"))
    for key, value in (option or {}).items():
        set_attr(option_node, key, value, origin("option", "usd mjc:option"))

    if compiler:
        compiler_node = get_or_create(root, "compiler")
        for key, value in compiler.items():
            old = compiler_node.get(key)
            compiler_node.set(key, fmt_value(value))
            notes.append(
                f"  compiler.{key}: {old!r} → {fmt_value(value)}（来源 {origin('compiler', 'usd mjc:compiler')}）"
            )

    if flags:
        _apply_flags(root, option_node, flags, notes)
    return log


def _apply_flags(
    root: ET.Element, option_node: ET.Element, flags: Mapping[str, bool], notes: list[str]
) -> None:
    """写 `<flag name="enable|disable"/>`。

    优先使用 `<flag>` 子元素（MJCF 官方形式）；如果产物里只有数值形式的
    `enableflags` / `disableflags`，则退化为按位修改。
    """
    flag_node = get_or_create(option_node, "flag")
    numeric_bits: tuple[int, int] | None = None

    for name, value in flags.items():
        token = "enable" if bool(value) else "disable"
        old = flag_node.get(name)
        flag_node.set(name, token)
        notes.append(f"  flag.{name}: {old!r} → {token}（来源 usd mjc:flag）")

    # 若 option 上存在数值化 flags，保持与 <flag> 元素一致
    if option_node.get("enableflags") is not None or option_node.get("disableflags") is not None:
        bits = _mjt_bits()
        if bits is not None and numeric_bits is None:
            enable_bits, disable_bits = bits
            enfl = int(option_node.get("enableflags") or 0)
            disfl = int(option_node.get("disableflags") or 0)
            for name, value in flags.items():
                if name in enable_bits:
                    enfl = enfl | enable_bits[name] if value else enfl & ~enable_bits[name]
                if name in disable_bits:
                    disfl = disfl | disable_bits[name] if not value else disfl & ~disable_bits[name]
            option_node.set("enableflags", str(enfl))
            option_node.set("disableflags", str(disfl))


def _mjt_bits() -> tuple[dict[str, int], dict[str, int]] | None:
    try:
        import mujoco
    except Exception:  # pragma: no cover
        return None
    enable: dict[str, int] = {}
    disable: dict[str, int] = {}
    for name in dir(mujoco.mjtEnableBit):
        if name.startswith("mjENBL_"):
            enable[name[len("mjENBL_") :].lower()] = int(getattr(mujoco.mjtEnableBit, name))
    for name in dir(mujoco.mjtDisableBit):
        if name.startswith("mjDSBL_"):
            disable[name[len("mjDSBL_") :].lower()] = int(getattr(mujoco.mjtDisableBit, name))
    return enable, disable


# ---------------------------------------------------------------------------
# 碰撞掩码修复
# ---------------------------------------------------------------------------
def fix_zero_collision_masks(
    root: ET.Element,
    colliders: Sequence[str],
    *,
    mask: int = 1,
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """把"本应碰撞但掩码被算成 0"的 geom 恢复成可碰撞。

    常见原因：USD 里 `newton:selfCollisionEnabled = 0`（整机自碰撞关闭）时，Newton
    会为该 articulation 的**所有** body 对生成形状级过滤。导出时
    `SolverMuJoCo` 用这些过滤对编译 `contype`/`conaffinity`：

    * 过滤对稀疏（W2：33 对 / 40 个碰撞体）→ 每个形状分到若干位，正常工作；
    * 过滤对稠密（S2：593 对 / 595 组合）→ **每个形状都"谁也不碰"**，掩码全为 0。

    后者会让整机**连地面都不碰**（MuJoCo 里 `contype=0` = 不参与任何接触），模型看起来
    像浮在空中、一放就穿地。

    正确语义是：「与**环境**碰撞，但**自身**各 body 之间不碰」—— MuJoCo 里应该用
    `contype=conaffinity=<mask>` + `<contact><exclude>`（Newton 已经把这些 body 对写成
    exclude 了）。所以这里只把**真正的碰撞体**的掩码从 0 恢复成 ``mask``，视觉体不动。

    注意：这个函数只修 **0** 掩码。Newton 编译出的非零位掩码由
    :func:`normalize_collision_masks` 在不改变碰撞关系时统一成 1/1。

    ``colliders`` 是 Newton 模型里带 ``COLLIDE_SHAPES`` 标志的 shape label 集合
    （见 :func:`convert.collider_labels`）。XML 里的 geom 名可能带 ``_N`` 去重后缀，
    所以匹配时同时尝试去掉尾部 ``_<digits>``。
    """
    log: list[str] = notes if notes is not None else []
    labels = set(colliders)
    if not labels:
        return {"fixed": [], "candidates": 0}

    suffix = re.compile(r"^(?P<base>.+)_(?P<idx>\d+)$")

    def is_collider(name: str) -> bool:
        if name in labels:
            return True
        match = suffix.match(name)
        return bool(match and match.group("base") in labels)

    fixed: list[str] = []
    candidates = 0
    for geom in _iter_targets(root, "geom"):
        name = geom.get("name")
        if not name or not is_collider(name):
            continue
        candidates += 1
        contype = geom.get("contype")
        conaffinity = geom.get("conaffinity")
        already_zero = contype in (None, "0") and conaffinity in (None, "0")
        if not already_zero:
            continue
        geom.set("contype", str(mask))
        geom.set("conaffinity", str(mask))
        fixed.append(name)

    if fixed:
        log.append(
            f"  碰撞掩码修复：{len(fixed)} 个碰撞体的 contype/conaffinity 为 0"
            f"（Newton 编译碰撞图时算成 0，连地面都不碰），已恢复为 {mask}/{mask}；"
            "自碰撞排除由 <contact><exclude> 表达。"
        )
    return {"fixed": fixed, "candidates": candidates}


# ---------------------------------------------------------------------------
# 额外碰撞排除（<contact><exclude>）
# ---------------------------------------------------------------------------
def add_contact_excludes(
    root: ET.Element,
    pairs: Sequence[Sequence[str]],
    *,
    notes: list[str] | None = None,
    strict: bool = False,
) -> dict[str, Any]:
    """按 body 名对追加 `<contact><exclude>`，并检测无效/重复项。

    为什么需要它：USD 的自碰撞排除是**形状级**（`physics:filteredPairs` /
    `CollisionGroup` / `collisionEnabled=0`）。Newton 会把它推导成 body 级 exclude，
    但推导规则是"**所有** shape 组合都被过滤才算一对"（`_find_body_collision_filter_pairs`）。
    于是：
      * 有多个碰撞体的 body（例如底盘 `base_footprint` 同时有外壳与碰撞壳），只要有一对
        shape 没被过滤，整对 body 就不会被排除；
      * 轮子被 `fix_self_material_colliders` 重新打开后，它与底盘/转向节的形状组合
        变成"可碰撞"，body 级 exclude 也就不会生成。
    这类"物理上必然重叠、但 USD 没表达成形状级过滤"的部位，用配置补最省事。

    ``pairs`` 里每项是 ``[body1, body2]``，两侧都支持 glob（`*` `?`），会展开成所有
    匹配的 body 组合。已存在的 exclude 会跳过；body 名不存在会**显式警告**
    （`strict=True` 时报错）——静默不生效是本项目最忌讳的失败模式。
    """
    log: list[str] = notes if notes is not None else []
    bodies = [b.get("name") for b in _iter_targets(root, "body") if b.get("name")]
    body_set = set(bodies)

    contact = get_or_create(root, "contact")
    existing: set[frozenset[str]] = set()
    for node in contact:
        if node.tag != "exclude":
            continue
        b1, b2 = node.get("body1"), node.get("body2")
        if b1 and b2:
            existing.add(frozenset((b1, b2)))

    added: list[str] = []
    skipped: list[str] = []
    unknown: list[str] = []

    for pair in pairs:
        if isinstance(pair, str) or len(pair) != 2:
            raise ValueError(f"contact.excludes 每项必须是 [body1, body2]，收到 {pair!r}")
        left, right = str(pair[0]), str(pair[1])
        lmatch = [b for b in bodies if fnmatch.fnmatchcase(b, left)]
        rmatch = [b for b in bodies if fnmatch.fnmatchcase(b, right)]
        for name, pattern in ((left, left), (right, right)):
            if not (lmatch if pattern is left else rmatch):
                unknown.append(name)
        for b1 in lmatch:
            for b2 in rmatch:
                if b1 == b2:
                    continue
                key = frozenset((b1, b2))
                if key in existing:
                    skipped.append(f"{b1} × {b2}")
                    continue
                node = ET.SubElement(contact, "exclude")
                node.set("body1", b1)
                node.set("body2", b2)
                existing.add(key)
                added.append(f"{b1} × {b2}")

    if unknown:
        message = f"  contact.excludes: 这些 body 名在产物里不存在：{sorted(set(unknown))}"
        if strict:
            raise LookupError(message.strip())
        log.append(message + " ⚠️")
    if added:
        log.append(f"  额外碰撞排除：新增 {len(added)} 条 <exclude>（配置指定）。例：{added[:3]}")
    if skipped:
        log.append(f"  额外碰撞排除：{len(skipped)} 条已存在，跳过。例：{skipped[:3]}")
    if not added and not skipped and not unknown:
        log.append("  额外碰撞排除：配置里没有可用的配对")
    return {"added": added, "skipped": skipped, "unknown": sorted(set(unknown))}


# ---------------------------------------------------------------------------
# 清理冗余的接触排除
# ---------------------------------------------------------------------------
def _iter_worldbody_bodies(root: ET.Element) -> Iterator[tuple[ET.Element, str | None]]:
    """遍历 `<worldbody>` 下的 body，产出 (元素, 父 body 名)。"""
    worldbody = root.find("worldbody")
    if worldbody is None:
        return

    def walk(node: ET.Element, parent: str | None) -> Iterator[tuple[ET.Element, str | None]]:
        for child in node:
            if child.tag != "body":
                continue
            name = child.get("name")
            yield child, parent
            yield from walk(child, name)

    yield from walk(worldbody, None)


def _body_direct_geoms(body: ET.Element) -> Iterator[ET.Element]:
    """产出一个 body 自己的 geom；`<frame>` 不建立新 body，需要继续往下看。"""
    for child in body:
        if child.tag == "geom":
            yield child
        elif child.tag == "frame":
            yield from _body_direct_geoms(child)


def _geom_can_collide(geom: ET.Element) -> bool:
    """按 MuJoCo 默认值判断一个 geom 是否能参与接触。

    `contype` / `conaffinity` 都缺省时，MuJoCo 使用 1/1，仍然会碰撞；
    只有两者都显式为 0（或等价）才确定不参与接触。
    """
    contype = geom.get("contype")
    conaffinity = geom.get("conaffinity")
    if contype is None and conaffinity is None:
        return True
    try:
        ct = 1 if contype is None else int(contype)
        ca = 1 if conaffinity is None else int(conaffinity)
    except ValueError:
        return True
    return bool(ct or ca)


def _body_collider_counts(root: ET.Element) -> dict[str, int]:
    """统计每个 body 自己名下可参与接触的 geom 数量。"""
    counts: dict[str, int] = {}
    for body, _parent in _iter_worldbody_bodies(root):
        name = body.get("name")
        if not name:
            continue
        counts[name] = sum(1 for geom in _body_direct_geoms(body) if _geom_can_collide(geom))
    return counts


def prune_redundant_excludes(
    root: ET.Element,
    *,
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """删除 MuJoCo 已经自动处理、或永远不可能碰撞的 `<contact><exclude>`。

    需要清理的两类：

    * **父子 body 对**：MuJoCo 的 ``filterparent``（默认开启）已经过滤它们；
    * **任一方没有可碰撞 geom 的 body 对**：不可能产生接触。

    其余排除（非父子、双方都有碰撞体）保留，它们可能表达资产作者的真实过滤意图，
    例如闭环机构里必然重叠的两个 body。
    """
    log: list[str] = notes if notes is not None else []
    contact = root.find("contact")
    result: dict[str, Any] = {"removed": 0, "parent": 0, "vacuous": 0, "kept": 0}
    if contact is None:
        return result

    filterparent = True
    option = root.find("option")
    if option is not None:
        flag = option.find("flag")
        value = flag.get("filterparent") if flag is not None else None
        if value is not None:
            try:
                filterparent = parse_bool(value)
            except ValueError:
                filterparent = True

    parents = {
        body.get("name"): parent
        for body, parent in _iter_worldbody_bodies(root)
        if body.get("name")
    }
    counts = _body_collider_counts(root)

    removed_parent: list[tuple[str, str]] = []
    removed_vacuous: list[tuple[str, str]] = []
    for node in list(contact):
        if node.tag != "exclude":
            continue
        body1, body2 = node.get("body1"), node.get("body2")
        if not body1 or not body2:
            continue
        if counts.get(body1, 0) == 0 or counts.get(body2, 0) == 0:
            contact.remove(node)
            removed_vacuous.append((body1, body2))
            continue
        if filterparent and (parents.get(body1) == body2 or parents.get(body2) == body1):
            contact.remove(node)
            removed_parent.append((body1, body2))
            continue
        result["kept"] += 1

    result["parent"] = len(removed_parent)
    result["vacuous"] = len(removed_vacuous)
    result["removed"] = len(removed_parent) + len(removed_vacuous)
    if result["removed"]:
        log.append(
            f"  清理冗余 <exclude>：{result['removed']} 条"
            f"（父子 body 对 {result['parent']} 条，无碰撞体 body 对 {result['vacuous']} 条）；"
            f"保留 {result['kept']} 条真实过滤"
        )
    return result


# ---------------------------------------------------------------------------
# 碰撞掩码统一
# ---------------------------------------------------------------------------
def _geom_masks(geom: ET.Element) -> tuple[int, int]:
    """返回 geom 的有效 (contype, conaffinity)；缺省按 MuJoCo 默认的 1/1。"""
    contype = geom.get("contype")
    conaffinity = geom.get("conaffinity")
    try:
        return (
            1 if contype is None else int(contype),
            1 if conaffinity is None else int(conaffinity),
        )
    except ValueError:
        return 1, 1


def _masks_allow(geom1: ET.Element, geom2: ET.Element) -> bool:
    """按 MuJoCo 的位掩码规则判断两个 geom 是否允许接触。"""
    contype1, conaffinity1 = _geom_masks(geom1)
    contype2, conaffinity2 = _geom_masks(geom2)
    return bool((contype1 & conaffinity2) or (contype2 & conaffinity1))


def _contact_excludes(root: ET.Element) -> set[frozenset[str]]:
    """读出 `<contact><exclude>` 的 body 对集合。"""
    contact = root.find("contact")
    if contact is None:
        return set()
    return {
        frozenset((node.get("body1"), node.get("body2")))
        for node in contact
        if node.tag == "exclude" and node.get("body1") and node.get("body2")
    }


def normalize_collision_masks(
    root: ET.Element,
    *,
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """把碰撞掩码统一成 ``contype=1 conaffinity=1`` —— **仅在不改变碰撞关系时**。

    Newton 会把形状级过滤编译成一组位掩码。值是正确的，但同一个模型里每个碰撞体
    的 ``contype``/``conaffinity`` 都可能不同，非常难读。

    这里先比较两种表示：

    1. 当前的位掩码；
    2. 全部写成 1/1，再叠加 ``filterparent``（父子过滤）和现有的
       ``<contact><exclude>``。

    如果两者对**所有非父子、未排除的 body 对**都允许全部 geom 组合，说明位掩码
    只是冗余编码，可以安全统一。否则保留原样，避免丢掉真正的形状级过滤（例如同一
    对 body 里只有部分 geom 需要互斥）。
    """
    log: list[str] = notes if notes is not None else []
    result: dict[str, Any] = {"normalized": False, "geoms": 0, "mismatches": []}

    filterparent = True
    option = root.find("option")
    if option is not None:
        flag = option.find("flag")
        value = flag.get("filterparent") if flag is not None else None
        if value is not None:
            try:
                filterparent = parse_bool(value)
            except ValueError:
                filterparent = True

    parents = {
        body.get("name"): parent
        for body, parent in _iter_worldbody_bodies(root)
        if body.get("name")
    }
    body_geoms: dict[str, list[ET.Element]] = {}
    for body, _parent in _iter_worldbody_bodies(root):
        name = body.get("name")
        if not name:
            continue
        geoms = [geom for geom in _body_direct_geoms(body) if _geom_can_collide(geom)]
        if geoms:
            body_geoms[name] = geoms

    excludes = _contact_excludes(root)
    mismatches: list[dict[str, Any]] = []
    for body1, body2 in itertools.combinations(sorted(body_geoms), 2):
        if frozenset((body1, body2)) in excludes:
            continue
        if filterparent and (parents.get(body1) == body2 or parents.get(body2) == body1):
            continue
        geoms1, geoms2 = body_geoms[body1], body_geoms[body2]
        total = len(geoms1) * len(geoms2)
        allowed = sum(1 for geom1 in geoms1 for geom2 in geoms2 if _masks_allow(geom1, geom2))
        if allowed != total:
            mismatches.append(
                {
                    "body1": body1,
                    "body2": body2,
                    "allowed": allowed,
                    "total": total,
                }
            )

    result["mismatches"] = mismatches
    if mismatches:
        sample = ", ".join(
            f"{item['body1']} × {item['body2']} ({item['allowed']}/{item['total']})"
            for item in mismatches[:3]
        )
        log.append(
            f"  碰撞掩码保持原样：{len(mismatches)} 个非父子 body 对存在形状级过滤，"
            f"统一成 1/1 会改变碰撞关系。例：{sample}"
        )
        return result

    count = 0
    for geoms in body_geoms.values():
        for geom in geoms:
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
            count += 1
    result["normalized"] = True
    result["geoms"] = count
    if count:
        log.append(
            f"  碰撞掩码统一：{count} 个碰撞体写为 contype=1 conaffinity=1"
            "（与当前过滤图等价；父子 body 对由 filterparent 过滤，"
            "其余排除由 <contact><exclude> 表达）"
        )
    return result


# ---------------------------------------------------------------------------
# 补充 <site>（TCP / EEF 等标记点）
# ---------------------------------------------------------------------------
#: 写进 `<site>` 的属性顺序 —— 产物 diff 稳定，便于和手改文件逐行对比
_SITE_ATTR_ORDER: tuple[str, ...] = (
    "name",
    "class",
    "type",
    "group",
    "pos",
    "quat",
    "euler",
    "axisangle",
    "xyaxes",
    "zaxis",
    "fromto",
    "size",
    "rgba",
    "material",
    "ref",
    "user",
)


def add_sites(
    root: ET.Element,
    sites: Sequence[Mapping[str, Any]],
    *,
    notes: list[str] | None = None,
    strict: bool = False,
) -> dict[str, Any]:
    """按配置往指定 body 里追加 `<site>`。

    为什么需要它：Newton 的 MJCF 出口（`save_to_mjcf`）不保留 USD 里的
    `MjcSite` / 变换标记（TCP、EEF、相机光心…），而这些点恰恰是 RL 侧计算
    末端位姿、奖励与观测时最常用的参考系。手改产物不可版本化，所以在配置里补。

    每条规则的结构::

        {"name": "LEFT_TCP_SITE", "body": "left_link7",
         "pos": [0, -0.095, 0], "quat": [0.5, 0.5, -0.5, 0.5],
         "size": 0.005, "rgba": [0.2, 0.9, 0.2, 0.5]}

    * ``body`` 必填，支持 glob（`*` `?`），匹配的是**产物最终里的 body 名**
      （即通用补丁之后的名字，`patch.rename` 的结果也算）；
    * ``name`` 必填且全局唯一，已存在时跳过（重复运行幂等）；
    * ``optional: true``：目标 body 不存在时只记一条日志，不告警 ——
      适合随 variant 变化的部件（例如只有装夹爪时才有的 EEF body）；
    * 其余键原样写进 `<site>`（`pos` / `quat` / `size` / `rgba` …）。

    body 模式匹配到多个 body 时只写入第一个并告警（site 名全局唯一，
    一条规则不可能同时挂到左右两侧；请拆成两条）。
    """
    log: list[str] = notes if notes is not None else []
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF 里没有 <worldbody>，无法补充 <site>")

    bodies = [b for b in worldbody.iter("body") if b.get("name")]
    existing = {s.get("name") for s in worldbody.iter("site") if s.get("name")}
    added: list[str] = []
    skipped: list[str] = []
    unknown: list[str] = []
    ambiguous: list[str] = []

    for index, entry in enumerate(sites):
        if not isinstance(entry, Mapping):
            raise ValueError(f"mjcf.sites[{index}] 必须是对象，收到 {entry!r}")
        pattern = str(entry.get("body", "")).strip()
        name = str(entry.get("name", "")).strip()
        if not pattern or not name:
            raise ValueError(f"mjcf.sites[{index}] 必须同时给出 body 和 name，收到 {dict(entry)!r}")
        optional = bool(entry.get("optional", False))

        matches = [b for b in bodies if fnmatch.fnmatchcase(b.get("name", ""), pattern)]
        if not matches:
            if optional:
                log.append(f"  site {name}: 目标 body {pattern!r} 不存在（optional，跳过）")
                continue
            unknown.append(f"{name} @ {pattern}")
            message = f"  site {name}: 目标 body {pattern!r} 不存在"
            if strict:
                raise LookupError(message.strip())
            log.append(message + " ⚠️")
            continue

        if name in existing:
            skipped.append(name)
            log.append(f"  site {name}: 产物里已存在，跳过")
            continue

        if len(matches) > 1:
            others = [b.get("name") for b in matches[1:]]
            ambiguous.append(f"{name}: {pattern} → {[b.get('name') for b in matches]}")
            message = (
                f"  site {name}: body 模式 {pattern!r} 匹配到 {len(matches)} 个 body，"
                f"只写入第一个 {matches[0].get('name')!r}（其余 {others}）"
            )
            if strict:
                raise LookupError(message.strip())
            log.append(message + " ⚠️")

        parent = matches[0]
        node = ET.Element("site")
        for key in _SITE_ATTR_ORDER:
            value = entry.get(key)
            if value is not None:
                node.set(key, fmt_value(value))
        # 理论上 validate() 已拦下未知键；这里仍兜底写入，避免配置项被静默吞掉
        for key, value in entry.items():
            if str(key).startswith("_") or key in _SITE_ATTR_ORDER or key in ("body", "optional"):
                continue
            node.set(str(key), fmt_value(value))

        # 放在自有 geom/joint 之后、子 body 之前（与 MJCF 常见排版一致）
        insert_at = len(parent)
        for position, child in enumerate(list(parent)):
            if child.tag == "body":
                insert_at = position
                break
        parent.insert(insert_at, node)
        existing.add(name)
        added.append(f"{name} @ {parent.get('name')}")

    if added:
        log.append(f"  补充 <site>：新增 {len(added)} 个（{added}）")
    if skipped:
        log.append(f"  补充 <site>：{len(skipped)} 个已存在，跳过。例：{skipped[:3]}")
    if not added and not skipped and not ambiguous and not unknown:
        log.append("  补充 <site>：配置里没有可用的规则")
    return {
        "added": added,
        "skipped": skipped,
        "unknownBodies": sorted(set(unknown)),
        "ambiguous": ambiguous,
    }


# ---------------------------------------------------------------------------
# 通用补丁引擎
# ---------------------------------------------------------------------------
def _matches(element: ET.Element, rule: Mapping[str, Any], tag: str) -> bool:
    """判断元素是否命中规则里的 `match` 子句。

    支持的匹配键：
      * `name`   —— 元素自身的 name 属性（actuator/joint/geom/body/... 通用）
      * `joint`  —— actuator 的 joint 属性（含 tendon/site 传动）
      * `body`, `body1`, `body2`, `tendon`, `site`, `class`, `type` —— 其余常用属性
    值可用 `*` / `?` 通配。
    """
    for key, pattern in rule.items():
        if key == "tag":
            if pattern != tag:
                return False
            continue
        value = element.get(key)
        if value is None:
            return False
        if isinstance(pattern, (list, tuple)):
            if not any(fnmatch.fnmatchcase(value, str(p)) for p in pattern):
                return False
        elif not fnmatch.fnmatchcase(value, str(pattern)):
            return False
    return True


def _iter_targets(root: ET.Element, tag: str) -> list[ET.Element]:
    """返回某个"实体类别"下的元素列表。

    两个必须处理的坑，否则补丁/改名/统计都会**静默作用在错的对象上**：

    * ``<actuator>`` / ``<equality>`` / ``<sensor>`` / ``<contact>`` / ``<tendon>``
      是**容器**，真正的实体是它们的子元素（``<general>`` / ``<joint>`` ...）；
    * ``<equality><joint .../>`` 与运动学 ``<joint>`` **同名**（mimic 就是这种），
      ``<default><geom .../>`` 之类的模板元素也会被 ``iter()`` 命中。
      所以物理实体一律限定在 ``<worldbody>`` 子树内，资产类限定在 ``<asset>`` 内。
    """
    if tag in ("worldbody", "option"):
        node = root.find(tag)
        return [node] if node is not None else []
    if tag in _CONTAINER_TAGS:
        node = root.find(tag)
        return list(node) if node is not None else []
    if tag in _WORLDBODY_SCOPED:
        worldbody = root.find("worldbody")
        return list(worldbody.iter(tag)) if worldbody is not None else []
    if tag in _ASSET_SCOPED:
        asset = root.find("asset")
        return list(asset.iter(tag)) if asset is not None else []
    return list(root.iter(tag))


def apply_patch(
    root: ET.Element,
    patch: Mapping[str, Any],
    *,
    notes: list[str] | None = None,
    strict: bool = False,
) -> list[str]:
    """应用通用补丁。未命中任何元素的规则会被**显式警告**（不静默失败）。

    规则结构见 `doc/configuration.md` 的“patch 段”与
    `config/fiveages_w2/config.json` 里的 `patch` 段。
    """
    log: list[str] = notes if notes is not None else []
    for section, spec in patch.items():
        if section.startswith("_"):
            continue  # 允许用 "_comment" 之类写注释
        if section in ("option", "compiler", "flags"):
            if section == "option":
                apply_scene_settings(root, option={k: v for k, v in spec.items() if not k.startswith("_")}, notes=log)
            elif section == "compiler":
                apply_scene_settings(root, compiler={k: v for k, v in spec.items() if not k.startswith("_")}, notes=log)
            else:
                node = root.find("option")
                option_node = node if node is not None else get_or_create(root, "option")
                _apply_flags(root, option_node, {k: v for k, v in spec.items() if not k.startswith("_")}, log)
            continue
        if section == "remove":
            _apply_remove(root, spec, log)
            continue
        if section == "add":
            _apply_add(root, spec, log)
            continue
        if section == "rename":
            _apply_rename(root, spec, log)
            continue
        if section in ("actuators", "joints", "geoms", "sites", "bodies", "equalities", "sensors"):
            tag = section[:-1] if section.endswith("s") else section
            if section == "equalities":
                tag = "equality"
            _apply_attribute_rules(root, tag, section, spec, log, strict=strict)
            continue
        raise ValueError(f"未知补丁段：{section!r}")

    return log


def _apply_attribute_rules(
    root: ET.Element,
    tag: str,
    section: str,
    rules: Iterable[Mapping[str, Any]],
    log: list[str],
    *,
    strict: bool,
) -> None:
    for rule in rules:
        match = rule.get("match", {})
        set_attrs = {
            k: v for k, v in rule.items() if k not in ("match", "type", "rename") and not k.startswith("_")
        }
        new_type = rule.get("type")
        count = 0
        for element in _iter_targets(root, tag):
            if not _matches(element, match, tag):
                continue
            count += 1
            old_name = element.get("name", "?")
            if new_type:
                _retag(element, tag, str(new_type), log, old_name)
            for key, value in set_attrs.items():
                old = element.get(key)
                if key in ("kp", "kv", "gear") and new_type:
                    key = _gain_attr_for(str(new_type), key)
                text = fmt_value(value)
                if old == text:
                    continue
                element.set(key, text)
                log.append(f"  patch {tag}[{old_name}].{key}: {old!r} → {text}")
        if count == 0:
            message = f"  patch {section} 规则 {match!r} 未命中任何元素"
            if strict:
                raise LookupError(message.strip())
            log.append(message + " ⚠️（请检查匹配模式）")


_GAIN_ATTR = {
    "position": "kp",
    "velocity": "kv",
    "intvelocity": "kp",
    "motor": "gear",
    "general": "gear",
    "damper": "kv",
}

# 只有 <general> 接受的通用的执行器属性；换成缩写形式（<position>/<velocity>/...）时必须清掉，
# 否则 MuJoCo 编译会直接报 "unrecognized attribute"。
_GENERAL_ONLY_ATTRS = ("gaintype", "gainprm", "biastype", "biasprm", "dyntype", "dynprm")
_SHORTHAND_ACTUATORS = frozenset(_GAIN_ATTR) - {"general"}


def _gain_attr_for(actuator_type: str, key: str) -> str:
    """把通用增益名映射到目标执行器类型的属性名。"""
    if key in ("kp", "kv"):
        return _GAIN_ATTR.get(actuator_type, key)
    return key


def _retag(element: ET.Element, tag: str, new_type: str, log: list[str], name: str) -> None:
    """把 `<position joint="x" kp="..."/>` 改成 `<velocity joint="x" kv="..."/>` 之类。"""
    if tag != "actuator":
        return
    old_type = element.tag
    if old_type == new_type:
        return
    # 把旧类型的增益属性迁移到新类型的属性名
    old_gain = _GAIN_ATTR.get(old_type)
    new_gain = _GAIN_ATTR.get(new_type)
    value = element.attrib.pop(old_gain, None) if old_gain else None
    element.tag = new_type
    if new_gain and value is not None:
        element.set(new_gain, value)
    if new_type in _SHORTHAND_ACTUATORS:
        for attr in _GENERAL_ONLY_ATTRS:
            element.attrib.pop(attr, None)
        for gain in {old_gain, new_gain} - {None}:
            if gain != new_gain:
                element.attrib.pop(gain, None)
        for gain in ("kp", "kv"):
            if gain != new_gain:
                element.attrib.pop(gain, None)
    log.append(f"  patch actuator[{name}]: type {old_type} → {new_type}")


def _apply_remove(root: ET.Element, spec: Mapping[str, Any], log: list[str]) -> None:
    for tag, patterns in spec.items():
        if tag.startswith("_"):
            continue
        if not isinstance(patterns, (list, tuple)):
            patterns = [patterns]
        for element in list(_iter_targets(root, tag)):
            attrs = _REMOVE_MATCH_ATTRS.get(tag, ("name",))
            matched = next(
                (
                    element.get(attr)
                    for attr in attrs
                    if element.get(attr) is not None
                    and any(fnmatch.fnmatchcase(element.get(attr), str(p)) for p in patterns)
                ),
                None,
            )
            if matched is not None:
                parent = _find_parent(root, element)
                if parent is not None:
                    parent.remove(element)
                    log.append(f"  remove {tag} {element.get('name') or matched}")


def _apply_add(root: ET.Element, spec: Mapping[str, Any], log: list[str]) -> None:
    for tag, snippets in spec.items():
        if tag.startswith("_"):
            continue
        if isinstance(snippets, str):
            snippets = [snippets]
        node = root.find(tag)
        if node is None:
            node = ET.SubElement(root, tag)
        for snippet in snippets:
            fragment = ET.fromstring(snippet)
            node.append(fragment)
            log.append(f"  add <{tag}> ← {fragment.tag} {fragment.get('name') or ''}".rstrip())


def _find_parent(root: ET.Element, target: ET.Element) -> ET.Element | None:
    for parent in root.iter():
        for child in list(parent):
            if child is target:
                return parent
    return None


# 各实体的"引用型"属性名：重命名时必须同步，否则产物会指向不存在的名字
_REFERENCE_ATTRS: dict[str, tuple[str, ...]] = {
    "joint": ("joint", "joint1", "joint2", "jointinparent"),
    "body": ("body", "body1", "body2"),
    "geom": ("geom", "geom1", "geom2"),
    "site": ("site", "site1", "site2"),
    "tendon": ("tendon", "tendon1", "tendon2"),
    "actuator": ("actuator",),
    "mesh": ("mesh",),
    "material": ("material",),
    "texture": ("texture",),
}

# remove 规则的匹配属性：既支持按自身 name，也支持按成对引用
# （例如按 joint 名删掉 mimic 的 `<equality><joint .../>`）
_REMOVE_MATCH_ATTRS: dict[str, tuple[str, ...]] = {
    "joint": ("name", "joint", "joint1", "joint2"),
    "body": ("name", "body", "body1", "body2"),
    "geom": ("name", "geom", "geom1", "geom2"),
    "site": ("name", "site", "site1", "site2"),
    "tendon": ("name", "tendon", "tendon1", "tendon2"),
    "exclude": ("name", "body1", "body2"),
    "pair": ("name", "geom1", "geom2"),
    "actuator": ("name", "actuator"),
}


def _rename_tag(root: ET.Element, tag: str, renames: Mapping[str, str], log: list[str]) -> int:
    """按 ``{旧名: 新名}`` 改名，并同步所有引用。

    重名会**直接报错**：静默产生两个同名 joint/body 会让产物变成另一种模型。
    """
    if not renames:
        return 0
    existing = {el.get("name") for el in _iter_targets(root, tag)}
    for old, new in renames.items():
        if new in existing and new not in renames:
            raise ValueError(f"rename {tag}: {old!r} → {new!r} 与已有名称冲突")

    ref_attrs = _REFERENCE_ATTRS.get(tag, ())
    for element in root.iter():
        for attr, value in list(element.attrib.items()):
            if attr == "name" and element.tag == tag:
                if value in renames:
                    element.set(attr, renames[value])
            elif attr in ref_attrs and value in renames:
                element.set(attr, renames[value])
    log.append(f"  rename {tag}: {len(renames)} 个名字已重写（含引用）")
    return len(renames)


def _apply_rename(root: ET.Element, spec: Mapping[str, Any], log: list[str]) -> None:
    """按正则重命名实体，并同步所有引用。

    规则形如::

        {"rename": {"joint": {"pattern": "^_Robot_", "replacement": ""}}}

    也可以给**一串规则**，按顺序依次施加（后面的规则看到的是前面的结果）::

        {"rename": {"body": [
            {"pattern": "^Left_Arm_link7_tcp_AG2F90_", "replacement": "left_gripper_"},
            {"pattern": "^Left_Arm_", "replacement": "left_"}
        ]}}

    替换串支持 Python 正则的反向引用（JSON 里写 ``"\\\\1"``）。
    """

    for tag, rules in spec.items():
        if tag.startswith("_"):
            continue
        if isinstance(rules, Mapping):
            rules = [rules]
        for rule in rules:
            if not isinstance(rule, Mapping):
                raise ValueError(f"rename.{tag} 的规则必须是对象：{rule!r}")
            pattern = rule.get("pattern")
            repl = rule.get("replacement", "")
            if pattern is None:
                raise ValueError(f"rename.{tag} 缺少 pattern")
            rx = re.compile(pattern)
            renames: dict[str, str] = {}
            for element in _iter_targets(root, tag):
                name = element.get("name")
                if not name:
                    continue
                new = rx.sub(repl, name)
                if new and new != name:
                    renames[name] = new
            if not renames:
                log.append(f"  rename {tag}: 规则 {pattern!r} 未命中任何名字 ⚠️")
                continue
            _rename_tag(root, tag, renames, log)


def apply_name_map(
    root: ET.Element,
    mapping: Mapping[str, Sequence[str]],
    tags: Sequence[str] = ("joint", "body"),
    *,
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """把 Newton 生成的“下划线路径名”换回 **USD prim 的叶子名**。

    ``mapping`` 由 :func:`usd_scene.collect_prim_names` 给出：下划线名 → 路径分段
    （如 ``("FiveAges_W2", "Linkhou_S2", "joints", "fl_steer_joint")``）。

    命名规则（可预测、可复现）：

    1. 优先用叶子名（``fl_steer_joint``、``body_joint1``）；
    2. 叶子名撞车时**逐步加长路径尾部**，直到组内互不相同且不与已占用名冲突
       （左右臂的 ``link1`` → 对称的 ``Left_Arm_link1`` / ``Right_Arm_link1``）；
    3. 仍无法唯一时保留原名，并在报告里列出来；
    4. **名字不在映射表里的（Newton 自己合成的，例如 float base 的 ``joint_1``）
       一律不动**，但会记进 ``unmatched``，提示用 ``patch.rename`` 正则处理。

    路径越浅的元素优先拿到短名，保证根/关键部件名字最干净。
    """
    log: list[str] = notes if notes is not None else []
    stats: dict[str, Any] = {
        "renamed": {},
        "lengthened": {},
        "kept": {},
        "unmatched": {},
        "freeJoints": [],
    }
    for tag in tags:
        elements = _iter_targets(root, tag)
        wanted: list[tuple[str, Sequence[str]]] = []
        unmatched: list[str] = []
        for element in elements:
            name = element.get("name")
            if not name:
                continue
            if name not in mapping:
                # Newton 合成的名字（prim 路径里查不到）：不动它，但报告出来
                unmatched.append(name)
                continue
            parts = tuple(mapping[name])
            if len(parts) < 2 or (len(parts) == 1 and parts[0] == name):
                continue  # 已经没有信息量可压缩
            wanted.append((name, parts))
        stats["unmatched"][tag] = unmatched
        if unmatched:
            log.append(
                f"  naming {tag}: {len(unmatched)} 个名字不在 USD prim 表里（Newton 合成名），"
                "保持原样：" + str(unmatched[:3]) + "；需要改就用 patch.rename 正则"
            )
        if not wanted:
            continue

        pending = {name for name, _ in wanted}
        # 不在本批次里的元素名先占位，避免改名后撞上它们
        taken = {el.get("name") for el in elements if el.get("name")} - pending

        # 按叶子名分组：**同组要么都用短名，要么都加长**，否则左右臂会不对称
        groups: dict[str, list[tuple[str, Sequence[str]]]] = {}
        for name, parts in wanted:
            groups.setdefault(parts[-1], []).append((name, parts))

        renames: dict[str, str] = {}
        lengthened: list[str] = []
        kept: list[str] = []
        # 叶子名唯一的组先拿短名（浅路径优先，保证根/关键部件名字最干净）
        ordered = sorted(groups.items(), key=lambda item: (len(item[1]), min(len(p) for _, p in item[1]), item[0]))
        for leaf, members in ordered:
            by_depth = sorted(members, key=lambda item: (len(item[1]), item[0]))
            min_depth = len(by_depth[0][1])
            shallow = [m for m in by_depth if len(m[1]) == min_depth]
            grow = [m for m in by_depth if len(m[1]) > min_depth]

            # 1) 叶子名可以先给「唯一最浅」的那一个：根 link / 关键部件名字最干净
            if len(shallow) == 1 and leaf not in taken:
                name, _parts = shallow[0]
                renames[name] = leaf
                taken.add(leaf)
            else:
                # 并列最浅 = 对称件（左右臂/左右轮），必须同进同退，
                # 否则会出现 link1 / Right_Arm_link1 这种不对称命名
                grow = shallow + grow

            # 2) 剩下的成员整体挑一个公共长度：组内互不相同、且不与已占用名冲突
            if not grow:
                continue
            max_len = max(len(parts) for _, parts in grow)
            for length in range(1, max_len + 1):
                candidates = ["_".join(parts[-length:]) for _, parts in grow]
                if len(set(candidates)) != len(candidates):
                    continue  # 组内自撞，继续加长
                if any(candidate in taken for candidate in candidates):
                    continue  # 撞上已占用的名字，继续加长
                for (name, _parts), candidate in zip(grow, candidates):
                    renames[name] = candidate
                    taken.add(candidate)
                    if candidate != leaf:
                        lengthened.append(f"{name} → {candidate}")
                break
            else:
                kept.extend(name for name, _parts in grow)
        renamed = _rename_tag(root, tag, renames, log)
        stats["renamed"][tag] = renamed
        stats["lengthened"][tag] = lengthened
        stats["kept"][tag] = kept
        if lengthened:
            log.append(
                f"  naming {tag}: {len(lengthened)} 个名称因叶子名撞车而加长路径：{lengthened[:3]}"
            )
        if kept:
            log.append(f"  naming {tag}: {len(kept)} 个名称无法消歧，保留原样：{kept[:3]}")

    for joint in root.iter("joint"):
        if (joint.get("type") or "") == "free":
            stats["freeJoints"].append(joint.get("name"))
    return stats


# ---------------------------------------------------------------------------
# 统计
# ---------------------------------------------------------------------------
def summarize(root: ET.Element) -> dict[str, Any]:
    """给转换报告用的基本统计（走作用域化遍历，避免把 equality/pair 当成实体）。"""
    return {
        "bodies": len(_iter_targets(root, "body")) + 1,  # +1 = worldbody
        "joints": len(_iter_targets(root, "joint")),
        "freeJoints": sum(
            1 for j in _iter_targets(root, "joint") if (j.get("type") or "") == "free"
        ),
        "geoms": len(_iter_targets(root, "geom")),
        "collidingGeoms": sum(
            1 for g in _iter_targets(root, "geom") if g.get("contype") not in (None, "0")
        ),
        "meshes": len(_iter_targets(root, "mesh")),
        "actuators": len(_iter_targets(root, "actuator")),
        "equalities": len(_iter_targets(root, "equality")),
        "excludes": sum(1 for e in _iter_targets(root, "contact") if e.tag == "exclude"),
        "sites": len(_iter_targets(root, "site")),
        "materials": len(_iter_targets(root, "material")),
    }


def external_mesh_refs(root: ET.Element, xml_dir: Path) -> list[tuple[str, bool]]:
    """列出产物引用的外部网格，返回 (引用路径, 文件是否存在)。"""
    out: list[tuple[str, bool]] = []
    for mesh in root.iter("mesh"):
        asset = mesh.find("asset")
        ref = mesh.get("file") or (asset.get("file") if asset is not None else None)
        if not ref:
            continue
        candidate = Path(ref)
        if not candidate.is_absolute():
            candidate = xml_dir / ref
        out.append((ref, candidate.is_file()))
    return out


def clone(root: ET.Element) -> ET.Element:
    return deepcopy(root)


# ---------------------------------------------------------------------------
# 材质写入
# ---------------------------------------------------------------------------
def apply_materials(
    root: ET.Element,
    report: Any,
    *,
    pbr: bool = False,
    notes: list[str] | None = None,
) -> list[str]:
    """把 `materials.MaterialReport` 写进 MJCF。

    只负责“颜色/材质”。碰撞体与视觉体的分组是独立的一步
    （:func:`separate_visual_collision`），因为它必须等**碰撞掩码定稿之后**才能做。

    ``pbr=False``（默认）只写 ``rgba``：OmniPBR 资产常带 ``metallic=1``，在 MuJoCo 的
    默认灯光下会渲染成近黑色；需要 PBR 一致时再开 ``pbr=True``。
    """
    log: list[str] = notes if notes is not None else []
    if not report.materials:
        log.append("  材质：USD 里没有可解析的颜色/材质，跳过")
        return log

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(0, asset)

    texture_for_material: dict[str, str] = {}
    for index, (name, info) in enumerate(sorted(report.materials.items())):
        if info.texture:
            tex_name = f"tex{index}"
            tex = ET.SubElement(asset, "texture")
            tex.set("name", tex_name)
            tex.set("type", "2d")
            tex.set("file", str(info.texture))
            texture_for_material[name] = tex_name
        material = ET.SubElement(asset, "material")
        material.set("name", name)
        material.set("rgba", fmt_value([round(float(c), 4) for c in info.rgba]))
        if name in texture_for_material:
            material.set("texture", texture_for_material[name])
        if pbr and info.metallic is not None:
            material.set("metallic", fmt_value(float(info.metallic)))
        if pbr and info.roughness is not None:
            material.set("roughness", fmt_value(float(info.roughness)))
    log.append(
        f"  材质：写入 {len(report.materials)} 个 <material>"
        f"（{len(report.textures_copied)} 张贴图）；"
        f"USD 侧扫描 {report.prims_scanned} 个可着色 prim"
    )

    renamed = 0
    for geom in root.iter("geom"):
        name = geom.get("name")
        if not name:
            continue
        material = report.resolve(name)
        if material:
            geom.set("material", material)
            renamed += 1
    log.append(f"  材质：{renamed} 个 geom 通过 prim 路径匹配到 USD 材质")
    return log


# ---------------------------------------------------------------------------
# 视觉体 / 碰撞体分组
# ---------------------------------------------------------------------------
#: MuJoCo 约定：0/1/2 是渲染组（默认可见），3 及以后默认不可见。
#: 把碰撞体放进 3 组，既能避免与视觉网格 z-fighting，又能随时在 viewer 里单独打开。
COLLISION_GROUP = 3
VISUAL_GROUP = 0
COLLIDER_RGBA = (0.35, 0.35, 0.38, 1.0)


def _is_collision_intent(geom: ET.Element) -> bool:
    """判断一个 geom 是否真的参与碰撞：只看有效掩码。

    掩码为 0/0 的几何不产生任何接触，必须按视觉体处理；名字里带 `collisions`
    不能作为碰撞依据 —— 资产常把相机外壳、装饰件也放在 `collisions` 目录下，
    这些 prim 没有 ``PhysicsCollisionAPI``，Newton 只把它们当成视觉网格。

    USD 里被 ``physics:collisionEnabled=0`` 禁用的真正碰撞体，会在导入前由
    ``collision_mode=enable`` 重新打开；如果最终仍是 0/0，说明它不是碰撞体。
    """
    return geom.get("contype") not in (None, "0") or geom.get("conaffinity") not in (None, "0")


def separate_visual_collision(
    root: ET.Element,
    *,
    hide_colliders: bool = True,
    collider_group: int = COLLISION_GROUP,
    visual_group: int = VISUAL_GROUP,
    collider_rgba: Sequence[float] = COLLIDER_RGBA,
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """把视觉体与碰撞体归到不同的 MuJoCo geom group。

    * **碰撞体** → ``group=3``（默认不渲染）+ 中性灰（无材质时），避免凸包盖在视觉网格上；
    * **视觉体** → ``group=0``（默认渲染）。

    为什么必须单独成一步：判断“谁是碰撞体”依赖 `contype/conaffinity`，而这两个值在
    USD 关闭自碰撞时会被 Newton 算成 0，需要先由
    :func:`fix_zero_collision_masks` 修复 —— 顺序反了就会把碰撞体当成视觉体（S2 就是
    这样把碰撞盒/碰撞圆柱暴露出来的）。

    分类只看有效掩码：0/0 的几何即使名字里带 `collisions` 也按视觉体处理。
    """
    log: list[str] = notes if notes is not None else []
    colliders: list[str] = []
    visuals: list[str] = []
    recolored = 0

    for geom in _iter_targets(root, "geom"):
        name = geom.get("name") or "<unnamed>"
        if _is_collision_intent(geom):
            colliders.append(name)
            if not hide_colliders:
                continue
            geom.set("group", str(collider_group))
            if geom.get("material") is None and geom.get("rgba") is None:
                geom.set("rgba", fmt_value([float(c) for c in collider_rgba]))
                recolored += 1
        else:
            visuals.append(name)
            if geom.get("group") is not None and geom.get("group") != str(visual_group):
                geom.set("group", str(visual_group))

    if hide_colliders:
        log.append(
            f"  视觉/碰撞分离：{len(colliders)} 个碰撞体 → group={collider_group}"
            f"（默认不渲染），{len(visuals)} 个视觉体 → group={visual_group}；"
            f"其中 {recolored} 个无材质碰撞体给了中性灰。"
        )
    else:
        log.append(f"  视觉/碰撞分离已关闭：{len(colliders)} 个碰撞体仍参与渲染")
    return {
        "colliders": len(colliders),
        "visuals": len(visuals),
        "recolored": recolored,
        "hidden": bool(hide_colliders),
    }


# ---------------------------------------------------------------------------
# geom 统一命名
# ---------------------------------------------------------------------------
#: geom 命名方式：kind=按视觉体/碰撞体生成短名；usd=保留 Newton 生成的 prim 路径名
GEOM_NAME_MODES = ("kind", "usd")


def collision_geom_ids(root: ET.Element) -> set[int]:
    """返回当前被判定为碰撞体的 geom 对象标识集合。

    统一命名发生在流程末尾，此时 geom 名字可能已经被名字还原或补丁改写，
    再靠名字里是否带碰撞路径来判断会出错。调用方在视觉/碰撞分组的时刻记下
    这张表，分类就能保持稳定。
    """
    return {
        id(geom)
        for geom in _iter_targets(root, "geom")
        if _is_collision_intent(geom)
    }


def _geom_kind(geom: ET.Element, collider_ids: set[int] | None) -> str:
    """判定一个 geom 是 ``VISUAL`` 还是 ``COLLISION``。

    ``collider_ids`` 是视觉/碰撞分组时记下的碰撞体集合。快照里的元素按快照判定
    （即使后来名字或掩码被改写）；快照之外的元素（例如补丁新加的几何）回退到
    当前的掩码和名字判断。
    """
    if collider_ids is not None and id(geom) in collider_ids:
        return "COLLISION"
    return "COLLISION" if _is_collision_intent(geom) else "VISUAL"


def _body_geom_pairs(root: ET.Element) -> list[tuple[ET.Element, str]]:
    """按文档顺序返回 (geom, 所属 body 名)。

    worldbody 直属的几何（例如地面）归入 ``world``；frame 是容器，递归进去但
    不改变所属 body。
    """
    worldbody = root.find("worldbody")
    pairs: list[tuple[ET.Element, str]] = []
    if worldbody is None:
        return pairs

    def walk(node: ET.Element, body_name: str) -> None:
        for child in node:
            if child.tag == "body":
                walk(child, child.get("name") or body_name)
            elif child.tag == "geom":
                pairs.append((child, body_name))
            elif child.tag == "frame":
                walk(child, body_name)

    walk(worldbody, "world")
    return pairs


def normalize_geom_names(
    root: ET.Element,
    *,
    mode: str = "kind",
    collider_ids: set[int] | None = None,
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """把 geom 名字统一成 ``VISUAL_<body>_<序号>`` / ``COLLISION_<body>_<序号>``。

    序号按“同一个 body + 同一类几何”各自从 0 开始，顺序与文档一致，因此转换
    结果可复现。mesh 名字保持不变，geom 上的 ``mesh=`` 引用也不动。

    判断视觉体还是碰撞体只看有效掩码（见 :func:`_is_collision_intent`）。
    如果在视觉/碰撞分组时已经记下了碰撞体集合（``collider_ids``），就优先用它，
    避免后续改名影响判断。这一步必须放在补丁之后，补丁可能重命名 body，而 geom 名
    要使用最终的 body 名。

    ``mode="usd"`` 时保持 Newton 生成的 prim 路径名，只返回统计。
    """
    log: list[str] = notes if notes is not None else []
    if mode not in GEOM_NAME_MODES:
        raise ValueError(f"geom 命名方式必须是 {GEOM_NAME_MODES} 之一，收到 {mode!r}")

    stats: dict[str, Any] = {"mode": mode, "visual": 0, "collision": 0, "renamed": 0}
    pairs = _body_geom_pairs(root)
    if mode == "usd":
        for geom, _body in pairs:
            stats[_geom_kind(geom, collider_ids).lower()] += 1
        log.append("  geom 命名：保留 prim 路径名（mode=usd）")
        return stats

    used = {geom.get("name") for geom in root.iter("geom") if geom.get("name")}
    counters: dict[tuple[str, str], int] = {}
    renames: dict[str, str] = {}
    unnamed = 0

    for geom, body in pairs:
        kind = _geom_kind(geom, collider_ids)
        index = counters.get((body, kind), 0)
        counters[(body, kind)] = index + 1

        candidate = f"{kind}_{body}_{index}"
        suffix = 1
        while candidate in used and candidate != geom.get("name"):
            candidate = f"{kind}_{body}_{index}_{suffix}"
            suffix += 1
        used.add(candidate)

        stats[kind.lower()] += 1
        old = geom.get("name")
        if old is None:
            geom.set("name", candidate)
            unnamed += 1
        elif old != candidate:
            renames[old] = candidate

    renamed = _rename_tag(root, "geom", renames, log) if renames else 0
    stats["renamed"] = renamed + unnamed
    log.append(
        f"  geom 命名：{stats['visual']} 个视觉体 → VISUAL_<body>_<序号>，"
        f"{stats['collision']} 个碰撞体 → COLLISION_<body>_<序号>"
        f"（共改写 {stats['renamed']} 个名字）"
    )
    return stats


# ---------------------------------------------------------------------------
# 执行器命名
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 执行器：ctrlrange 单位修正 + 缺失时按关节限位推导
# ---------------------------------------------------------------------------
RAD_PER_DEG = 0.017453292519943295


def _joint_ranges(root: ET.Element) -> dict[str, tuple[float, float, bool]]:
    """joint 名 → (lower, upper, limited)。"""
    out: dict[str, tuple[float, float, bool]] = {}
    for joint in _iter_targets(root, "joint"):
        name = joint.get("name")
        if not name:
            continue
        limited = (joint.get("limited") or "").lower() in ("true", "1")
        raw = joint.get("range")
        if not raw:
            continue
        parts = raw.split()
        if len(parts) != 2:
            continue
        try:
            out[name] = (float(parts[0]), float(parts[1]), limited)
        except ValueError:
            continue
    return out


def _joint_span_deg_ratio(ctrl: tuple[float, float], jnt: tuple[float, float]) -> float:
    """ctrlrange 与关节 range 的跨度比（用于判断单位）。"""
    cspan = ctrl[1] - ctrl[0]
    jspan = jnt[1] - jnt[0]
    if jspan <= 0 or cspan <= 0:
        return 1.0
    return cspan / jspan


def fix_actuator_ctrlrange(
    root: ET.Element,
    *,
    auto_convert_degrees: bool = True,
    fill_missing_from_joint: bool = True,
    tolerance: float = 5.0,
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """修正执行器 `ctrlrange` 的两个常见问题。

    **1) 单位混用（度 vs 弧度）** —— USD 里 `mjc:ctrlRange` 由作者手填，同一个资产里
    常混着两种单位：底盘转向轮写的是弧度（±π），而腰部/头部/手臂/夹爪写的是**度**
    （`-77.5 .. 32.5`、`±178`）。MJCF 是 `angle="radian"`，直接照抄会让控制范围放大
    57.3 倍 —— 策略输出的目标值落在物理限位之外，伺服会一直顶到力矩上限。

    判据：`ctrlrange 跨度 / 关节 range 跨度 ≈ 180/π`（±`tolerance`%）。只在**明显**是
    度的时候才换算，避免把本来就用弧度的值误改。

    **2) `ctrllimited=false` 时 ctrlrange 为空** —— 例如夹爪执行器在 USD 里写了
    `ctrlLimited=true, ctrlRange=0..57.3`，但 Newton 因为关节本身 `limited` 组合的原因
    没把它带过来，产物里就成了 `ctrlrange="0 0"` 且 `ctrllimited="false"`，等于**控制
    量完全不限幅**（`ctrl` 直接当目标角度用）。这里从关节限位补出来。

    返回统计，写进转换报告。
    """
    log: list[str] = notes if notes is not None else []
    joints = _joint_ranges(root)
    converted: list[str] = []
    filled: list[str] = []
    skipped: list[str] = []

    for actuator in _iter_targets(root, "actuator"):
        joint_name = actuator.get("joint")
        if not joint_name or joint_name not in joints:
            continue
        jlo, jhi, jlimited = joints[joint_name]
        if not jlimited or jhi <= jlo:
            continue
        raw = actuator.get("ctrlrange")
        ctrllimited = (actuator.get("ctrllimited") or "").lower() in ("true", "1")

        if raw:
            parts = raw.split()
            if len(parts) != 2:
                continue
            try:
                clo, chi = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            if not auto_convert_degrees or chi <= clo:
                continue
            ratio = _joint_span_deg_ratio((clo, chi), (jlo, jhi))
            if abs(ratio - 180.0 / 3.141592653589793) <= (180.0 / 3.141592653589793) * tolerance / 100.0:
                actuator.set(
                    "ctrlrange",
                    fmt_value([round(clo * RAD_PER_DEG, 6), round(chi * RAD_PER_DEG, 6)]),
                )
                converted.append(f"{joint_name}: {clo:g}..{chi:g} → {clo * RAD_PER_DEG:.4f}..{chi * RAD_PER_DEG:.4f}")
            continue

        # ctrlrange 缺失：只有"限幅本应生效"时才补，避免把速度伺服/力矩源改成位置限幅
        if fill_missing_from_joint and ctrllimited:
            actuator.set("ctrlrange", fmt_value([jlo, jhi]))
            filled.append(f"{joint_name}: 补为关节限位 {jlo:.4f}..{jhi:.4f}")
        elif fill_missing_from_joint and (actuator.get("ctrllimited") is None):
            # ctrllimited 未授权时，位置伺服默认就该限幅
            if actuator.tag in ("position", "intvelocity") or (
                actuator.get("biastype") == "affine" and (actuator.get("gainprm") or "") != ""
            ):
                actuator.set("ctrlrange", fmt_value([jlo, jhi]))
                actuator.set("ctrllimited", "true")
                filled.append(f"{joint_name}: 补为关节限位 {jlo:.4f}..{jhi:.4f}（并开启 ctrllimited）")
        else:
            skipped.append(joint_name)

    if converted:
        log.append(
            f"  执行器 ctrlrange 单位修正：{len(converted)} 个由「度」换算为弧度"
            f"（判据：跨度比 ≈ 180/π）。例：{converted[0]}"
        )
    if filled:
        log.append(f"  执行器 ctrlrange 补齐：{len(filled)} 个从关节限位推导。例：{filled[0]}")
    if skipped:
        log.append(f"  执行器 ctrlrange：{len(skipped)} 个未限幅（ctrllimited=false，按原样保留）")
    return {"converted": converted, "filled": filled, "unlimited": skipped}


# ---------------------------------------------------------------------------
# 执行器命名
# ---------------------------------------------------------------------------
def name_actuators(root: ET.Element, mode: str = "joint", *, notes: list[str] | None = None) -> int:
    """给 `<actuator>` 里的每个执行器补 `name`。

    Newton 的 MJCF 出口不写执行器名，而 RL 侧（mjlab / Isaac Lab 风格配置）几乎都要
    按名字索引执行器，所以这里用**它驱动的 joint 名**来命名；同一个 joint 有多个执行器
    （例如 POSITION_VELOCITY 会生成两个）时追加 `_0`/`_1` 保证唯一。

    返回命名的执行器数量。
    """
    log: list[str] = notes if notes is not None else []
    if mode == "none":
        log.append("  执行器命名：已关闭")
        return 0
    container = root.find("actuator")
    if container is None:
        return 0
    used: set[str] = {a.get("name") for a in container if a.get("name")}
    named = 0
    for actuator in container:
        if actuator.get("name"):
            continue
        target = (
            actuator.get("joint")
            or actuator.get("tendon")
            or actuator.get("site")
            or actuator.get("body")
            or actuator.tag
        )
        base = _short_name(target)
        candidate = base
        suffix = 0
        while candidate in used:
            candidate = f"{base}_{suffix}"
            suffix += 1
        actuator.set("name", candidate)
        used.add(candidate)
        named += 1
    log.append(f"  执行器命名：{named} 个执行器按所驱动 joint 命名（mode={mode}）")
    return named


def _short_name(target: str) -> str:
    """把 USD 路径风格的名字压成可读的短名（保留可辨识性）。"""
    text = str(target).strip("/")
    if "/" not in text:
        return text
    parts = text.split("/")
    # 形如 <Robot>/<Part>/joints/<joint_name> 时取最后一段
    if len(parts) >= 2 and parts[-2] in ("joints", "actuators"):
        return parts[-1]
    return parts[-1]


# ---------------------------------------------------------------------------
# 网格外置
# ---------------------------------------------------------------------------
def externalize_meshes(
    root: ET.Element,
    out_dir: Path,
    *,
    mode: str = "obj",
    inline_max_vertices: int = 2000,
    subdir: str = "meshes",
    notes: list[str] | None = None,
) -> dict[str, Any]:
    """把 `<mesh>` 内联的 `vertex`/`face` 落成 `.obj` 文件并改为 `file=` 引用。

    Newton 的 MJCF 出口用 ``MjSpec.to_xml()``，而"由数组构造的网格"会被写成内联
    ``vertex``/``face`` —— 对高精度视觉网格（几十万顶点）会让 XML 膨胀到上百 MB。
    这里把它们外置成 OBJ，XML 只剩引用。

    ``mode="inline"`` 时不外置，但顶点数超过 ``inline_max_vertices`` 的仍会外置
    （避免"单文件"变成"上百 MB 单文件"）。
    """
    log: list[str] = notes if notes is not None else []
    written: list[str] = []
    inline_kept = 0
    for index, mesh in enumerate(root.iter("mesh")):
        vertex = mesh.get("vertex")
        face = mesh.get("face")
        if not vertex or not face:
            continue
        verts = vertex.split()
        faces = face.split()
        if mode == "inline" and len(verts) // 3 <= inline_max_vertices:
            inline_kept += 1
            continue
        name = mesh.get("name") or f"mesh{index}"
        filename = _mesh_filename(name)
        mesh_dir = out_dir / subdir
        mesh_dir.mkdir(parents=True, exist_ok=True)
        path = mesh_dir / filename
        _write_obj(path, verts, faces)
        mesh.set("file", f"{subdir}/{filename}")
        mesh.attrib.pop("vertex", None)
        mesh.attrib.pop("face", None)
        written.append(str(path))
    if written:
        log.append(
            f"  网格外置：{len(written)} 个网格写成 {subdir}/*.obj"
            + (f"（另有 {inline_kept} 个保留内联）" if inline_kept else "")
        )
    elif inline_kept:
        log.append(f"  网格：{inline_kept} 个网格保留内联（mode={mode}）")
    return {"written": written, "inline_kept": inline_kept, "count": len(written)}


def _mesh_filename(name: str) -> str:
    safe = re_sub(r"[^0-9A-Za-z._-]+", "_", name.strip("/"))
    safe = safe.strip("_") or "mesh"
    return f"{safe}.obj"[:180]


def _write_obj(path: Path, verts: list[str], faces: list[str]) -> None:
    lines: list[str] = []
    for i in range(0, len(verts) - 2, 3):
        lines.append(f"v {verts[i]} {verts[i + 1]} {verts[i + 2]}")
    for i in range(0, len(faces) - 2, 3):
        # MJCF 的 face 索引是 0 基；OBJ 是 1 基
        lines.append(f"f {int(faces[i]) + 1} {int(faces[i + 1]) + 1} {int(faces[i + 2]) + 1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def re_sub(pattern: str, repl: str, text: str) -> str:
    return re.sub(pattern, repl, text)
