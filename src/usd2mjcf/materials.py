"""USD 材质/颜色 → MJCF `<material>` 的解析。

Newton 的 MJCF 出口只带物理信息，不带视觉材质；而"能否正确渲染"很大程度取决于颜色。
这里用**多套回退**把颜色取出来（Omniverse 导出的资产常见 OmniPBR 风格输入）：

1. ``UsdPreviewSurface.diffuseColor``
2. OmniPBR 风格 ``base_color_factor`` / ``base_alpha``
3. ``primvars:displayColor`` / ``displayOpacity``
4. 若输入接了 ``UsdUVTexture``，记录贴图文件并复制到产物目录

材质绑定在 ``<mesh>`` 的 ``GeomSubset`` 上很常见，所以解析时允许沿路径向上回退。
"""

from __future__ import annotations

import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

# 归一化到 MJCF 的 rgba
_COLOR_INPUTS = ("diffuseColor", "base_color_factor", "base_color", "diffuse_color")
_ALPHA_INPUTS = ("opacity", "base_alpha", "opacity_factor")
_TEXTURE_INPUTS = ("diffuseColor", "base_color_texture", "diffuse_texture", "baseColorTexture")
_METALLIC_INPUTS = ("metallic", "metallic_factor")
_ROUGHNESS_INPUTS = ("roughness", "roughness_factor")
_EMISSIVE_INPUTS = ("emissiveColor", "emissive_color", "emissive_factor")

_INDEX_SUFFIX = re.compile(r"_\d+$")


@dataclass
class MaterialInfo:
    name: str
    rgba: tuple[float, float, float, float]
    texture: str | None = None       # 相对产物目录的贴图路径
    metallic: float | None = None
    roughness: float | None = None

    def key(self) -> tuple:
        return (self.rgba, self.texture, self.metallic, self.roughness)


@dataclass
class MaterialReport:
    prim_materials: dict[str, str] = field(default_factory=dict)  # USD prim 路径 -> 材质名
    materials: dict[str, MaterialInfo] = field(default_factory=dict)
    textures_copied: list[str] = field(default_factory=list)
    prims_scanned: int = 0
    prims_without_color: int = 0
    prims_errored: int = 0

    def resolve(self, geom_name: str) -> str | None:
        """把 MJCF geom 名（Newton 用 USD prim 路径命名）映到材质名。

        Newton 对重名会加 `_N` 后缀（如 `.../mesh/red_0`），也可能只保留了祖先路径，
        所以依次尝试：精确路径 → 去掉 `_N` → 逐级祖先。
        """
        candidates: list[str] = [geom_name]
        stripped = _INDEX_SUFFIX.sub("", geom_name)
        if stripped != geom_name:
            candidates.append(stripped)
        parts = geom_name.strip("/").split("/")
        for cut in range(len(parts) - 1, 0, -1):
            candidates.append("/" + "/".join(parts[:cut]))
        for candidate in candidates:
            if candidate in self.prim_materials:
                return self.prim_materials[candidate]
        return None


def _is_colorable(prim: Any, usd_module: tuple) -> bool:
    _, _, UsdGeom, _, _ = usd_module
    try:
        if prim.GetTypeName() == "GeomSubset":
            return True
        return bool(UsdGeom.Gprim(prim))
    except Exception:
        return False


def _to_rgb(value: Any) -> tuple[float, float, float] | None:
    if value is None:
        return None
    try:
        seq = list(value)
    except TypeError:
        scalar = float(value)
        return (scalar, scalar, scalar)
    if len(seq) == 3:
        return tuple(float(c) for c in seq)
    if len(seq) == 4:
        return tuple(float(c) for c in seq[:3])
    return None


def _first_input(shader: Any, names: Iterable[str], *, want: str = "value") -> tuple[str, Any] | None:
    """在若干候选输入名里找第一个*可用*的。

    OmniPBR / UsdPreviewSurface 的 shader 上往往同时存在多个同义输入（例如
    ``diffuseColor`` 存在但为空、真正取值在 ``base_color_factor``），所以不能只判
    “输入是否存在”，必须判“有没有值 / 有没有连接”。
    """
    for name in names:
        try:
            inp = shader.GetInput(name)
        except Exception:
            continue
        if inp is None:
            continue
        if want == "connection":
            if _safe_connections(inp):
                return name, inp
        elif _input_value(inp) is not None:
            return name, inp
    return None


def _safe_connections(input_obj: Any) -> list:
    """读输入的连接源。

    OpenUSD 的 Shade API 在**实例代理**（instanceable 子树）上会抛
    ``UsdExpiredPrimAccessError``（"null prim"）；这里退让而不是让整次转换失败 ——
    颜色信息丢了比转换失败轻得多，而且会在报告里统计出来。
    """
    try:
        attr = input_obj.GetAttr()
        if not attr:
            return []
        return list(attr.GetConnections())
    except Exception:
        return []


def _input_value(input_obj: Any) -> Any:
    try:
        return input_obj.Get()
    except Exception:
        return None


def _resolve_texture(usd_module: tuple, source_path: Any) -> str | None:
    """从 UsdUVTexture 节点里取出图片文件路径。"""
    _, _, _, _, UsdShade = usd_module
    try:
        shader = UsdShade.Shader(source_path)
        if not shader:
            return None
        file_input = shader.GetInput("file")
        if file_input is None:
            return None
        asset = _input_value(file_input)
    except Exception:
        return None
    if asset is None:
        return None
    return getattr(asset, "path", None) or str(asset) or None


def _material_from_prim(prim: Any, usd_module: tuple) -> MaterialInfo | None:
    """从一个 Material prim 上抽取颜色/贴图。"""
    _, Usd, UsdGeom, _, UsdShade = usd_module
    material = UsdShade.Material(prim)
    if not material:
        return None

    rgb: tuple[float, float, float] | None = None
    alpha = 1.0
    texture: str | None = None
    metallic = None
    roughness = None

    try:
        children = list(prim.GetChildren())
    except Exception:
        return None
    shaders = []
    for child in children:
        try:
            shader = UsdShade.Shader(child)
        except Exception:
            continue
        if shader:
            shaders.append(shader)
    for shader in shaders:
        try:
            tex_found = _first_input(shader, _TEXTURE_INPUTS, want="connection")
            col_found = _first_input(shader, _COLOR_INPUTS, want="value")
        except Exception:
            tex_found = col_found = None
        if tex_found is not None and texture is None:
            sources = _safe_connections(tex_found[1])
            if sources:
                texture = _resolve_texture(usd_module, sources[0])
        if col_found is not None and rgb is None:
            rgb = _to_rgb(_input_value(col_found[1]))
        found = _first_input(shader, _ALPHA_INPUTS)
        if found is not None:
            alpha_value = _input_value(found[1])
            if alpha_value is not None:
                try:
                    alpha = float(alpha_value)
                except (TypeError, ValueError):
                    pass
        for inputs, target in ((_METALLIC_INPUTS, "metallic"), (_ROUGHNESS_INPUTS, "roughness")):
            found = _first_input(shader, inputs)
            if found is None:
                continue
            value = _input_value(found[1])
            if value is None:
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            if target == "metallic":
                metallic = numeric
            else:
                roughness = numeric

    if rgb is None:
        # 贴图 + 无常量 tint 时给白色，避免整机变黑
        rgb = (1.0, 1.0, 1.0) if texture else None
    if rgb is None:
        return None
    return MaterialInfo(name="", rgba=(*rgb, alpha), texture=texture, metallic=metallic, roughness=roughness)


def _display_color(prim: Any, usd_module: tuple) -> tuple[float, float, float, float] | None:
    _, _, UsdGeom, _, _ = usd_module
    try:
        gprim = UsdGeom.Gprim(prim)
        if not gprim:
            return None
        colors = gprim.GetDisplayColorAttr().Get()
        if not colors:
            return None
        rgb = _to_rgb(colors[0])
        if rgb is None:
            return None
        opacities = gprim.GetDisplayOpacityAttr().Get()
        alpha = float(opacities[0]) if opacities else 1.0
    except Exception:
        return None
    return (*rgb, alpha)


def _find_prim_for_geom(stage: Any, name: str) -> Any:
    """Newton 用 USD prim 路径给 geom 命名；同名会有 `_N` 后缀。"""
    candidates = [name]
    stripped = _INDEX_SUFFIX.sub("", name)
    if stripped != name:
        candidates.append(stripped)
    for candidate in candidates:
        prim = stage.GetPrimAtPath(candidate)
        if prim and prim.IsValid():
            return prim
    # 向上回退（逐级去掉最后一段）
    parts = name.strip("/").split("/")
    for cut in range(len(parts) - 1, 0, -1):
        prim = stage.GetPrimAtPath("/" + "/".join(parts[:cut]))
        if prim and prim.IsValid():
            return prim
    return None


def collect_for_stage(
    stage: Any,
    *,
    usd_module: tuple,
    texture_dir: Path | None = None,
    texture_rel: str = "textures",
) -> MaterialReport:
    """扫描整个 stage，为每个可着色 prim 解析颜色。

    **必须在 Newton 解析之前调用**：Newton 的 USD 导入会改写/下载 layer，
    之后旧 prim 句柄会变成过期对象（UsdExpiredPrimAccessError）。
    """
    _, Usd, _, _, UsdShade = usd_module
    from . import usd_scene

    report = MaterialReport()
    cache: dict[str, MaterialInfo | None] = {}
    texture_names: dict[str, str] = {}
    key_to_name: dict[tuple, str] = {}

    for prim in usd_scene.iter_prims(stage):
        try:
            colorable = _is_colorable(prim, usd_module)
            path = prim.GetPath().pathString
        except Exception:
            report.prims_errored += 1
            continue
        if not colorable:
            continue
        report.prims_scanned += 1

        info: MaterialInfo | None = None
        node = prim
        while node and node.IsValid() and node.GetPath().pathString != "/":
            try:
                bound, _ = UsdShade.MaterialBindingAPI(node).ComputeBoundMaterial()
            except Exception:
                report.prims_errored += 1
                bound = None
            if bound:
                key = bound.GetPath().pathString
                if key not in cache:
                    try:
                        cache[key] = _material_from_prim(bound.GetPrim(), usd_module)
                    except Exception:
                        cache[key] = None
                info = cache[key]
                if info is not None:
                    break
            rgba = _display_color(node, usd_module)
            if rgba:
                info = MaterialInfo(name="", rgba=rgba)
                break
            node = node.GetParent()

        if info is None:
            report.prims_without_color += 1
            continue

        if info.texture and texture_dir is not None:
            source = Path(info.texture)
            if source.is_file():
                if info.texture not in texture_names:
                    texture_dir.mkdir(parents=True, exist_ok=True)
                    dest = texture_dir / source.name
                    try:
                        shutil.copyfile(source, dest)
                    except OSError:  # pragma: no cover
                        pass
                    else:
                        texture_names[info.texture] = f"{texture_rel}/{source.name}"
                        report.textures_copied.append(str(dest))
                info.texture = texture_names.get(info.texture, None)
            else:
                info.texture = None

        key = info.key()
        material_name = key_to_name.get(key)
        if material_name is None:
            material_name = f"mat{len(report.materials)}"
            key_to_name[key] = material_name
            report.materials[material_name] = MaterialInfo(
                name=material_name,
                rgba=info.rgba,
                texture=info.texture,
                metallic=info.metallic,
                roughness=info.roughness,
            )
        report.prim_materials[path] = material_name
    return report
