"""产物验证：用**不带 USD 支持**的 pip MuJoCo 编译 + 前向 + 短程步进 + 离屏渲染。

这一层是"自包含"的证明：能编译说明产物不依赖 USD；能渲染说明几何/材质正确；
短程步进 + 显式积分器守卫说明它在 MuJoCo 侧是被数值条件认可的。
"""

from __future__ import annotations

import ctypes.util
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np

__all__ = ["load_mujoco", "pick_gl_backend", "inspect", "smoke_step", "render", "verify"]


# ---------------------------------------------------------------------------
# GL 后端探测（MUJOCO_GL 必须在 import mujoco 之前设置）
# ---------------------------------------------------------------------------
def pick_gl_backend() -> str:
    """挑一个最可能可用的离屏 GL 后端。"""
    if os.environ.get("MUJOCO_GL"):
        return os.environ["MUJOCO_GL"]
    if ctypes.util.find_library("EGL"):
        return "egl"
    if ctypes.util.find_library("OSMesa"):
        return "osmesa"
    return "glfw"


def load_mujoco(gl: str | None = None) -> Any:
    """设置 MUJOCO_GL 后导入 mujoco（必须在本进程首次 import 之前调用）。"""
    os.environ.setdefault("MUJOCO_GL", gl or pick_gl_backend())
    import mujoco

    return mujoco


# ---------------------------------------------------------------------------
@dataclass
class InspectResult:
    model_path: Path
    njnt: int = 0
    nq: int = 0
    nv: int = 0
    nu: int = 0
    nbody: int = 0
    ngeom: int = 0
    nmesh: int = 0
    nsite: int = 0
    neq: int = 0
    timestep: float = 0.0
    integrator: str = ""
    solver: str = ""
    cone: str = ""
    iterations: int = 0
    total_mass: float = 0.0
    actuator_guard: float = 0.0
    as_dict: Any = None

    def table(self) -> dict[str, Any]:
        return {
            "njnt": self.njnt, "nq": self.nq, "nv": self.nv, "nu": self.nu,
            "nbody": self.nbody, "ngeom": self.ngeom, "nmesh": self.nmesh,
            "nsite": self.nsite, "neq": self.neq,
            "timestep": self.timestep, "integrator": self.integrator,
            "solver": self.solver, "cone": self.cone, "iterations": self.iterations,
            "totalMass": self.total_mass, "integratorGuardRatio": self.actuator_guard,
        }


def inspect(model_path: Path) -> InspectResult:
    """编译产物并抽取关键参数（不需要 USD，也不需要 GPU）。"""
    mujoco = load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def enum_name(enum: Any, value: int) -> str:
        try:
            return enum(value).name
        except Exception:  # pragma: no cover
            return str(value)

    result = InspectResult(model_path=model_path)
    result.njnt, result.nq, result.nv = model.njnt, model.nq, model.nv
    result.nu, result.nbody, result.ngeom = model.nu, model.nbody, model.ngeom
    result.nmesh, result.nsite, result.neq = model.nmesh, model.nsite, model.neq
    result.timestep = float(model.opt.timestep)
    result.iterations = int(model.opt.iterations)
    result.integrator = enum_name(mujoco.mjtIntegrator, model.opt.integrator)
    result.solver = enum_name(mujoco.mjtSolver, model.opt.solver)
    result.cone = enum_name(mujoco.mjtCone, model.opt.cone)
    result.total_mass = float(sum(model.body_mass))
    # 高增益伺服 + 显式积分的发散判据：dt * kd / (M0 + armature) >= 2 必发散
    denom = np.asarray(model.dof_M0) + np.asarray(model.dof_armature)
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(denom > 0, model.opt.timestep * np.asarray(model.dof_damping) / denom, 0.0)
    result.actuator_guard = float(np.max(ratio)) if ratio.size else 0.0
    return result


# ---------------------------------------------------------------------------
@dataclass
class SmokeResult:
    steps: int
    finite: bool
    max_qvel: float
    rms_qvel: float
    warnings: list[str] = field(default_factory=list)


def smoke_step(model_path: Path, steps: int = 200, seed: int = 0) -> SmokeResult:
    """零控制短程步进：只用来暴露 NaN / 立即发散，不做稳定性验收。"""
    mujoco = load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if model.nu:
        # 位置伺服把 ctrl 置为中心点，避免"ctrl=0 把关节拉到零位"造成的假发散
        data.ctrl[:] = 0.0
    peak = 0.0
    windows_warnings: list[str] = []
    for _ in range(steps):
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qvel)):
            return SmokeResult(steps, False, float("inf"), float("inf"), ["步进过程中出现 NaN/Inf"])
        peak = max(peak, float(np.max(np.abs(data.qvel))))
    rms = float(np.sqrt(np.mean(np.square(data.qvel)))) if data.qvel.size else 0.0
    if peak > 1e4:
        windows_warnings.append(f"qvel 峰值 {peak:.3e} rad/s，明显发散")
    return SmokeResult(steps, True, peak, rms, windows_warnings)


# ---------------------------------------------------------------------------
def render(
    model_path: Path,
    out_png: Path,
    *,
    width: int = 1280,
    height: int = 900,
    azimuth: float = 135.0,
    elevation: float = -20.0,
    distance_scale: float = 1.25,
    lookat: Sequence[float] | None = None,
    qpos: Any = None,
) -> Path:
    """离屏渲染一张图，返回 PNG 路径。"""
    mujoco = load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    if qpos is not None:
        data.qpos[:] = np.asarray(qpos, dtype=float)[: model.nq]
    mujoco.mj_forward(model, data)

    # 提高验证渲染的灯光：MuJoCo 默认 headlight 在金属度高的材质上会接近全黑
    try:
        model.vis.headlight.ambient[:] = (0.45, 0.45, 0.45)
        model.vis.headlight.diffuse[:] = (0.65, 0.65, 0.65)
        model.vis.headlight.specular[:] = (0.25, 0.25, 0.25)
    except Exception:  # pragma: no cover
        pass

    if lookat is None:
        positions = np.asarray(data.geom_xpos)
        radii = np.asarray(model.geom_rbound)
        if positions.size:
            lo, hi = positions.min(axis=0), positions.max(axis=0)
            center = 0.5 * (lo + hi)
            extent = float(np.max(hi - lo)) + 2.0 * float(np.max(radii))
        else:
            center, extent = np.zeros(3), 1.0
    else:
        center = np.asarray(lookat, dtype=float)
        extent = 1.0

    # 按垂直视场角算距离，让模型刚好填满画面
    fovy = float(getattr(model.vis.global_, "fovy", 45.0))
    fit_distance = 0.5 * max(extent, 1e-3) / max(np.tan(np.radians(fovy) / 2.0), 1e-6)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.lookat[:] = center
    camera.distance = fit_distance * distance_scale
    camera.azimuth = azimuth
    camera.elevation = elevation

    def _shoot(w: int, h: int) -> np.ndarray:
        # MuJoCo 离屏缓冲尺寸来自 model.vis.global_.off*，默认只有 640×480，
        # 直接请求更大尺寸会报 "framebuffer width" 错误。
        if w > model.vis.global_.offwidth:
            model.vis.global_.offwidth = int(w)
        if h > model.vis.global_.offheight:
            model.vis.global_.offheight = int(h)
        renderer = mujoco.Renderer(model, height=h, width=w)
        try:
            renderer.update_scene(data, camera=camera)
            return renderer.render()
        finally:
            renderer.close()

    try:
        pixels = _shoot(width, height)
    except ValueError as exc:
        # 硬件/EGL 上限更小时，按错误信息里的可用宽度退让一次
        import re

        match = re.search(r"framebuffer width (\d+)", str(exc))
        if not match:
            raise
        fallback_w = int(match.group(1))
        fallback_h = max(240, int(fallback_w * height / max(width, 1)))
        pixels = _shoot(fallback_w, fallback_h)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image

        Image.fromarray(pixels).save(out_png)
    except ImportError:  # pragma: no cover
        import numpy as _np

        _np.save(out_png.with_suffix(".npy"), pixels)
        return out_png.with_suffix(".npy")
    return out_png


# ---------------------------------------------------------------------------
@dataclass
class VerifyReport:
    inspect: InspectResult | None = None
    smoke: SmokeResult | None = None
    images: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


def verify(
    model_path: Path,
    *,
    render_dir: Path | None = None,
    views: Sequence[tuple[float, float]] = ((135.0, -20.0), (45.0, -10.0), (225.0, 0.0)),
    smoke_steps: int = 200,
    do_render: bool = True,
) -> VerifyReport:
    report = VerifyReport()
    try:
        report.inspect = inspect(model_path)
    except Exception as exc:  # pragma: no cover
        report.errors.append(f"MuJoCo 编译失败：{type(exc).__name__}: {exc}")
        return report

    info = report.inspect
    if info.integrator.lower().startswith("euler") and info.actuator_guard >= 2.0:
        report.warnings.append(
            f"积分器守卫：integrator={info.integrator}，dt·kd/(M0+armature) 最大比值 "
            f"{info.actuator_guard:.3g} ≥ 2 → 显式积分解必发散。请确认 USD 里 "
            "mjc:option:integrator 是否为 implicitfast。"
        )
    try:
        report.smoke = smoke_step(model_path, steps=smoke_steps)
        report.warnings.extend(report.smoke.warnings)
    except Exception as exc:  # pragma: no cover
        report.errors.append(f"步进失败：{type(exc).__name__}: {exc}")

    if do_render and render_dir is not None:
        for index, (azimuth, elevation) in enumerate(views):
            png = render_dir / f"{model_path.stem}_view{index}.png"
            try:
                report.images.append(str(render(model_path, png, azimuth=azimuth, elevation=elevation)))
            except Exception as exc:
                report.errors.append(
                    f"渲染视角 {index} 失败：{type(exc).__name__}: {exc} —— "
                    "可尝试 --gl egl/osmesa/glfw 或 --no-render"
                )
                break
    return report
