"""`usd2mjcf` 命令行入口 —— **只有三个参数**。

    usd2mjcf <输入.usda> [输出.xml] [配置.json]

* 输出省略 → `./<输入名>.xml`；
* 配置省略 → 全部用默认值（先跑一次，看 `<输出>.report.json` 里的 variants 段，
  再把要覆盖的键写进配置）；
* 其余旋钮（variant 选择、碰撞体模式、网格外置、材质、补丁、验证…）**全在配置文件里**，
  见 `config/config.example.json` 与 `config.py`。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Sequence

from . import __version__, config as config_mod, usd_scene


def _looks_like_config(text: str) -> bool:
    return text.lower().endswith(".json")


def _build_parser():
    import argparse

    parser = argparse.ArgumentParser(
        prog="usd2mjcf",
        description="把 OpenUSD 物理资产（Isaac Sim / MjcPhysics / Newton schema）转换成 MuJoCo MJCF。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "参数只有三个（后两个可省略）：\n"
            "  usd2mjcf robot.usda out/robot.xml out/config.json\n"
            "  usd2mjcf robot.usda out/robot.xml          # 用默认配置\n"
            "  usd2mjcf robot.usda out/robot.xml cfg.json\n"
            "  usd2mjcf robot.usda cfg.json               # 第二个参数是 .json 时按配置解析\n"
            "\n"
            "其余所有旋钮都在配置文件里：variant（换底盘/机械臂/夹爪）、碰撞体模式、\n"
            "网格外置、材质、timestep、补丁、验证开关… 见 config/config.example.json。\n"
            "每次转换都会写出 <输出>.report.json，里面有全部 variant 候选值、场景参数\n"
            "来源、碰撞体修复清单、材质/网格/执行器统计。"
        ),
    )
    parser.add_argument("input", nargs="?", help="输入 USD 文件或目录（必填）")
    parser.add_argument("output_pos", nargs="?", help="输出 MJCF 路径或目录（可省略）")
    parser.add_argument("config_pos", nargs="?", help="配置文件 JSON（可省略）")
    parser.add_argument("-o", "--output", dest="output_opt", help="同第二个位置参数")
    parser.add_argument("-c", "--config", dest="config_opt", help="同第三个位置参数")
    parser.add_argument("--version", action="version", version=f"usd2mjcf {__version__}")
    return parser


def _split_positionals(args) -> tuple[str | None, str | None, str | None]:
    """兼容 `usd2mjcf in.usda cfg.json`（第二个位置参数是配置）。"""
    output = args.output_opt or args.output_pos
    config = args.config_opt or args.config_pos
    if config is None and output is not None and _looks_like_config(output):
        config, output = output, None
    return args.input, output, config


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    input_arg, output_arg, config_arg = _split_positionals(args)

    if not input_arg:
        _build_parser().print_help()
        return 2

    # 目录输入：自动挑入口
    source = Path(input_arg).expanduser()
    if source.is_dir():
        picked, ranked = usd_scene.resolve_input_path(source)
        print(f"🔎 目录输入，自动挑选入口：{picked}")
        for score, path, reason in ranked[:5]:
            print(f"    [{score:+.1f}] {path}   ({reason})")
        input_arg = str(picked)

    cfg = config_mod.load_or_default(config_arg)
    config_mod.apply_cli_overrides(cfg, usd=input_arg, output=output_arg)
    for problem in getattr(cfg, "_problems", []) or []:
        print(f"⚠️  配置：{problem}")

    if cfg.verify.gl:
        import os

        os.environ["MUJOCO_GL"] = cfg.verify.gl

    from . import convert as convert_mod

    result = convert_mod.convert(cfg)
    _print_report(result)

    exit_code = 0
    if cfg.verify.enabled:
        from . import verify as verify_mod

        render_dir = Path(cfg.verify.render_dir) if cfg.verify.render_dir else result.output.parent
        report = verify_mod.verify(
            result.output,
            render_dir=render_dir,
            views=config_mod.views_tuples(cfg.verify),
            smoke_steps=cfg.verify.smoke_steps,
            do_render=cfg.verify.render,
        )
        exit_code = _print_verify(report, result.output)
    return exit_code


def _print_report(result: Any) -> None:
    print()
    print("═" * 78)
    print("转换完成")
    print("═" * 78)
    print(f"  输入    : {result.input}")
    print(f"  输出    : {result.output}")
    if result.report_path:
        print(f"  报告    : {result.report_path}")
    print(f"  耗时    : {result.seconds:.1f}s")
    print()
    print("─ Newton 解析 ─")
    for key, value in result.model_stats.items():
        print(f"  {key:<14}{value}")
    print("─ MJCF 产物 ─")
    for key, value in result.mjcf_stats.items():
        print(f"  {key:<14}{value}")
    if result.material_stats.get("materials"):
        print("─ 材质 / 网格 / 执行器 ─")
        print(f"  materials     {result.material_stats}")
        print(f"  meshes        {result.mesh_stats}")
        print(f"  actuators     {result.actuator_stats}")
    print("─ 改动明细 ─")
    for line in result.changes:
        print(f"  {line}" if line.startswith("  ") else f"  • {line}")
    if result.warnings:
        print("─ ⚠️ 警告 ─")
        for line in result.warnings:
            print(f"  ! {line}")


def _print_verify(report: Any, output: Path) -> int:
    print()
    print("─ 验证（pip MuJoCo，无 USD 支持）─")
    if report.inspect is not None:
        for key, value in report.inspect.table().items():
            print(f"  {key:<20}{value}")
    if report.smoke is not None:
        print(
            f"  {'smokeStep':<20}{report.smoke.steps} 步，finite={report.smoke.finite}，"
            f"|qvel|max={report.smoke.max_qvel:.3g}，RMS={report.smoke.rms_qvel:.3g}"
        )
    for line in report.warnings:
        print(f"  ⚠️  {line}")
    for line in report.errors:
        print(f"  ❌  {line}")
    if report.images:
        print("  渲染图片：")
        for path in report.images:
            print(f"    {path}")
    ok = report.inspect is not None and not report.errors
    print()
    print(f"{'✅ 验证通过' if ok else '❌ 验证失败'}：{output}")
    return 0 if ok else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
