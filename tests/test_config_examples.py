"""配置文件示例的“防漂移”测试。

配置文件是本工具唯一的旋钮面，而 README/示例里的键名最容易随时间漂移成
“拼错的键 + 静默不生效”。这里把仓库里所有示例配置都加载一遍，确保：

* 能被解析（合法 JSON + 合法取值）；
* **没有未知键**（拼错会变成 problems 而不是被忽略）；
* `config.example.json` 的键集合与 dataclass 完全一致（新增/删除键时会被逼着更新）。
"""

from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path

import pytest

from usd2mjcf import config as config_mod

REPO = Path(__file__).resolve().parents[1]

EXAMPLE_CONFIGS = [
    REPO / "config.example.json",
    REPO / "examples" / "fiveages_w2" / "config.json",
    REPO / "examples" / "fiveages_s2" / "config.json",
]

#: examples 里 config.json 的注释键（加载器会忽略 _ 开头的键）
ALLOWED_COMMENT_KEYS = {"_comment"}


@pytest.mark.parametrize("path", EXAMPLE_CONFIGS, ids=lambda p: p.parent.name)
def test_example_config_has_no_unknown_keys(path: Path):
    assert path.is_file(), f"缺少示例配置：{path}"
    cfg, problems = config_mod.load(path)
    assert problems == [], f"{path.name} 配置有问题：{problems}"
    assert isinstance(cfg, config_mod.Config)


def test_generated_example_matches_schema():
    """config.example.json 由 scripts/write_example_config.py 生成，键集必须与 schema 一致。"""
    path = REPO / "config.example.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    expected = {f.name for f in fields(config_mod.Config)} | ALLOWED_COMMENT_KEYS
    assert set(raw) == expected, (
        "config.example.json 与配置 schema 不一致，请重新运行 "
        "`uv run python scripts/write_example_config.py`"
    )
    # 子配置键也在 schema 里（逐段核对）
    assert set(raw["usd"]) == {f.name for f in fields(config_mod.UsdConfig)}
    assert set(raw["mjcf"]) == {f.name for f in fields(config_mod.MjcfConfig)}
    assert set(raw["mjcf"]["scene"]) == {f.name for f in fields(config_mod.SceneConfig)}
    assert set(raw["mjcf"]["materials"]) == {f.name for f in fields(config_mod.MaterialConfig)}
    assert set(raw["verify"]) == {f.name for f in fields(config_mod.VerifyConfig)}


def test_save_round_trip(tmp_path: Path):
    """save() → load() 必须完全等价（示例文件就是这么来的）。"""
    cfg = config_mod.Config()
    cfg.usd.variants = {"Physics": "mujoco"}
    cfg.patch = {"option": {"integrator": "implicitfast"}}
    path = config_mod.save(cfg, tmp_path / "cfg.json", comment="round trip")
    loaded, problems = config_mod.load(path)
    assert problems == []
    assert loaded.usd.variants == {"Physics": "mujoco"}
    assert loaded.patch == {"option": {"integrator": "implicitfast"}}


def test_validation_catches_typos(tmp_path: Path):
    """拼错的键必须变成 problems，而不是被静默忽略。"""
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"mjcf": {"meshMode": "obj"}}), encoding="utf-8")
    cfg, problems = config_mod.load(path)
    assert any("meshMode" in p for p in problems)


def test_validation_catches_bad_values(tmp_path: Path):
    path = tmp_path / "bad2.json"
    path.write_text(
        json.dumps({"usd": {"collision_mode": "nope"}, "mjcf": {"mesh_mode": "zip"}}),
        encoding="utf-8",
    )
    _cfg, problems = config_mod.load(path)
    assert any("collision_mode" in p for p in problems)
    assert any("mesh_mode" in p for p in problems)


def test_strict_mode_raises(tmp_path: Path):
    path = tmp_path / "bad3.json"
    path.write_text(json.dumps({"unknownKey": 1}), encoding="utf-8")
    with pytest.raises(config_mod.ConfigError):
        config_mod.load(path, strict=True)
