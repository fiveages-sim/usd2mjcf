#!/usr/bin/env python
"""生成 `config.example.json`（全量键 + 默认值）。

配置文件是**唯一旋钮面**，但键会随代码演进；手写示例迟早漂移成"拼错的键 + 静默不生效"。
所以示例文件由这个脚本从 dataclass 生成，`tests/test_config_examples.py` 会校验
仓库里的这几份配置文件都能被加载且没有未知键。

    uv run python scripts/write_example_config.py
"""

from __future__ import annotations

import json
from pathlib import Path

from usd2mjcf import config as config_mod

REPO = Path(__file__).resolve().parents[1]

HEADER = """\
usd2mjcf 配置文件（唯一旋钮面）。
CLI 只有三个参数：  usd2mjcf <输入.usda> <输出.xml> [本文件]
未出现的键取默认值；**拼错的键会给出显式警告**（不会静默忽略）。
建议流程：先不带配置跑一次 → 看 <输出>.report.json 里的 variants / scene /
collision 段 → 只把要覆盖的键写进配置。"""


def main() -> None:
    cfg = config_mod.Config()
    path = config_mod.save(cfg, REPO / "config.example.json", comment=HEADER)
    payload = json.loads(path.read_text(encoding="utf-8"))
    print(f"已写出 {path.relative_to(REPO)}（{len(payload)} 个顶层键）")


if __name__ == "__main__":
    main()
