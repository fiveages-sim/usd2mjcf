"""usd2mjcf —— 把 OpenUSD 物理资产转换成 MuJoCo MJCF。

设计目标：
  * 通用：任何带 UsdPhysics / MjcPhysics / Newton schema 的 USD 都能转，不做机型特判。
  * 保真：USD 里已授权的物理参数（执行器增益、力限、armature、阻尼、关节限位、
    场景/求解器设置）尽量原样落到 MJCF，避免在 MuJoCo 侧重新调参。
  * 可校验：转换后用不带 USD 的 pip MuJoCo 重新编译 + 离屏渲染，证明产物自包含。
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
