# usd2mjcf

把 OpenUSD 物理资产转换成 MuJoCo MJCF 的命令行工具。

转换结果是一个自包含的模型，普通的 MuJoCo、`mujoco-warp` 或 mjlab 可以直接加载，不需要 USD 运行时。USD 里已经写好的控制器参数、关节参数和场景参数会尽量原样保留。

## 安装

需要 Python 3.12，依赖通过 `uv` 管理：

```bash
uv sync
```

默认使用 CUDA 后端转换，需要较新的 NVIDIA 驱动。没有 GPU 时，把配置里的 `backend` 改成 `"cpu"`。

## 快速开始

```bash
# 最简用法：输入 USD，输出 MJCF
uv run usd2mjcf ~/project/assets/robot.usda ~/project/out/robot.xml

# 使用配置文件
uv run usd2mjcf ~/project/assets/robot.usda ~/project/out/robot.xml ~/project/config/robot.json

# 输入也可以是目录或网址
uv run usd2mjcf ~/project/assets/robot_dir/ ~/project/out/robot.xml
uv run usd2mjcf https://example.com/robot.usda ~/project/out/robot.xml
```

省略输出路径时，会在当前目录生成与输入同名的 `.xml`。省略配置路径时，全部使用默认值。

## 产物

```
~/project/out/
├── robot.xml              # 模型文件
├── robot.report.json      # 转换报告
├── robot_view0.png ...    # 验证渲染
├── meshes/*.obj           # 外置网格
└── textures/              # 材质贴图（如有）
```

报告记录了本次转换的全部判断：变体选择、场景参数来源、碰撞体修复、材质和网格统计、命名结果，以及实际生效的配置。

## 自动验证

转换结束后会自动编译模型、检查积分器参数、空控制步进并离屏渲染，验证不通过时命令返回非零退出码。

不需要验证时，在配置里关闭：

```json
{
  "verify": {
    "enabled": false
  }
}
```

## 配置

命令行只负责输入、输出和配置文件路径，其余设置都写在 JSON 配置里。仓库的 `config/` 目录提供了完整示例：

- `config/config.example.json`：全部键和默认值；
- `config/fiveages_w2/config.json`、`config/fiveages_s2/config.json`、`config/unitree_g1d/config.json`：实际可用的完整配置。

常用项包括变体选择、碰撞体处理、凸包精度、命名规则、执行器控制范围、标记点和通用补丁。完整说明见[配置参考](doc/configuration.md)。

## 文档

- [文档索引](doc/index.md)
- [使用指南](doc/usage.md)
- [配置参考](doc/configuration.md)
- [转换规则](doc/conversion.md)
- [已知限制与排错](doc/limitations.md)
- [开发指南](doc/development.md)
