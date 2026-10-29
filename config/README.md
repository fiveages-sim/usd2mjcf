# 示例配置

这个目录保存可以直接使用的配置文件。

| 文件 | 内容 |
|---|---|
| `config.example.json` | 全部配置项和默认值，由脚本生成，可作为键表查阅 |
| `fiveages_w2/config.json` | 轮式双臂人形机器人的完整配置 |
| `fiveages_s2/config.json` | 固定立柱双臂人形机器人的完整配置 |

用法：

```bash
uv run usd2mjcf <输入.usda> <输出.xml> config/fiveages_w2/config.json
```

两份机型配置的重点：

- `fiveages_w2`：变体切换、正则改名、额外碰撞排除、驱动轮参数改写；
- `fiveages_s2`：固定基座、整机自碰撞关闭时的碰撞掩码修复、提高碰撞网格凸包精度。

只写想覆盖的项即可，没有写出的键使用默认值。

- 每个配置项的含义、默认值和可选值，见[配置参考](../doc/configuration.md)。
- 转换命令、产物和验证方式，见[使用指南](../doc/usage.md)。
- 产物中名字、执行器、几何和碰撞体的来源，见[转换规则](../doc/conversion.md)。

