# 示例配置

这个目录保存可以直接使用的配置文件。

| 文件 | 内容 |
|---|---|
| `config.example.json` | 全部配置项和默认值，由脚本生成，可作为键表查阅 |
| `fiveages_w2/config.json` | 轮式双臂人形机器人的完整配置 |
| `fiveages_s2/config.json` | 固定立柱双臂人形机器人的完整配置 |
| `unitree_g1d/config.json` | Unitree G1D 轮式底盘加升降柱双臂人形的完整配置 |

用法：

```bash
uv run usd2mjcf <输入.usda> <输出.xml> config/fiveages_w2/config.json
```

三份机型配置的重点：

- `fiveages_w2`：变体切换、正则改名、额外碰撞排除、驱动轮参数改写；
- `fiveages_s2`：固定基座、整机自碰撞按父子过滤、提高碰撞网格凸包精度；
- `unitree_g1d`：使用资产的 physx 物理层、整机自碰撞按父子过滤、跳过零厚度碰撞网格、驱动轮参数改写。

`unitree_g1d` 的 `Physics` 变体必须选 `physx`。该资产的 `mujoco` 层是空的，选它会
得到没有关节和执行器的空壳，而且转换过程不会报错。

只写想覆盖的项即可，没有写出的键使用默认值。

- 每个配置项的含义、默认值和可选值，见[配置参考](../doc/configuration.md)。
- 转换命令、产物和验证方式，见[使用指南](../doc/usage.md)。
- 产物中名字、执行器、几何和碰撞体的来源，见[转换规则](../doc/conversion.md)。

