"""示例用的是**真实资产**。

| 例子 | 资产 | 演示什么 |
|---|---|---|
| [`fiveages_w2`](fiveages_w2/) | FiveAges W2 轮式双臂人形（带完整 `mjc:` 授权） | 换底盘/机械臂/末端执行器；正则改名；额外碰撞排除；驱动轮唯一一处“有意偏离” |
| [`fiveages_s2`](fiveages_s2/) | FiveAges S2 固定立柱双臂人形（带完整 `mjc:` 授权） | 固定基座（无 freejoint）；**USD 关闭自碰撞导致掩码全 0 的修复**；无需手写 excludes |

> 示例刻意只用**带 `mjc:` 授权**的资产。纯 PhysX 授权的资产（例如 Isaac 官方的
> Franka Panda）虽然也能转（靠 `DriveAPI` 换算增益），但没有 `mjc:` 这一层，
> 执行器力限/armature/关节级参数都只能靠猜测或手填，不适合当作“保真转换”的范例。

跑法（CLI 只有三个参数）：

```bash
uv run usd2mjcf ~/FaSim-Isaac/robots/humanoid/FiveAges/Gen2/W2/FiveAges_W2.usda \
  out/FA-W2/robot.xml examples/fiveages_w2/config.json

uv run usd2mjcf ~/FaSim-Isaac/robots/humanoid/FiveAges/Gen2/S2/FiveAges_S2.usda \
  out/FA-S2/robot.xml examples/fiveages_s2/config.json
```

每次转换都会写 `<输出>.report.json`，里面能看到：

* `variants[]` —— 每个 variant set 的**全部候选值**、本次选中值、以及是作者授权还是自动/配置指定；
* `scene` —— 场景参数的有效值与**来源**（`usd` / `config`）；
* `collision` —— 被 USD 禁用的碰撞体路径、被重开了哪些、自带材质而被 Newton 丢弃的碰撞体；
* `contactExcludes` —— 新增/跳过/未找到的额外碰撞排除；
* `sites` —— TCP/EEF 等 `<site>` 的新增/跳过/目标 body 不存在的记录；
* `naming` —— 名字还原结果、因撞名而加长的名称、Newton 合成名（改用 `patch.rename`）；
* `ctrlrange` —— `ctrlrange` 单位修正与补齐；
* `config` —— 本次实际生效的完整配置（可直接改完再跑）。

## 换资产

1. 跑一次不带配置的转换，看 `report.json` 的 `variants[]`；
2. 把要覆盖的键抄进自己的 `config.json`（结构见 [`config.example.json`](../config.example.json)）；
3. 若产物里有“物理上必然重叠、但 USD 没写成形状级过滤”的部位，补 `mjcf.contact.excludes`；
4. 需要 TCP/EEF 等标记点时，用 `mjcf.sites` 把 `<site>` 挂到指定 body 上（见根 README §6.4）；
5. 若命名不合口味，用 `patch.rename` 的正则（见根 README §4.5）。

## 远程资产

`usd2mjcf` 的输入支持 `https://` URL，会自动把依赖闭包（含 `payloads/`）镜像到
`usd.download_dir`（默认 `./.usd_cache`），下次直接复用缓存。

> ⚠️ 只顺着已打开的 layer 找引用会**漏掉整个 `payloads/` 目录**（payload 是按需加载的
> 合成弧）。本工具用 `UsdUtils.ComputeAllDependencies` 递归收集，并跳过 `.mdl` 这类
> Omniverse 材质库文件。
