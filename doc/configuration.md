# 配置参考

配置文件是调整转换行为的唯一入口。命令行只负责给出输入、输出和配置文件路径，其余设置全部写在 JSON 里。

## 配置文件怎么用

```bash
uv run usd2mjcf ~/project/assets/robot.usda ~/project/out/robot.xml ~/project/config/robot.json
```

也可以把输入和输出写进配置，命令行参数优先生效。

加载规则：

- 没有写出的键使用默认值；
- 写错键名不会被直接忽略，而是打印一条警告；
- 取值不合法同样会打印警告；
- 把 `strict_patch` 设为 `true` 后，补丁规则没有命中任何元素会直接报错；
- 以 `_` 开头的键会被忽略，可以用来写注释。

完整键表可以直接看仓库里的 `config/config.example.json`，它由脚本从代码中的配置定义生成，不会和代码脱节。

## 顶层键

| 键 | 默认值 | 说明 |
|---|---|---|
| `usd` | 见下文 | USD 侧的设置 |
| `mjcf` | 见下文 | 输出 MJCF 的设置 |
| `patch` | `{}` | 最后应用的通用补丁 |
| `strict_patch` | `false` | 补丁没有命中元素时是否直接报错 |
| `backend` | `"cuda"` | 转换使用的后端，可选 `cuda` 或 `cpu` |
| `report` | `null` | 报告路径，默认写在输出文件旁边 |
| `report_enabled` | `true` | 是否生成报告 |
| `verbose` | `false` | 是否打印 USD 与 Newton 的详细解析日志 |
| `verify` | 见下文 | 转换后的自动验证 |

`backend` 设为 `"cpu"` 时使用 MuJoCo 的 CPU 后端导出，适合没有 GPU 的机器；也可以用 `"mujoco_cpu"` 或 `"mujoco-c"`。

## usd 段

| 键 | 默认值 | 说明 |
|---|---|---|
| `path` | `null` | 输入 USD，通常由命令行给出 |
| `variants` | `{}` | 指定变体集合的取值，只需要写要覆盖的部分 |
| `solver_variants` | `["mujoco", "newton"]` | 资产没有指定默认变体时的自动挑选顺序 |
| `collision_mode` | `"enable"` | `enable` 表示重新打开被资产禁用的碰撞体，`respect` 表示完全按 USD |
| `max_hull_vertices` | `null` | 碰撞网格凸包的最大顶点数，`null` 表示沿用 Newton 默认值 64 |
| `fix_self_material_colliders` | `true` | 修复“碰撞体自带材质信息”导致整个碰撞体被跳过的问题 |
| `add_usd` | `{}` | 直接透传给 Newton 的 USD 导入参数 |
| `download_dir` | `null` | 远程资产的缓存目录，默认是 `./.usd_cache` |
| `schema_priority` | `["mjc", "newton", "physx"]` | 读取 USD 属性时的 schema 优先级 |

### variants

USD 里可以有多个可切换的部分，例如底盘型号、左右机械臂型号、末端执行器型号。写法是“集合名：取值”：

```json
{
  "usd": {
    "variants": {
      "Chassis": "LinkhouS2_V2",
      "EE": "AG2F90"
    }
  }
}
```

只写想覆盖的项，其余沿用资产作者的默认选择。

### collision_mode

有些资产用 `physics:collisionEnabled = false` 表达“这个部位不参与自碰撞”。对 PhysX 来说这是碰撞过滤，但对 MuJoCo 来说等于没有碰撞体。

- `enable`（默认）：重新打开这些碰撞体，并在报告里列出全部路径；
- `respect`：完全按 USD 的设置，可能得到没有碰撞的模型。

重新打开后如果出现部件互相挤开的情况，用 `mjcf.contact.excludes` 补上排除关系。

### max_hull_vertices

碰撞网格最终会被转成凸包。默认上限是 64 个顶点，顶点太少时凸包会不对称地切掉底面角部，导致支撑面变小。

```json
{
  "usd": {
    "max_hull_vertices": 512
  }
}
```

设置后，工具会在导入前覆盖 USD 里已经写好的顶点上限，同时也把该值作为导入参数的默认值。报告里的 `collision.maxHullVertices` 和 `collision.hullOverrides` 会记录实际改动。

### add_usd

需要直接控制 Newton 的 USD 导入行为时使用，例如：

```json
{
  "usd": {
    "add_usd": {
      "floating": false
    }
  }
}
```

这里写入的参数会原样传给 Newton。显式写在 `add_usd.mesh_maxhullvert` 里的值优先于 `max_hull_vertices`。

## mjcf 段

| 键 | 默认值 | 说明 |
|---|---|---|
| `path` | `null` | 输出 MJCF，通常由命令行给出 |
| `mesh_mode` | `"obj"` | `obj` 表示网格外置成文件，`inline` 表示写进 XML |
| `inline_max_vertices` | `2000` | `inline` 模式下超过该顶点数的网格仍然外置 |
| `actuator_names` | `"joint"` | `joint` 表示用所驱动的关节名命名执行器，`none` 表示不命名 |
| `geom_names` | `"kind"` | `kind` 表示用 `VISUAL_<body>_<序号>` / `COLLISION_<body>_<序号>` 命名几何；`usd` 表示保留 Newton 生成的路径名 |
| `name_from_usd` | `["joint", "body"]` | 把 Newton 生成的长名字还原成 USD 里的名字，作用对象可多选 |
| `fix_ctrlrange` | `true` | 修正式执行器控制范围的单位问题和缺失 |
| `ctrlrange_auto_degrees` | `true` | 判定为“度”时自动换算成弧度 |
| `ctrlrange_from_joint` | `true` | 控制范围缺失时用关节限位补 |
| `fix_zero_collision_masks` | `true` | 修复碰撞掩码被全部算成 0 的问题 |
| `visuals` | `true` | 是否导出只用于显示的几何 |
| `sites` | `[]` | 补充标记点 |
| `scene` | 见下文 | 场景参数 |
| `materials` | 见下文 | 材质与颜色 |
| `contact` | 见下文 | 额外碰撞排除 |

### mesh_mode

MuJoCo 的 MJCF 写出接口会把由数组构造的网格写成内联顶点和面片，视觉网格顶点很多时 XML 会变得非常大。

- `obj`（默认）：每个网格写成 `meshes/` 下的 `.obj` 文件，XML 只保留文件引用；
- `inline`：保留内联写法，但顶点数超过 `inline_max_vertices` 的网格仍然外置。

### name_from_usd

Newton 会用“下划线加 prim 路径”的方式命名实体，产物里的名字会很长。这个选项把名字还原成 USD 里的原始名字。

可选值：`joint`、`body`、`geom`、`site`、`mesh`、`tendon`、`actuator`。空列表表示完全不改名字。

详细命名规则见[转换规则](conversion.md)中的“命名”一节。

### geom_names

控制几何的名字。默认值 `kind` 会统一成短名：

- 视觉体：`VISUAL_<body>_<序号>`
- 碰撞体：`COLLISION_<body>_<序号>`

序号按同一个 body、同一类几何各自从 0 开始。网格本身的名字和 `mesh=` 引用不变。

统一命名在补丁之后执行，所以名字里的 body 段是补丁改名后的最终 body 名。反过来，补丁里按名字匹配 geom 的规则（`patch.geoms`、`patch.rename.geom`）看到的仍是统一命名之前的名字；需要自己控制 geom 名字时，把 `mjcf.geom_names` 设为 `"usd"`。

改成 `"usd"` 时保留 Newton 生成的路径名（例如 `/Robot/base/collisions/...`）。

如果 `name_from_usd` 里包含 `geom`，还原出的 geom 名字也会被统一命名覆盖；想保留 USD 里的名字，需要同时把 `mjcf.geom_names` 设为 `"usd"`。

### fix_ctrlrange 与相关选项

部分资产把控制范围写成了“度”，而 MJCF 使用弧度，直接照抄会让控制范围放大 57 倍。工具会判断控制范围与关节限位的跨度比，接近 180/π 时自动换算。

`ctrlrange_from_joint` 负责另一种情况：执行器声明了限幅，但控制范围为空或全零。此时用关节限位补上，并打开限幅开关。

不需要这些修正时，把 `fix_ctrlrange` 设为 `false`。

## mjcf.scene 段

| 键 | 默认值 | 说明 |
|---|---|---|
| `sync_from_usd` | `true` | 是否用 USD 场景里的参数覆盖这里的默认值 |
| `timestep` | `0.002` | 时间步长 |
| `iterations` | `null` | 求解器迭代次数 |
| `gravity` | `null` | 重力向量 |
| `flags` | `{}` | 直接写入 `<flag>` 的开关 |
| `option` | `{}` | 直接写入 `<option>` 的其他属性 |
| `compiler` | `{}` | 直接写入 `<compiler>` 的属性 |

这里的值是默认值：USD 场景里有设置时以 USD 为准，没有才使用配置。`sync_from_usd` 设为 `false` 可以强制使用配置。

时间步长有单独的取值顺序：USD 的 `mjc:option:timestep`、`newton:timeStepsPerSecond`、配置里的 `timestep`。报告里会标注实际来源。

## mjcf.materials 段

| 键 | 默认值 | 说明 |
|---|---|---|
| `enabled` | `true` | 是否从 USD 传递颜色和材质 |
| `pbr` | `false` | 是否同时写入金属度和粗糙度 |
| `hide_colliders` | `true` | 是否把碰撞体放到默认不显示的渲染组 |

`pbr` 默认关闭，是因为很多资产使用高金属度材质，在 MuJoCo 的默认灯光下会显示成黑色。

## mjcf.contact 段

```json
{
  "mjcf": {
    "contact": {
      "excludes": [
        ["base_footprint", "body_link*"],
        ["base_footprint", "*_wheel"]
      ]
    }
  }
}
```

每一项是一对 body 名，两侧都支持通配符。已经存在的排除会跳过，名字找不到会给出警告。这里写的是产物里的 body 名，不是 USD 路径。

## mjcf.sites 段

用来补充 TCP、末端执行器中心、相机位置等标记点。

```json
{
  "mjcf": {
    "sites": [
      {
        "name": "LEFT_TCP_SITE",
        "body": "left_link7",
        "pos": [0, -0.095, 0],
        "quat": [0.5, 0.5, -0.5, 0.5],
        "size": 0.005,
        "rgba": [0.2, 0.9, 0.2, 0.5]
      },
      {
        "name": "LEFT_EEF_SITE",
        "body": "left_gripper_gripper_base",
        "pos": [0, 0, 0.19],
        "optional": true
      }
    ]
  }
}
```

规则：

- `name` 和 `body` 必填；
- `body` 支持通配符，命中多个 body 时只使用第一个，并给出警告；
- `optional` 为 `true` 时，body 不存在只记日志，不告警；
- 其余键会原样写入 `<site>`，可用键包括 `type`、`group`、`pos`、`quat`、`euler`、`axisangle`、`xyaxes`、`zaxis`、`fromto`、`size`、`rgba`、`material`、`ref`、`user`；
- 已经存在的同名 site 会跳过。

site 在补丁之后写入，所以 `body` 要写产物里的最终名字。

## patch 段

补丁最后应用，用来处理配置里其他选项覆盖不到的情况。

支持的部分：

| 部分 | 作用 |
|---|---|
| `option`、`compiler`、`flags` | 直接写入对应节点 |
| `joints`、`actuators`、`geoms`、`bodies`、`sites`、`equalities`、`sensors` | 按条件匹配元素并修改属性 |
| `rename` | 按正则重命名，并同步所有引用 |
| `remove` | 按条件删除元素 |
| `add` | 直接插入 XML 片段 |

按条件修改的规则写法：

```json
{
  "patch": {
    "joints": [
      { "match": { "name": "*_wheel_joint" }, "actuatorfrcrange": [-100, 100] }
    ],
    "actuators": [
      {
        "match": { "joint": "*_wheel_joint" },
        "type": "velocity",
        "kv": 20,
        "forcerange": [-100, 100],
        "ctrlrange": [-15, 15]
      }
    ],
    "geoms": [
      { "match": { "name": "*_wheel*" }, "friction": [1.0, 0.005, 0.0001] }
    ]
  }
}
```

`match` 使用通配符匹配，可匹配 `name`、`joint`、`body`、`type` 等属性。执行器规则里写 `type` 会把元素转换成对应的执行器类型，同时清理不兼容的属性。

重命名：

```json
{
  "patch": {
    "rename": {
      "body": [
        { "pattern": "^Left_Arm_link7_tcp_", "replacement": "left_gripper_" },
        { "pattern": "^Left_Arm_", "replacement": "left_" }
      ]
    }
  }
}
```

同一个实体可以写多条规则，按顺序执行，后面的规则会看到前面的结果。替换文本支持正则反向引用。

删除和新增：

```json
{
  "patch": {
    "remove": { "geom": ["*_debug_*"] },
    "add": {
      "sensor": ["<framepos name=\"tip\" objtype=\"body\" objname=\"left_link7\"/>"]
    }
  }
}
```

补丁规则没有命中任何元素时会给出警告；`strict_patch` 为 `true` 时直接报错。

## verify 段

| 键 | 默认值 | 说明 |
|---|---|---|
| `enabled` | `true` | 是否在转换后自动验证 |
| `render` | `true` | 是否离屏渲染 |
| `gl` | `null` | 渲染后端，可选 `egl`、`osmesa`、`glfw` |
| `smoke_steps` | `200` | 空控制步进的步数 |
| `views` | `[[135, -20], [45, -10], [225, 0]]` | 渲染视角，每项是方位角和俯仰角 |
| `render_dir` | `null` | 渲染图片目录，默认与输出文件同级 |

## 两个例子

只换末端执行器：

```json
{
  "usd": {
    "variants": { "EE": "AG2F90" }
  }
}
```

关闭视觉几何、关闭渲染、提高凸包精度：

```json
{
  "usd": {
    "max_hull_vertices": 512
  },
  "mjcf": {
    "visuals": false
  },
  "verify": {
    "render": false
  }
}
```

仓库的 `config/` 目录里有两份完整可用的示例配置，分别对应不同的机器人形态，可以直接参考或复制。
