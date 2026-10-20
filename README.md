# usd2mjcf

把 **OpenUSD 物理资产**（Isaac Sim 导出、`MjcPhysics` / Newton / PhysX schema 授权）转换成
**MuJoCo MJCF**，并保证产物：

* **自包含** —— 能被不带 USD 支持的 pip `mujoco` / `mujoco-warp`（mjlab）直接加载；
* **保真** —— USD 里已授权的控制器参数（执行器增益、力限、armature、阻尼、关节限位、
  场景/求解器设置）原样落到 MJCF，**不需要在 MuJoCo 侧重新调参**；
* **可验证** —— 转换后自动编译、短程步进、离屏渲染，并做显式积分器发散守卫。

```mermaid
flowchart LR
  U["USD<br/>(含 payload/variant)"] --> V["选 variant"] --> C["碰撞体处理"] --> P["读场景参数 + 材质 + 命名表"]
  P --> N["Newton add_usd"] --> X["SolverMuJoCo<br/>save_to_mjcf"] --> S["场景同步 → 材质 → 命名 → 补丁"] --> M["MJCF + 验证"]
```

---

## 1. 快速开始

```bash
uv sync                                  # 首次：建 .venv，装 newton / usd-core / mujoco / mujoco-warp

# CLI 只有三个参数：<输入.usda> <输出.xml> [配置.json]
uv run usd2mjcf robot.usda out/robot.xml
uv run usd2mjcf robot.usda out/robot.xml my_config.json
uv run usd2mjcf robot.usda my_config.json        # 第二个参数是 .json 时按配置解析

# 输入也可以是目录（自动挑入口）或 https URL（自动下载依赖闭包）
uv run usd2mjcf path/to/robot_dir/ out/robot.xml
uv run usd2mjcf https://…/robot.usda out/robot.xml

uv run pytest                                   # 单元测试（纯文本层，不需要 GPU/USD）
uv run python scripts/write_example_config.py   # 重新生成 config.example.json
```

产物（`-o` 给目录时）：

```
out/robot/
├── robot.xml
├── robot.report.json        # 完整转换报告（variant/场景/碰撞/命名/材质/实际配置）
├── robot_view0..2.png       # 验证渲染
├── meshes/*.obj             # 外置网格
└── textures/                # 仅当 USD 材质带贴图
```

---

## 2. 配置文件（唯一旋钮面）

**CLI 故意只有三个参数**；其余全部走 JSON 配置。未出现的键取默认值，**拼错的键会给出
显式警告**（`strict_patch` 时直接报错）——静默不生效是本项目最忌讳的失败模式。

完整键表见 [`config.example.json`](config.example.json)（由
`scripts/write_example_config.py` 从 dataclass 生成，`tests/test_config_examples.py`
保证它不会漂移）。

| 键 | 默认 | 说明 |
|---|---|---|
| `usd.path` / `mjcf.path` | CLI 位置参数 | 也可写在配置里（CLI 优先） |
| `usd.variants` | `{}` | **换底盘/机械臂/末端执行器写这里**：`{"Chassis": "LinkhouS2_V2", "EE": "AG2F90"}` |
| `usd.solver_variants` | `["mujoco","newton"]` | 只对"USD 里没授权默认值"的 variant set 生效 |
| `usd.collision_mode` | `enable` | `enable`=重开被资产禁用的碰撞体；`respect`=完全按 USD |
| `usd.fix_self_material_colliders` | `true` | 修掉"自带 `NewtonMaterialAPI` 而被 Newton 整块跳过"的碰撞体（W2 的轮子就是） |
| `usd.add_usd` | `{}` | 透传 `newton.ModelBuilder.add_usd` 参数（如 `{"floating": false}`） |
| `usd.download_dir` | `./.usd_cache` | 远程 https 资产的镜像目录 |
| `usd.schema_priority` | `["mjc","newton","physx"]` | schema resolver 优先级 |
| `mjcf.mesh_mode` | `obj` | `obj`=网格外置成 `.obj`；`inline`=写进 XML（超 `inline_max_vertices` 仍外置） |
| `mjcf.actuator_names` | `joint` | `joint`=执行器用所驱动 joint 的名字命名；`none`=不命名 |
| `mjcf.name_from_usd` | `["joint","body"]` | **名字还原**：Newton 下划线路径名 → USD prim 叶子名（见 §4） |
| `mjcf.fix_ctrlrange` | `true` | 修正执行器 `ctrlrange` 的单位混用与缺失（见 §5.2） |
| `mjcf.ctrlrange_auto_degrees` | `true` | 跨度比 ≈ 180/π 时把 `ctrlrange` 由「度」换算为弧度 |
| `mjcf.ctrlrange_from_joint` | `true` | `ctrlrange` 缺失时用关节限位补 |
| `mjcf.visuals` | `true` | 是否导出仅可视化的几何（关掉产物更小，但渲染是空模型） |
| `mjcf.scene.*` | `timestep=0.002` … | 场景参数**兜底值**；USD 里有就以 USD 为准 |
| `mjcf.materials.*` | `enabled/pbr/hide_colliders` | 颜色传递；`pbr` 默认关（高金属度在 MuJoCo 默认灯光下发黑） |
| `mjcf.contact.excludes` | `[]` | **额外碰撞排除** `[[body1, body2], ...]`，两侧支持 glob（见 §6.2） |
| `mjcf.sites` | `[]` | **补充 `<site>`**：往指定 body 里加 TCP / EEF 标记点，`body` 支持 glob（见 §6.4） |
| `patch` | `{}` | 通用补丁：`option/compiler/flags/joints/actuators/geoms/rename/remove/add`（见 §7） |
| `strict_patch` | `false` | 补丁规则未命中任何元素时直接报错 |
| `backend` | `cuda` | `cpu` 时用 MuJoCo-C 后端导出（无 GPU 环境） |
| `verify.*` | `enabled/render/gl/smoke_steps/views` | 验证与渲染开关 |

---

## 3. variant：默认值怎么定的？是不是 Isaac 独有的？

**variant 不是 Isaac 独有的东西**，它是 USD 自带的核心合成特性（`UsdVariantSet`，
和 reference / payload 一样属于 composition arc）。但**这套 variant 的命名约定**
（`Physics` / `Visual` / `Chassis` / `Arm_Left` / `Head` / `EE` / `Gripper` / `Mesh`）
是**资产作者**定的：Isaac Sim 的资产规范与 URDF/MJCF 导入器按
"一个机器人 = 一份资产 + 若干可切换变体"打包，于是会看到
`Physics=physx|physics|mujoco|none` 这种"按引擎切物理层"的固定套路。

工具选取规则（`usd_scene.apply_solver_variants`）：

1. **配置写了就用配置的**（`usd.variants`），并校验取值是否存在；
2. 否则用 **USD 里作者授权的默认值**（`GetVariantSelection()`）；
3. 只有**作者没授权默认值**时才自动挑：按 `usd.solver_variants` 顺序在候选里找
   （默认先 `mujoco` 再 `newton`）；
4. 都命中不了就**不选**，并在报告里列出候选值 + 给出警告（该 variant set 不贡献内容）。

> 规则 3 存在的原因：Isaac 资产常把 `Physics` 默认值**故意留空**，由运行时 VariantSwitcher
> 按引擎选。不显式选中的话，组合结果里**读不到 `mjc:*` 授权值**（只剩几何）。
>
> 规则 2 优先的原因：有些资产作者授权的是 `Physics=physx`，而 PhysX 层里的
> `DriveAPI` 恰好能被 Newton 换算成 MuJoCo 位置伺服（kp/kv）。盲目改成 `physics`
> 反而会丢掉手指 mimic。

报告里每次都会列出**全部候选值 + 本次选中值 + 原因**：

```jsonc
"variants": [{"prim": "/FiveAges_W2", "variantSet": "Chassis",
              "candidates": ["LinkhouS2_V1", "LinkhouS2_V2", "None"],
              "selection": "LinkhouS2_V1", "reason": "authored"}]
```

---

## 4. 命名：规则、为什么不对称、怎么用正则覆盖

### 4.1 Newton 给的名字

Newton 用 **`_` + prim 路径（`/` → `_`）** 命名实体，所以 MJCF 里最初长这样：

```
joint: _FiveAges_W2_Linkhou_S2_joints_fl_steer_joint
body : _FiveAges_W2_Left_Arm_link7_tcp_AG2F90_gripper_base
```

### 4.2 工具怎么改（`mjcf.name_from_usd`，默认 joint + body）

在 Newton 解析**之前**先建好一张表 `下划线名 → prim 路径分段`
（`usd_scene.collect_prim_names`；必须在解析前建，之后 prim 句柄会过期）：

```
_FiveAges_W2_Left_Arm_joints_left_joint1 → ("FiveAges_W2", "Left_Arm", "joints", "left_joint1")
_FiveAges_W2_Left_Arm_link1              → ("FiveAges_W2", "Left_Arm", "link1")
```

然后按四条规则改（`mjcf_edit.apply_name_map`）：

| # | 规则 | 例子 |
|---|---|---|
| 1 | 叶子名唯一 → 直接用叶子名 | `…_joints_fl_steer_joint` → `fl_steer_joint` |
| 2 | 叶子名撞车 → **同组统一加长路径尾部**，取"最短且唯一"的后缀 | 两侧都叫 `link1` → `Left_Arm_link1` / `Right_Arm_link1` |
| 3 | 加长到极限仍不唯一 → 保留原名，写进报告 `naming.kept` | — |
| 4 | 名字**不在表里**（Newton 合成名）→ 不动，写进报告 `naming.unmatched` | `joint_1`（float base 的自由关节） |

两条附加保证：

* **对称**：深度并列的组（左右臂/左右轮）**同进同退**，不会出现 `link1` / `Right_Arm_link1`；
* **引用同步**：改名同时重写所有引用（`<actuator joint=>`、`<equality joint1/joint2>`、
  `<contact><exclude body1/body2>`…），并做**重名冲突检测**（撞名直接报错，不静默产出畸形模型）。

### 4.3 为什么 body 有 `Left_Arm_` 前缀、joint 却是 `left_joint*`？

规则是同一条，差异来自 **USD 自己的命名**：

| | USD 里的 prim 名 | 叶子名唯一？ | 结果 |
|---|---|---|---|
| joint | `left_joint1` / `right_joint1`（两侧本来就不同名） | ✅ | 直接用叶子名 → `left_joint1` |
| body | `link1` / `link1`（两侧**同名**，只靠父层 `Left_Arm` / `Right_Arm` 区分） | ❌ | 加长到含区分段 → `Left_Arm_link1` |

所以"对不上"不是 bug：**工具只负责把唯一性补齐**，名字不对称是资产作者造成的。
加长用"最短可唯一后缀"，所以已尽量短：`link1` → `Arm_link1` 仍撞 → `Left_Arm_link1`。

### 4.4 为什么末端执行器名字那么长？

`Left_Arm_link7_tcp_AG2F90_gripper_base` 的原始路径是
`("FiveAges_W2","Left_Arm","link7","tcp","AG2F90","gripper_base")`。
左右两个夹爪的叶子名**完全相同**（`gripper_base`、`left_in_link`…），必须一路加长到
第 5 段（`Left_Arm`）才能区分 → 名字自然长。想变短就用 §4.5 的正则。

### 4.5 用配置里的正则重命名（规则 1–4 兜不住的部分）

`patch.rename` 支持按实体类型给**一串正则**（按顺序施加，后面的规则看到前面的结果；
替换串支持反向引用，JSON 里写 `\\1`）：

```jsonc
"patch": {
  "rename": {
    "body": [
      { "pattern": "^Left_Arm_link7_tcp_AG2F90_", "replacement": "left_gripper_" },
      { "pattern": "^Right_Arm_link7_tcp_AG2F90_", "replacement": "right_gripper_" },
      { "pattern": "^Left_Arm_", "replacement": "left_" },
      { "pattern": "^Right_Arm_", "replacement": "right_" }
    ],
    "joint": [ { "pattern": "^joint_1$", "replacement": "base_freejoint" } ],
    "geom":  [ { "pattern": "_visuals_", "replacement": "_v_" } ]
  }
}
```

效果（[`examples/fiveages_w2/config.json`](examples/fiveages_w2/config.json) 里就是这么写的）：

```
Left_Arm_link1                          → left_link1
Right_Arm_link7_tcp_AG2F90_left_in_link → right_gripper_left_in_link
joint_1                                 → base_freejoint
```

未命中任何名字会在报告里**显式警告**（`strict_patch: true` 时直接报错）。
可改的实体类型：`joint / body / geom / site / mesh / actuator`；改名会自动同步引用并检测冲突。

### 4.6 报告里一览

```jsonc
"naming": {
  "renamed":   {"joint": 44, "body": 52},
  "lengthened":{"body": ["_FiveAges_W2_Left_Arm_link1 → Left_Arm_link1", "…"]},
  "kept":      {"body": []},
  "unmatched": {"joint": ["joint_1"]}     // ← Newton 合成名，请用 patch.rename 处理
}
```

---

## 5. 执行器：它们是位置控制器吗？`ctrlrange` 为什么会被改？

### 5.1 产物里是 `<general>`，但它**就是**位置伺服

MJCF 里看到的是

```xml
<general joint="body_joint1" biastype="affine" gainprm="5000" biasprm="0 -5000 -500"
         ctrlrange="-1.3526 0.5672" ctrllimited="true" forcerange="-500 500"/>
```

而不是 `<position kp="5000" kv="500"/>`。两者**完全等价** —— `<position>` 只是
`<general gainprm=[kp] biasprm=[0,-kp,-kv] biastype=affine>` 的语法糖。实测：

```
mujoco.MjModel.actuator_gainprm  逐元素相同
mujoco.MjModel.actuator_biasprm  逐元素相同
300 步轨迹 qpos / actuator_force   max|Δ| = 0.000e+00
```

为什么不直接写成 `<position>`？因为 USD 的 `MjcActuator` 授权的是
`mjc:gainType` / `mjc:biasType` / `mjc:gainPrm` / `mjc:biasPrm`（MuJoCo 的
**通用执行器**三元组），`gainType` 还可以是 `user`/`affine` 等，`biasPrm` 也可以带
更多项 —— 只有 `gainType=fixed & biasType=affine & biasPrm[1]=-kp & biasPrm[2]=-kv`
时才等价于 `<position>`。强行改写成缩写形式会**丢掉 USD 里的原始授权**，所以工具保留
`<general>`，语义不丢。

> 想改成缩写形式？在配置里加补丁即可：
> `"patch": {"actuators": [{"match": {"joint": "body_joint*"}, "type": "position"}]}`
> （`type` 转换会顺手把 `gainprm/biasprm` 换成 `kp/kv` 并清掉 general 专有属性。）

### 5.2 USD 里的 `ctrlRange` 单位是**混用**的（真 bug，工具已修）

同一个 W2 资产里：

| 执行器 | USD `mjc:ctrlRange` | 关节 `range`（rad） | 单位 |
|---|---|---|---|
| `fl_steer_joint` | `-3.14159 .. 3.14159` | `-3.14159 .. 3.14159` | 弧度 ✅ |
| `body_joint1` | `-77.49999 .. 32.5` | `-1.3526 .. 0.5672` | **度** ❌ |
| `left_joint1` | `-178 .. 178` | `-3.1067 .. 3.1067` | **度** ❌ |
| `head_joint1` | `-24.981 .. 24.981` | `-0.4360 .. 0.4360` | **度** ❌ |

MJCF 是 `angle="radian"`，照抄会让控制范围放大 **57.3 倍** —— 策略输出的目标角落在
物理限位之外，伺服会一直顶到力矩上限（表现为「关节不动 / 一直推到底」）。

工具默认修（`mjcf.fix_ctrlrange: true`）：判据是 **`ctrlrange` 跨度 / 关节 `range` 跨度
≈ 180/π**（容差 5%）。实测 W2 修正 20 个、保留 4 个本来就正确的转向轮，最终所有
执行器的 `ctrlrange` 与关节 `range` 跨度比均为 **1.000**。

### 5.3 `ctrllimited` 但 `ctrlrange` 为空 → 补关节限位

夹爪执行器在 USD 里写了 `ctrlLimited=true, ctrlRange=0..57.3`，但产物里成了
`ctrlrange="0 0"` + `ctrllimited="false"` —— 等于**控制量完全不限幅**。
工具会在 `ctrlrange` 缺失/为零且是位置类执行器时，用关节限位补上并开启 `ctrllimited`
（W2 的左右夹爪被补成 `0..1`）。速度伺服不受影响（它的控制量是角速度，不是角度）。

### 5.4 各类执行器的实际形态

| 关节族 | 产物形式 | kp / kv | 力限 |
|---|---|---|---|
| 转向轮 `*_steer_joint` | `<general>`（位置） | 625 / 80 | ±200 |
| 驱动轮 `*_wheel_joint` | `<velocity>`（**唯一由配置改写**） | kv=20 | ±100 |
| 腰 `body_joint*` | `<general>`（位置） | 5000 / 500 | ±500 |
| 头 `head_joint*` | `<general>`（位置） | 625 / 50 | ±100 |
| 臂 `left/right_joint*` | `<general>`（位置） | 800 / 50 | ±108 / ±66 / ±18 |
| 夹爪 `*_gripper_joint` | `<general>`（位置） | 572.958 / 57.2958 | ±5 |

数值全部来自 USD 的 `mjc:gainPrm` / `mjc:biasPrm`（手臂是 800/50，不是旧 README 里
写的 200/10 —— 那是上一代 URDF 链路的值）。驱动轮是唯一一处**刻意偏离**：Isaac 给的是
`kv=100` + 力矩 `±1000 N·m` + `ctrl` 不限幅，RL 里策略自由输出 `ctrl` 会让力矩长期贴上限。

---

## 6. 几何、碰撞体、材质、timestep：产物里的东西都从哪来

| 产物里的东西 | 来源 | 说明 |
|---|---|---|
| **几何类型** | **USD** | USD 里是 `Cylinder` → MJCF `<geom type="cylinder">`（W2 的轮子）；USD 里是 `Mesh` + `physics:approximation=convexHull` → 凸包 mesh geom（W2 的头部/腰部/底盘）。工具不会把 mesh 换成参数化几何，也不会替 USD 补碰撞体 —— **USD 里没有 `collisions/` 就没有碰撞**（W2 的转向节如此）。 |
| **网格序列化** | 工具（可配） | MuJoCo 的 `to_xml()` 对"由数组构造的网格"会写成**内联 `vertex`/`face`**。本工具默认把它们**外置成 `meshes/*.obj`**（`mjcf.mesh_mode: "obj"`），XML 只留 `file=`；`"inline"` 可保留内联，但超过 `inline_max_vertices` 的仍外置（防 XML 爆到上百 MB）。 |
| **碰撞体是否参与接触** | USD + 工具 | ①`physics:collisionEnabled=0` 的碰撞体：对 PhysX 是"自碰撞排除"，对 MJCF 等于**没有碰撞** → `usd.collision_mode: "enable"`（默认）重开并列出全部路径；②碰撞体自带 `NewtonMaterialAPI` 时会被 Newton **整块跳过**（W2 的 4 个轮子）→ `usd.fix_self_material_colliders`（默认开）转成普通碰撞体 + 独立材质 prim；③`<contact><exclude>` 的来源见 §6.1。 |
| **颜色/材质** | USD | Newton 的 MJCF 出口**不带视觉材质**。工具按 `diffuseColor` → OmniPBR `base_color_factor` → `displayColor` → `UsdUVTexture`（复制贴图）取色，写成 `<material>`/`<texture>`。顺序有讲究：必须遍历**实例代理**（视觉几何常在 `instanceable=true` 容器里），且必须在 Newton 解析**之前**扫完（之后 prim 句柄会过期）。 |
| **`<option timestep>`** | **必须由工具补** | Newton 的 `SolverMuJoCo` 从不设置 `spec.option.timestep`（`dt` 是每步传参的），不补会静默退回 MuJoCo 默认 2 ms。取值顺序：`mjc:option:timestep` > `newton:timeStepsPerSecond`（Hz→秒）> 配置 `mjcf.scene.timestep` 兜底；报告标注每个值**实际来源**（`usd` / `config`）。 |
| 其它 `<option>`/`<compiler>`/`<flag>` | USD → 工具 | `mjc:option:*` / `mjc:flag:*` / `mjc:compiler:*` / `physics:gravity*` 直通。 |
| **执行器** | USD | `MjcActuator` 的 `mjc:gainPrm/biasPrm/gainType/biasType/target/forceRange/ctrlRange` → `<general>`；关节级 `mjc:armature/damping/actuatorfrcrange` → `<joint>` 属性。没有 `mjc:` 时走 PhysX `DriveAPI`（per-degree → per-radian 由 Newton 换算，例如把 `/deg` 的 stiffness 折成 `/rad`）。 |
| **执行器 `name`** | 工具 | Newton 不写执行器名，而 RL 侧几乎都按名字索引 → `mjcf.actuator_names: "joint"` 用所驱动 joint 的名字命名（一个 joint 多个执行器时追加 `_0/_1`）。 |

### 6.1 那些 `<contact><exclude>` 是怎么来的？

W2 产物里有 **663 条** exclude。它们**不是** USD 里逐条写出来的，而是
Newton 从**形状级**过滤规则推导成 **body 级**排除的结果（`SolverMuJoCo._find_body_collision_filter_pairs`）：

```
对每一对 body：若它的所有 shape 组合都被过滤 → 整对 body 生成一条 <exclude>
```

形状级过滤来自三处（都在 USD 里）：

| 来源 | USD 属性 | W2 实际情况 |
|---|---|---|
| 自碰撞开关 | `newton:selfCollisionEnabled` / `physxArticulation:enabledSelfCollisions` | `base_link`、双肩 = `true`；`base_footprint`、`Head_V1`、两个夹爪根 = `false` |
| 形状级过滤对 | `physics:filteredPairs` | 33 对 |
| 碰撞组 | `PhysicsCollisionGroup` / `physics:filteredGroups` | 用于自碰撞排除 |

**为什么数量这么大**：`_find_body_collision_filter_pairs` 的判据是"**全部** shape 组合都被
过滤"。W2 有 52 个 body、37 个带碰撞体，其中大量 body 只有 **1 个**碰撞体 —— 只要它与
另一个单碰撞体 body 的那一对被过滤，就立即构成一条 body 级 exclude。所以 33 对形状级过滤
可以放大成几百条 body 级排除。这是**正常且正确**的（MuJoCo 的 exclude 是 O(1) 查表，
不影响性能）。

> 反过来说，**只有当一对 body 的所有 shape 组合都被过滤时才会生成 exclude**。如果某个
> body 有多个碰撞体、其中任意一对没被过滤，整对 body 就不会被排除 —— 这正是下面
> 需要手工补的原因。

### 6.2 补 USD 没表达的重叠部位（`mjcf.contact.excludes`）

W2 里有两类"物理上必然重叠、但 USD 没写成形状级过滤"的部位：

1. **底盘 × 腰部升降**：升降柱坐在底盘壳内；
2. **底盘 × 驱动轮/转向节**：轮子的碰撞体被 `fix_self_material_colliders` 重新打开后，
   与底盘壳体相交（转向节本来就没有碰撞体，但它的父级外壳有）。

这类情况在配置里补（两侧都支持 glob，已存在的会跳过，body 名不存在会**显式警告**）：

```jsonc
"mjcf": {
  "contact": {
    "excludes": [
      ["base_footprint", "body_link*"],   // 底盘 × 腰部升降
      ["base_footprint", "*_wheel"],      // 底盘 × 驱动轮
      ["base_footprint", "*_steer"],      // 底盘 × 转向节
      ["base_link", "*_wheel"],
      ["base_link", "*_steer"]
    ]
  }
}
```

实测（`examples/fiveages_w2/config.json`）：新增 11 条、9 条已存在被跳过，产物 exclude
从 652 → **663**，`base_footprint × {body_link*, *_wheel, *_steer}` 全部覆盖。

**写在哪一步生效**：在名字还原之后、通用补丁之前，所以 `excludes` 里写的是**产物里的
body 名**（`base_footprint`、`left_link1`…），不是 USD prim 路径。

> 如果你在 `simulate` 里看到底盘和轮子互相"打架"（整机抖动 / 轮子被顶飞），八成就是
> 缺了这几条。注意 MuJoCo 默认已自动排除 **父子** body 对，所以只需关心**非父子但重叠**
> 的部位。

### 6.3 视觉体 / 碰撞体分离（`group`）

MuJoCo 用 geom 的 `group` 区分“默认渲染”和“只在需要时显示”：

| group | 默认可见 | 用途 |
|---|---|---|
| 0 / 1 / 2 | ✅ | 渲染 |
| **3** | ❌ | **碰撞专用** |
| 4 / 5 | ❌ | 其它辅助 |

工具在**碰撞掩码定稿之后**做一次分组（`mjcf_edit.separate_visual_collision`）：

* **碰撞体** → `group=3`（无材质时给中性灰，方便你在 viewer 里打开 3 组检查碰撞形状）；
* **视觉体** → `group=0`。

“谁是碰撞体”的判据有两层：

1. `contype`/`conaffinity` 非零；
2. 名字包含 `/collisions/`（USD 里被禁用、或 Newton 直接丢弃的碰撞体，掩码会是 0，
   但它们本来就不该渲染）。

> **为什么必须晚于掩码修复**：S2 的 USD 关闭了自碰撞，Newton 把掩码全算成 0；如果先
> 做分组，就会把这些碰撞体当成视觉体（曾经的 bug：S2 里碰撞盒/碰撞圆柱盖在视觉网格上）。
> 顺序现在固定为：材质 → 掩码修复 → **视觉/碰撞分组** → ctrlrange → 命名 → excludes → 补丁 → **sites**。

实测：

| 产物 | 碰撞体 → group 3 | 视觉体 → group 0 |
|---|---|---|
| W2 | 52 | 154 |
| S2 | 47 | 123 |

在 `simulate` 里按 `Tab` 打开右侧面板，`Rendering → Geom Groups` 勾选/取消第 3 组即可
单独检查碰撞形状；`mjcf.materials.hide_colliders: false` 可以让两者同时渲染（调试对称性时有用）。

### 6.4 补充 `<site>`（`mjcf.sites`）

Newton 的 `save_to_mjcf` **不保留** USD 里的 `MjcSite` / 变换标记，所以 TCP、EEF、
相机光心这些 RL 侧最常用的参考系会整块丢失。手改产物下一次转换就没了，写在配置里才可
版本管理（两个 `examples/*/config.json` 都有完整示例）：

```jsonc
"mjcf": {
  "sites": [
    {
      "name": "LEFT_TCP_SITE",         // 必填，全局唯一
      "body": "left_link7",            // 必填，产物里的 body 名，支持 glob
      "pos": [0, -0.095, 0],
      "quat": [0.5, 0.5, -0.5, 0.5],   // 省略 = 与 body 同姿态
      "size": 0.005,
      "rgba": [0.2, 0.9, 0.2, 0.5]
    },
    {
      "name": "LEFT_EEF_SITE",
      "body": "left_gripper_gripper_base",
      "pos": [0, 0, 0.19],
      "size": 0.005,
      "rgba": [0.2, 0.9, 0.2, 0.5],
      "optional": true                 // body 不存在时只记日志，不告警
    }
  ]
}
```

规则：

* `body` / `name` 必填；`body` 用 glob 展开，**命中多个 body 时只写第一个并告警**
  （site 名全局唯一，左右两侧请拆成两条规则）；
* `optional: true` 用于**随 variant 变化**的部件（例如 `EE=None` 时夹爪 base 不存在）；
* 其余键（`type` / `group` / `pos` / `quat` / `euler` / `axisangle` / `xyaxes` /
  `zaxis` / `fromto` / `size` / `rgba` / `material` / `ref` / `user`）原样写进 `<site>`；
  拼错的键在**加载配置时**就会变成 warning（`strict_patch: true` 时直接报错）；
* 产物里已有同名 site 时跳过，重复转换幂等。

**写在哪一步生效**：通用补丁之后 —— 所以 `body` 写的是**产物最终的名字**
（`left_link7`、`left_gripper_gripper_base`），不是 USD prim 路径，也不是
`patch.rename` 之前的中转名。这样你看到产物里叫什么，配置里就写什么。

实测：S2 / W2 在 `EE=None` 时，两个 TCP site 正常写入（`nsite=2`），EEF 规则按
`optional` 静默跳过；把 `EE` 换成 `AG2F90` 后四条全部出现，与参考产物逐字一致。

---

## 7. 通用补丁（`patch`）

写在配置里（完整示例见 [`examples/fiveages_w2/config.json`](examples/fiveages_w2/config.json)）：

```jsonc
"patch": {
  "option":   { "integrator": "implicitfast" },     // 直通 <option>
  "compiler": { "balanceinertia": true },           // 直通 <compiler>
  "flags":    { "refsafe": true },                  // <flag> 开关

  "joints":    [ { "match": {"name": "*_wheel_joint"}, "actuatorfrcrange": [-100, 100] } ],
  "actuators": [ { "match": {"joint": "*_wheel_joint"},
                   "type": "velocity",               // 顺带做 position→velocity 转换
                   "kv": 20, "forcerange": [-100, 100], "ctrlrange": [-15, 15] } ],
  "geoms":     [ { "match": {"name": "*_wheel*"}, "friction": [1.0, 0.005, 0.0001] } ],

  "rename": { "joint": { "pattern": "^joint_1$", "replacement": "base_freejoint" } },
  "remove": { "equality": ["*_gripper_joint"] },    // 按 joint1/joint2 也能匹配
  "add":    { "sensor": ["<framepos name=\"tip\" objtype=\"body\" objname=\"left_link7\"/>"] }
}
```

* `match` 用 glob（`*` `?`），可匹配 `name` / `joint` / `body` / `type` 等属性；
* `type` 转换会顺手清掉缩写形式不接受的 general 专有属性（`biasprm` 等）；
* 规则**未命中任何元素**会在报告里警告（`strict_patch: true` 时报错）；
* 驱动轮偏离写在这里的原因：Isaac 给驱动轮 `kv=100` + 力矩 `±1000 N·m` + `ctrl` 不限幅，
  RL 里策略自由输出 `ctrl`，力矩会长期贴上限。这类"必须人工偏离"的项放在配置里可审计、可版本管理。

---

## 8. 验证（自动执行）

用**不带 USD 支持**的 pip `mujoco` 编译，所以"通过"本身就证明产物**自包含**：

| 检查 | 内容 |
|---|---|
| 编译 | `MjModel.from_xml_path`（不需要 USD，不需要 GPU） |
| 统计 | `njnt/nq/nv/nu/nbody/ngeom/nmesh/neq`、总质量、`excludes` |
| **积分器守卫** | `dt·kd/(M0+armature)` 最大比值；若 `≥2` 且积分器是 Euler → 明确警告"显式积分解必发散" |
| 步进 | 默认 200 步零控制，检查 NaN/Inf 与 `|qvel|` 峰值 |
| 渲染 | EGL/OSMesa 离屏渲染 3 个视角（自动适配离屏缓冲上限） |

渲染后端不可用不会让转换失败，只给可执行建议（`verify.gl` 或 `verify.render: false`）。
**无显示环境（容器/CI）不要用 `simulate` GUI**。

---

## 9. 已知限制

* `save_to_mjcf`（Newton 官方唯一的 MJCF 出口）**不是完整语义往返**：`<sensor>`、相机/灯光、
  `<keyframe>`、`<composite>/<flex>`、`<skin>`、`<plugin>` 不导入；loop closure 只保留运动学耦合；
  `joint_velocity_limit` 被丢弃；kinematic root 的 `joint_armature` 会被改成 `1e10`。
* 非凸网格一律**凸包化**（MuJoCo 只支持凸碰撞体）；缺 `scipy`/`trimesh`/`coacd` 时会静默退化成
  包围盒，所以依赖里强制带上 `newton[importers]`。
* 远程资产镜像只跟随**相对路径**依赖；若原作者的 payload 引用了绝对路径（例如
  Robotiq 2F-85 的 `/Robotiq/2F-85/...`），该 variant 不可用，工具会明确报出缺失文件。
* 给 mjlab 用：`nconmax/njmax/nvmax` 是 GPU 侧缓冲，**不要**从 Isaac 的 `/PhysicsScene` 搬；
  转换器用的 `mujoco` 版本可能与 mjlab pin 的不同，纯文本 XML 跨版本可读，建议在 mjlab 环境再编译一次。

---

## 10. 示例

| 例子 | 资产 | 演示 |
|---|---|---|
| [`examples/fiveages_w2`](examples/fiveages_w2/) | FiveAges W2 轮式双臂人形（完整 `mjc:` 授权） | 换底盘/机械臂/末端执行器；正则改名；额外碰撞排除；驱动轮唯一一处偏离 |
| [`examples/fiveages_s2`](examples/fiveages_s2/) | FiveAges S2 固定立柱双臂人形（完整 `mjc:` 授权） | 固定基座；USD 关闭自碰撞导致掩码全 0 的修复；无需手写 excludes |

> 示例只用**带 `mjc:` 授权**的资产。纯 PhysX 授权的资产（如一些官方示例机器人）
> 虽然也能转（靠 `DriveAPI` 换算增益），但没有 `mjc:` 这层，执行器力限 / `armature` /
> 关节级参数都只能靠猜测或手填，不适合当作“保真转换”的范例。

---

## 11. 代码结构

```
src/usd2mjcf/
├── cli.py          # 三个参数的命令行
├── config.py       # 配置 schema / 加载保存 / 校验（唯一旋钮面）
├── convert.py      # 编排：USD → Newton → MJCF → 后处理
├── usd_scene.py    # stage/variant/场景参数/碰撞体/命名表/远程闭包下载/入口发现
├── materials.py    # USD 材质与颜色解析
├── mjcf_edit.py    # 场景同步 + 材质 + 命名还原 + site/exclude 补充 + 通用补丁
└── verify.py       # 编译/步进/渲染/积分器守卫
```

| 依赖 | 版本 | 作用 |
|---|---|---|
| Python | 3.12（uv 管理） | |
| `newton[importers]` | ≥1.7,<2 | USD 解析 + `save_to_mjcf` 出口 |
| `usd-core` | ≥25.5,<26.9 | OpenUSD 运行时 |
| `newton-usd-schemas` | ≥0.5 | `newton:*` codeless schema |
| `mujoco` / `mujoco-warp` | ~3.15.0 | 验证（与 mjlab 同源） |
| `scipy` / `trimesh` / `coacd` | 随 importers | 碰撞网格凸包/分解 |

GPU：Newton 的 `SolverMuJoCo` 依赖 Warp，CUDA 需要驱动 **R580+**（Warp 1.18）；
无 GPU 时用 `backend: "cpu"`，或只导出不验证。
