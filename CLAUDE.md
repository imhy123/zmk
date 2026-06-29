# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> 本仓库是上游 `zmkfirmware/zmk` 的 fork，当前工作分支 `v0.3-branch` 基于上游的 `v0.3-branch`（发布维护分支），在其之上对 EC11 编码器驱动做了若干修改，目的是缓解滚轮响应"慢"或"抖"的问题。下面的所有说明都用中文。

## 仓库定位

ZMK 是基于 [Zephyr RTOS](https://www.zephyrproject.org/) 的机械键盘固件。本仓库本身只是一个 **Zephyr 模块 / 应用**：编译时需要通过 `west` 拉取 Zephyr 主仓和若干第三方模块（`app/west.yml`），然后由 Zephyr 的 CMake 系统驱动生成最终固件。

直接 `cmake` / `make` 是不行的，必须走 `west`。

## 常用命令

### 初始化工作区（首次或换分支后）

```sh
# 在 zmk/ 的上一级目录中
west init -l zmk/app
west update          # 拉取 Zephyr + 各模块
west zephyr-export
```

### 构建固件

```sh
# 在 zmk/ 目录下，给某个 board 构建（示例：nice_nano_v2 + corne_left shield）
west build -s app -b nice_nano_v2 -- -DSHIELD=corne_left

# 构建后的固件： build/zephyr/zmk.uf2
```

### 运行测试（snapshot 测试，本机就能跑）

`app/run-test.sh` 是测试入口。每个用例在 `app/tests/<group>/<case>/` 下，包含一个
`native_posix_64.keymap`，构建后产生的事件流会和 `keycode_events.snapshot` 做 diff。

```sh
cd app
./run-test.sh tests/encoders/rotate         # 跑单个用例
./run-test.sh tests/encoders                # 跑某一组
./run-test.sh all                           # 跑所有用例（并行 4 个，可用 J=n 调）

# 调试常用环境变量：
#   ZMK_TESTS_VERBOSE=1       打开 west build 的完整输出
#   ZMK_TESTS_AUTO_ACCEPT=1   把当前输出当作新 snapshot 写回（修改预期行为时用）
```

BLE 集成测试用 `./run-ble-test.sh`，依赖 nRF52 BSim，必须先设置 `BSIM_OUT_PATH`，通常 CI 才会跑。

### Lint / 格式化

```sh
# C 代码：clang-format（pre-commit 会自动跑）
clang-format -i path/to/file.c

# board 描述的 YAML：
cd app && npm run prettier:format       # 检查用 prettier:check

# 文档（Docusaurus，在 docs/ 下）
cd docs && npm ci && npm start          # 本地预览
cd docs && npm run lint                 # ESLint
cd docs && npm run prettier:format
cd docs && npm run typecheck

# 一次性安装所有 pre-commit hooks（clang-format / prettier / gitlint / trailing-whitespace 等）
pip3 install pre-commit && pre-commit install
```

提交信息走 [Conventional Commits](https://www.conventionalcommits.org/)（`gitlint` 在 pre-commit 中强制）。

## 代码总体结构

```
app/                       主 Zephyr 应用（=固件入口）
├── src/                   核心 C 源码
│   ├── main.c             ZMK 启动入口，只做 settings_load + display_init
│   ├── event_manager.c    所有功能模块之间的事件总线（核心）
│   ├── events/            事件类型定义（keycode/sensor/layer/battery/...）
│   ├── behaviors/         键位行为实现（key_press、hold_tap、macro、sensor_rotate ...）
│   ├── pointing/          指针/鼠标输入处理（input listener、processors）
│   ├── split/             分体键盘的中心/外设通信
│   ├── sensors.c          扫描 ZMK_KEYMAP_SENSORS_NODE 下挂的所有传感器并触发事件
│   └── ...                ble/usb/hid/display/battery/backlight/rgb/...
├── include/zmk/           对内公开的 ZMK 头文件
├── include/drivers/       自定义驱动接口（behavior、input_processor 等用 syscall 暴露）
├── boards/                板级 .overlay / .conf / shield 定义
├── module/                作为 Zephyr 模块挂入的自定义驱动
│   ├── drivers/sensor/ec11/   ★ EC11 编码器驱动（本分支的修改重点）
│   ├── drivers/{input,gpio,display,kscan}/
│   └── dts/bindings/      自定义 devicetree 绑定（YAML）
├── snippets/              可复用的 Kconfig 片段
├── tests/                 native_posix_64 上的 snapshot 行为测试
├── west.yml               模块清单：固定 zephyr v3.5.0+zmk-fixes
└── run-test.sh / run-ble-test.sh
docs/                      官网文档（Docusaurus 3，独立 npm 项目）
```

### 几个关键架构概念

- **事件总线 (`event_manager`)**：键扫描、传感器、BLE 等模块都把变化推到事件总线，行为/输出端再订阅。新增功能基本都是"发事件 + 订阅事件"。
- **Behaviors 由 devicetree 定义**：每个 behavior 在 `.dts/.overlay` 里声明一个节点，C 端用 `BEHAVIOR_DT_INST_DEFINE` 通过 `DT_INST_FOREACH_STATUS_OKAY` 展开生成实例。改 behavior 通常需要同步改 `dts/bindings/` 下的 YAML。
- **Sensors**：keymap 中 `&sensors` 节点列出的设备，由 `app/src/sensors.c` 统一 `sample_fetch` + `channel_get`，把 `SENSOR_CHAN_ROTATION` 转成 `sensor_event`，再交给 `behavior_sensor_rotate*` 去映射成按键。
- **分体键盘**：同一份 app 既能编成 central 也能编成 peripheral，`CMakeLists.txt` 里很多 `target_sources` 都加了 `if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)` 这类条件——给中心/外设加新文件时要看清楚归属。
- **Kconfig + devicetree 双驱**：功能开关都在 `Kconfig*` 中，硬件接线在 `boards/` 的 `.overlay` 中。新加属性时记得同时改 `module/dts/bindings/.../*.yaml`，否则 dtc 会报 unknown property。

## 本分支当前的"在途修改"（EC11 编码器）

涉及文件：

- `app/module/drivers/sensor/ec11/ec11.c`
- `app/module/drivers/sensor/ec11/ec11.h`
- `app/module/dts/bindings/sensor/alps,ec11.yaml`

新增了两个 devicetree 属性，目的是过滤 EC11 的抖动 / 一格多触发：

| 属性 | 类型 | 作用 |
|---|---|---|
| `pulses-per-detent` | int，默认 1 | 累计多少个有效正交跳变才算一格（`drv_data->accum` 攒够后才更新 `pulses`） |
| `debounce-us` | int，默认 0 | 两次 `sample_fetch` 间隔小于该值则直接忽略（去抖窗口） |

用户侧用法示例（在 `.overlay`）：

```dts
&left_encoder {
    pulses-per-detent = <4>;
    debounce-us = <500>;
};
```

### 在 EC11 上下文工作时务必注意

- **正确的 Zephyr 头文件是 `<zephyr/sys_clock.h>`，不是 `<zephyr/sys/clock.h>`**。
  历史里有过一次 typo 修复（commit `dec2c220`），别再改回去。
- **debug 提交未清理**：`ec11.c` 当前用 `LOG_MODULE_REGISTER(EC11, LOG_LEVEL_DBG)` 把日志级别钉死成 DBG，而不是 `CONFIG_SENSOR_LOG_LEVEL`。这是临时调试用的 commit (`c5f72e39`)，**向上游 PR 前必须还原**。
- `pulses-per-detent` / `debounce-us` 是新增可选属性，对没有声明这两项的旧 keymap 完全向后兼容（DT 默认值就是关掉过滤行为）。
- 任何改这块的实验，至少应跑 `./run-test.sh tests/encoders` 看 snapshot 不回归。

## 分支与提交

- 本仓库的 `v0.3-branch` 是从上游 `zmkfirmware/zmk` 的 `v0.3-branch`（发布维护分支）fork 出来的，**不是** main。
- 日常改动直接提交并 push 到本 fork 的 `v0.3-branch`——CCK-BALL 的 `config/west.yml` 就是指向这个分支，push 后 CI 会自动拉取重编。
- 如果要把改动回馈给上游 ZMK：开发主线在上游 `main`，新功能通常对 `main` 提 PR；`v0.3-branch` 是发布维护分支，一般只接 backport。
- commit message 走 conventional commits（CI 用 `gitlint` 校验）。
- 提 PR 前一定要先跑 `pre-commit run -a`：clang-format 不通过会被立刻挡下来。
