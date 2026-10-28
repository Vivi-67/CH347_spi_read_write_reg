# WL1606 收发测试说明

适用脚本：同目录下的 `wl160x_test_mode.py`。核对日期：2026-10-28。

本说明按当前脚本的实际代码编写。测试前请先阅读“当前版本注意事项”。

## RX 模式与共用参数

```powershell
python wl160x_test_mode.py --mode 1 --pkt_type 0x55 --len 10 --chan 40 --out_rx_result_time 1000
```

此命令在 2440 MHz 持续接收，预期每包 payload 为 10 字节的 `0x55`，每 1000 ms 输出一次累计统计；按 `Ctrl+C` 停止并关闭设备。

| 参数 | 说明 | 默认值 |
|---|---|---|
| `--mode` | `0` 为 TX，`1` 为 RX | `0` |
| `--pkt_type` | TX 生成的数据 / RX 预期数据类型；RX 使用 `random` 会报错 | `0xff` |
| `--len` | TX payload 长度 / RX 预期 payload 长度；RX 短包支持 `1～63` 字节 | `20` |
| `--chan` | 共用信道，频率为 `2400 + chan` MHz | `40` |
| `--out_rx_result_time` | RX 统计间隔，单位 ms，必须为有限正数 | `1000` |

旧名 `--tx_pkt_type`、`--tx_len`、`--tx_chan` 保留为共用参数的兼容别名。
RX 忽略 `--tx_test_mode`、`--tx_cnt`、`--tx_interval`、`--tx_pwr`、`--tx_timeout`；TX 忽略 `--out_rx_result_time`，包括这些专用参数中不适用于当前方向的数值校验。

输出格式：

```text
Rx cnt: 0, Crc_err_cnt: 0,  Invalid pkt cnt:0
Rx cnt: 10, Crc_err_cnt: 0,  Invalid pkt cnt:0
Rx cnt: 15, Crc_err_cnt: 0,  Invalid pkt cnt:0
Rx cnt: 25, Crc_err_cnt: 0,  Invalid pkt cnt:0
```

- `Rx cnt`：CRC 正确、长度符合预期且全部 payload 字节符合 `pkt_type` 的包数。
- `Crc_err_cnt`：硬件 `reg0x60[7]` 指示 CRC 错误的包数；不重复计入无效包。
- `Invalid pkt cnt`：CRC 正确，但 FIFO 读出长度或 payload 内容不符合预期的包数。

三项均从本次启动开始累计，不在输出后清零。没有新包也会定时输出当前计数。统计使用单调时钟，SPI 调用和 Windows 调度会使实际打印时间略有延迟。

接收完成通过 `reg0x61[6]` 判断，FIFO 默认使用每字节独立的 `CS↓ → 0xE4 → 读1字节 → CS↑` 事务读取（`--fifo-read-style single`），可用 `--fifo-read-style burst` 切回原来的连续读取以对照抓线，并按 C 驱动对 payload 逐字节反转位序。每包处理后都会清标志、清 RX FIFO 读指针并重新进入 RX。当前 `arm_rx()` 顺序为：清 `reg0x25[1:0]`、清 `reg0x0F[7]`、清 `reg0x0E[0]`、请求等待10 µs、置 `reg0x0F[7]`，最后连续两次置 `reg0x69[7]` 清 RX FIFO 指针。初始化还会置 `reg0x68[7]` 清 TX FIFO。连续1秒未收包时只更新计时器，不退出或重新使能 RX，因为超时分支的 `radio.arm_rx()` 已注释。

**包长与兼容性：** 本实现沿用参考 RX 序列的固定包长配置（`reg0x5F=0x80|len`；脚本不主动清 `reg0x52[5]`，对应调用已注释），与当前脚本的短包 TX 格式一致，不读取长度前缀。`receive_packet(expect_len)` 使用 `--len` 作为 `expect_len`，在 CRC 正确后从 FIFO 读取指定数量的字节，与 C 驱动固定包长分支一致，不再通过 FIFO 指针差推导长度。固定包长硬件按照配置长度进行 CRC 校验，因此空口包长不一致时，可能表现为 CRC 错误或无法完成收包，而不一定计入 `Invalid pkt cnt`。当前格式不能独立测出原始空口包长。

每个 CRC 正确的包均打印反转位序后的 payload，例如 `time: 2026-10-28 14:00:00.123456, Rx data: 55 55 (2 bytes)`，无需开启 `--verbose`。即使数据内容不符合 `--pkt_type`，仍会打印并计入 `Invalid pkt cnt`；CRC 错误包打印 `CRC err!!!`，不读取 FIFO payload。

离线检查（不连接硬件、不发射）：

```powershell
python wl160x_test_mode.py --mode 1 --pkt_type 0x55 --len 10 --dry-run
python -B -m unittest test_wl160x_rx -v
```

RX `--dry-run` 只输出一次零计数后退出。现有测试文件含历史流程预期，不保证完整测试集通过；当前构造函数增加了 `spi_mode`，部分旧测试的参数传递、重启顺序、超时恢复及日志预期需要另行适配。本说明更新未修改测试或运行逻辑。离线结果不代表完成了实板射频验证。

## 1. 测试准备

1. 使用 Windows 系统和 **64 位 Python 3.8 或以上版本**。
2. 安装 CH347 驱动，将 CH347 连接到电脑，并按测试板要求接好 SPI、共地和电源。
3. 将 `CH347DLLA64.dll` 放在脚本同一目录；放在其他目录时，通过 `--dll` 指定。
4. 打开 PowerShell，进入脚本目录：

```powershell
cd C:\CMS\TEMP_TEST\CH347_spi_read_write_reg
```

只检查参数和操作流程、不连接硬件时，可以使用 `--dry-run`，不需要加载 DLL。

## 2. 命令格式

```text
python wl160x_test_mode.py [参数名 参数值] [开关参数]
```

完整格式如下。方括号表示可选内容，实际输入时不要输入方括号。

```text
python wl160x_test_mode.py [--mode 0或1] [--spi_mode 3-wire或4-wire] [--fifo-read-style single或burst] [--spi-cs-setup-us 微秒] [--spi-cs-hold-us 微秒] [--out_rx_result_time 毫秒] [--tx_test_mode 模式] [--chan 信道] [--tx_pwr 功率档] [--len 字节数] [--pkt_type 数据类型] [--tx_cnt 包数] [--tx_interval 毫秒] [--dev_index 设备索引] [--dll DLL路径] [--tx_timeout 毫秒] [--verbose] [--dry-run]
```

- 未填写的参数使用默认值；参数顺序不限。
- 整数参数支持十进制和 `0x` 开头的十六进制。例如，`32` 与 `0x20` 相同。
- `--tx_interval`、`--tx_timeout` 使用十进制数，支持小数，例如 `0.5`。
- `--verbose`、`--dry-run` 后面不填数值；不写即关闭，写上即开启。
- 查看脚本帮助：`python wl160x_test_mode.py --help`。
- **停止测试：在运行窗口按 `Ctrl+C`。** 脚本会尝试停止发射、复位 WL1606 并关闭 CH347。

## 3. 参数表

| 参数 | 作用 | 范围或可选值 | 默认值 |
|---|---|---|---|
| `--mode` | 收发方向 | `0`：TX；`1`：RX | `0` |
| `--spi_mode` | packet TX/RX初始化时写WL1606寄存器0x5E | `3-wire`：写0x80；`4-wire`：写0x00；不切换CH347数据引脚方向，CW不使用 | `3-wire` |
| `--fifo-read-style` | RX FIFO事务结构 | `single`：每字节独立E4/CS；`burst`：一次E4连续读len字节 | `single` |
| `--spi-cs-setup-us` | 写入DLL的CS建立延时字段，实验参数 | 整数0～65535 µs；当前实测未观察到影响 | `0` |
| `--spi-cs-hold-us` | 写入DLL的CS撤销前延时字段，实验参数 | 整数0～4294967295 µs；当前实测未观察到影响 | `0` |
| `--tx_test_mode` | 选择发射模式 | `0`：CW 单载波；`1`：packet 数据包 | `1`，数据包模式 |
| `--chan` | 设置信道，目标频率为 `2400 + 信道` MHz | packet数据包：`0～80`；CW：`2～80` | `40`，即 2440 MHz |
| `--tx_pwr` | 设置发射功率档位 | `1～15`，1 档最大，15 档最小 | `1`，约 8 dBm |
| `--len` | 每个包的 payload 数据长度，单位为字节 | 参数接受 `0～255`；当前完整装载仅支持 `0～63`，常规测试请用 `1～63` | `20`，即 `0x14` |
| `--pkt_type` | 设置 payload 的数据内容 | 全 1、全 0、`0x55`、`0xAA`、`0x0F`、`0xF0`、`random`，具体写法见下表 | `0xff`，全 1 |
| `--tx_cnt` | 设置发包循环次数 | `1～65534`：指定次数；`65535` / `0xFFFF`：持续发送 | `65535`，持续发送 |
| `--tx_interval` | 两次发包调用之间的等待时间，单位为毫秒 | 有限的非负数；`0` 有特殊行为，见注意事项 | `20` ms |
| `--dev_index` | 选择 CH347 设备索引 | 参数接受 `0～4294967295`；须对应实际连接的设备 | `0` |
| `--dll` | 指定 CH347 动态库路径 | 有效的 `CH347DLLA64.dll` 文件路径；含空格时加双引号 | 脚本同目录的 `CH347DLLA64.dll` |
| `--tx_timeout` | 预留的单包发送完成等待超时，单位为毫秒 | 有限的正数；**当前未生效** | `1000` ms |
| `--verbose` | 打印寄存器读写及 FIFO 数据，便于排查 | 开关，无需参数值 | 关闭 |
| `--dry-run` | 模拟操作，不连接硬件、不实际发射 | 开关，无需参数值 | 关闭 |
| `-h` / `--help` | 显示帮助后退出 | 开关，无需参数值 | 不显示帮助，正常执行测试 |

CW 模式使用信道、功率及CH347连接、CS配置、日志相关参数；`--spi_mode`仅在packet TX/RX初始化时写入寄存器，CW不使用；包长、数据类型、包数、包间隔和超时不控制 CW 发射。即使设置 `--tx_cnt 1`，CW 仍持续运行，直到按 `Ctrl+C`。仅对当前收发方向使用的专用参数执行数值及范围检查。

### 数据类型写法

| 测试内容 | `--pkt_type` 推荐值 | 也支持的写法 |
|---|---|---|
| 每字节全 1 | `0xff` | `all1`、`ones`、`255` |
| 每字节全 0 | `0x00` | `all0`、`zeros`、`0` |
| 每字节为 0x55 | `0x55` | `055`、`85` |
| 每字节为 0xAA | `0xaa` | `170` |
| 每字节为 0x0F | `0x0f` | `15` |
| 每字节为 0xF0 | `0xf0` | `240` |
| 随机数据，每包重新生成 | `random` | 无 |

十六进制推荐始终写 `0x` 前缀。例如，`55` 表示十进制 55，不是 `0x55`，脚本会报错；`055` 是单独支持的别名。

### 功率档位

以下是脚本功率表中的参考值，测试结果以仪器实测为准。

| 档位 | 参考功率（dBm） | 档位 | 参考功率（dBm） | 档位 | 参考功率（dBm） |
|---|---|---|---|---|---|
| 1 | 8 | 6 | -1.7 | 11 | -7.8 |
| 2 | 3 | 7 | -2.7 | 12 | -9 |
| 3 | 1.7 | 8 | -3.8 | 13 | -10 |
| 4 | 0.3 | 9 | -5.2 | 14 | -18 |
| 5 | -0.8 | 10 | -6.8 | 15 | -27 |

## 4. 常用测试命令

### 先做离线检查

模拟 2 包，每包 32 字节的 `0x55`，同时打印操作日志：

```powershell
python wl160x_test_mode.py --dry-run --tx_test_mode 1 --len 0x20 --pkt_type 0x55 --tx_cnt 2 --verbose
```

此命令不发射。离线模式不执行真实等待；若包数为默认的 `0xFFFF`，只模拟 1 包。RX 离线模式输出一次零计数后退出。离线通过不能证明真实射频发送成功。

### 发送指定数量的数据包

在 2430 MHz、功率 1 档下，执行 100 次发包，每包 32 字节的 `0x55`，两次调用之间等待 20 ms：

```powershell
python wl160x_test_mode.py --tx_test_mode 1 --chan 30 --tx_pwr 1 --len 0x20 --pkt_type 0x55 --tx_cnt 100 --tx_interval 20
```

### 持续发送随机数据

在 2440 MHz、功率 4 档下，持续发送 32 字节随机数据，按 `Ctrl+C` 停止：

```powershell
python wl160x_test_mode.py --tx_test_mode 1 --chan 40 --tx_pwr 4 --len 32 --pkt_type random --tx_cnt 0xffff --tx_interval 20
```

### 测试 CW 单载波

在 2440 MHz、功率 1 档下开启单载波，按 `Ctrl+C` 停止：

```powershell
python wl160x_test_mode.py --tx_test_mode 0 --chan 40 --tx_pwr 1
```

CW 频点示例：2402 MHz 用 `--chan 2`；2480 MHz 用 `--chan 80`。

### 打印并保存日志

```powershell
python wl160x_test_mode.py --tx_test_mode 1 --len 32 --pkt_type 0x55 --tx_cnt 10 --verbose > tx_test.log 2>&1
```

日志保存在当前目录的 `tx_test.log`；使用此命令会覆盖同名日志。

### 指定设备和 DLL

```powershell
python wl160x_test_mode.py --dev_index 0 --dll "C:\CMS\TEMP_TEST\CH347_spi_read_write_reg\CH347DLLA64.dll" --tx_cnt 10
```

### 使用全部默认参数

```powershell
python wl160x_test_mode.py
```

执行数据包模式：2440 MHz、功率 1 档、20 字节全 1、持续发送、等待间隔 20 ms。按 `Ctrl+C` 停止。

## 5. 日志怎么看

- `write=12 0C`：向寄存器 `0x12` 写入 `0x0C`。寄存器日志里的地址和数据均为十六进制。
- `read=04 C2`：从寄存器 `0x04` 读回 `0xC2`。
- `write_fifo=64 AA AA ... (32 bytes)`：一次 FIFO 事务，发送一次地址 `0x64`，随后连续发送 32 个数据字节。
- 设置 `--pkt_type 0x55` 时，FIFO 日志为 `AA` 是当前脚本的正常行为：payload 写入前做了逐字节位序反转。`0x0F` 与 `0xF0` 也会互换。
- `发送完成: N 包`：当前版本只表示已执行 N 次发包调用，**不代表已确认空口成功发送 N 包**。须结合仪器或接收端确认。
- TX不加 `--verbose` 时，只打印第1包、每100包及指定包数的最后一包计数；RX仍逐包打印CRC正确的payload和时间戳。
- `--verbose` 不打印高频轮询的 `reg0x61` 回读；FIFO读取日志为原始线序数据，`Rx data`为逐字节反转后的payload。single模式的 `read_fifo=E4 ...` 汇总整包数据，并不表示只有一次CS事务。

## 6. 当前版本注意事项

1. **常规包长请使用 1～63 字节。** 虽然参数接受到 255，但当前长包补充 FIFO 的代码已注释。超过 63 字节时只装载首批 63 字节，其中包含 1 字节长度和 62 字节 payload，不能据此完成完整长包测试。`0` 字节可以通过参数检查，但零长度包的射频行为需另行验证。
2. **当前没有轮询发送完成标志。** 完成标志轮询和超时判断已注释，所以 `--tx_timeout` 暂不起作用，也不会因为未检测到 TX 完成而报超时。发包计数不等同于射频成功包数。
3. **等待间隔不等于精确的空口包间隔。** SPI 传输、日志打印和 Windows 调度也会耗时。`--tx_interval 0` 时，脚本仍调用 `sleep(0.0001)` 等待 100 微秒，实际等待可能更长，不是完全无间隔发送。
4. **CW 信道 0、1 不可用。** 命令行初步校验允许 0～80，但 CW 初始化进一步限制为 2～80；CW 使用 0、1 会报错退出。
5. **详细日志会影响发送速度。** 排查问题时加 `--verbose`；比较发送节奏时可关闭。当前脚本修改了硬件配置，完成或中断测试后会尝试复位 WL1606。

## 7. 常见问题

| 现象 | 处理方法 |
|---|---|
| 提示需要 Windows 64 位 Python | 使用 Windows 下的 64 位 Python，不要使用 32 位解释器 |
| DLL 加载失败 | 检查 DLL 是否在脚本目录，或通过 `--dll` 指定正确路径，并确认 CH347 驱动已安装 |
| 无法打开 CH347 | 检查 USB 连接、驱动和设备索引，关闭可能占用设备的其他程序 |
| 初始化回读失败 | 记录报错寄存器，检查供电、共地、SPI 接线及波形 |
| 参数越界或数据类型错误 | 对照参数表填写；十六进制使用 `0x` 前缀 |
| 打印“发送完成”但接收端无数据 | 当前没有完成标志确认；核对频点、功率、接收配置和仪器结果，必要时开启 `--verbose` 保存日志 |
| 超过 63 字节的数据不完整 | 当前长包补充未启用，请先使用 1～63 字节进行测试 |

记录测试结果时，建议同时记录完整命令、脚本版本或修改时间、连接设备、仪器设置和测量结果，便于复测。


## 8. 当前SPI配置与时序限制

| 项目 | 当前实现 |
|---|---|
| SPI模式、位序、时钟 | Mode 1（CPOL=0、CPHA=1）、MSB first、468.75 kHz（mClock=7） |
| 配置结构体 | SPI_CONFIG使用1字节对齐；Windows大小20字节；间隔字段偏移3、片选字段偏移6 |
| 字节间隔 | mSPIWriteReadInterval固定为0，无对应命令行参数 |
| 片选 | mChipSelect=0x80、mIsAutoDeactiveCS=1；正常事务仅调用一次WriteRead，参数0x80；失败时尝试ChangeCS(1)释放 |
| CS建立/保持延时 | 分别传入mActiveDelayNum和mDelayNum，默认0；当前实测改变参数未观察到影响，仅保留实验入口 |
| 读寄存器 | 一次CS内发送地址或0x80，再发送dummy FF，从返回缓冲取第二字节 |
| 读FIFO | 默认重复len次E4 FF，每字节独立CS；burst为一次E4后跟len个FF，单次CS |
| 写FIFO | 单次CS内发送64及整个数据块，payload在上层反转位序 |

`--spi_mode 3-wire`仅在packet TX/RX初始化中写WL1606的 `reg0x5E=0x80`；`4-wire`写0x00。CH347仍调用四线 `CH347SPI_WriteRead`，从MOSI输出、从MISO输入。dummy FF不等于释放MOSI；三线SDIO连接需要由接线或接口电路正确处理方向，脚本没有单独切换CH347引脚输入/输出的动作。

实板反馈：去掉正常传输前后的独立ChangeCS调用后，相关耗时减少；但在当前设备和DLL组合下，将字节间隔改为1且CS建立/保持均为0时，CSN仍保持高，改回0恢复。内部原因尚未确认，不能将非零间隔描述为当前已验证可用的功能。此前增加的字节间隔命令行参数与手动片选绕行方案已撤销，当前代码不包含它们。

CS建立/保持参数不控制地址与数据之间的间隔，也不控制两次独立事务之间的CS高电平时间。当前时序不能保证与MCU的2.98 µs建立、1.25 µs保持、20.76 µs高电平逐项相同；USB、DLL和Windows调度均影响事务间隔。使用Python sleep只能增加等待，无法缩短已有间隔或保证微秒精度。

参考MCU抓线的命令时钟约250 kHz，接收函数还有额外收尾时钟动作。当前代码对齐的是FIFO逐字节事务结构，没有逐边沿复制MCU波形。把多个E4 FF拼成一次WriteRead会保持一次长片选；也不能未经协议确认就把E4和数据拆成两次片选。

常用对比命令：

```powershell
# 当前默认：WL1606三线配置，逐字节读取FIFO
python wl160x_test_mode.py --mode 1 --pkt_type 0x55 --len 10 --chan 40 --out_rx_result_time 1000

# 切换WL1606端四线配置，需匹配实际接线
python wl160x_test_mode.py --mode 1 --pkt_type 0x55 --len 10 --spi_mode 4-wire

# 连续读FIFO，用于与逐字节方式对比
python wl160x_test_mode.py --mode 1 --pkt_type 0x55 --len 10 --fifo-read-style burst

# CS延时实验；当前实测未观察到参数影响
python wl160x_test_mode.py --mode 1 --pkt_type 0x55 --len 10 --spi-cs-setup-us 3 --spi-cs-hold-us 1
```
