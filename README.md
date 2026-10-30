# baseboard_test

## Current target

The project targets the STM32G474RCTX and uses STM32 HAL with FreeRTOS. USART2 (PA2/PA3, 115200 8-N-1) is connected to the DYP-A22 bus through an RS485 transceiver. The transceiver direction is handled externally, so firmware does not control a DE/RE pin.

## USART2/DYP implementation history

- Added DMA RX plus USART2 IDLE-frame handoff from the interrupt handler to the DYP module.
- Added Modbus RTU CRC16, DMA transmission, distance-read and register-write helpers in `Core/Src/DYP/DYPA22.c`.
- Added on-demand DYP configuration helpers for algorithm enable, angle/range levels, and slave ID changes.
- Added RX frame validation and sensor status storage for the two configured sensor IDs.
- Added an independent USB CDC result module; the FreeRTOS task sends one result line after each complete sensor polling round (`DYP,ID1=...mm,ID2=...mm`).
- Added periodic DYP service invocation from the default FreeRTOS task.

## USART2 read diagnostics

Each polling round now sends the original result line followed by one diagnostic
line per sensor, in the same USB CDC transfer. DYP only records structured data;
`Core/Src/CDC/usb_cdc_result.c` owns formatting and USB transmission. No new task
or timer is created, and the existing polling delay is preserved.

Example of a receive timeout:

```text
DYP,ID1=INVALID,ID2=INVALID
DYP_DIAG,ID=1,SEQ=1,ERR=3,DETAIL=TIMEOUT,TX_HAL=0,UART=0x00000000,MS=800,RX_LEN=0,PARTIAL=1,EX=0x00,HEX=NONE
```

- `SEQ`: global read transaction sequence, cached separately for each sensor.
- `ERR`: unchanged read return code: 0 success, 1 TX/parameter failure,
  2 rejected response, 3 response timeout.
- `DETAIL`: OK, PARAM, TX_FAIL, TIMEOUT, LENGTH, ADDRESS, FUNCTION, BYTE_COUNT,
  CRC, or EXCEPTION. It reports the first failed check, not all possible defects.
- `TX_HAL`: return status of the send helper: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT.
  The existing helper reports ERROR when the UART is not ready. HAL_OK only means
  DMA transmission was accepted, not that the sensor received the request.
- `UART`: HAL error bitmask recorded by the USART2 error callback and sampled at
  read completion: 0x01 parity, 0x02 noise, 0x04 framing, 0x08 overrun,
  0x10 DMA, 0x20 receiver timeout. Multiple bits can be set. This is not a Modbus
  error code; zero does not prove that wiring or the sensor is correct.
- `MS`: elapsed read time in milliseconds.
- `RX_LEN` / `HEX`: number of captured bytes and their raw hexadecimal values;
  NONE means no captured bytes. At most 128 bytes are retained.
- `PARTIAL=1`: timeout snapshot of the DMA buffer, not an IDLE-completed frame;
  it can be empty. A timeout can therefore have nonzero RX_LEN.
- `EX`: Modbus exception byte, populated only for a CRC-valid 0x83 response.

Typical checks: TX_FAIL means inspect UART/DMA state; TIMEOUT with no bytes means
inspect the bus, device address and baud rate; LENGTH with an 8-byte request in
HEX suggests local echo; CRC suggests corrupted or incorrectly framed data.
The first received frame is still used, so this diagnostic change does not fix
echoes, stale frames, UART recovery, or sensor special-distance handling.

Only distance reads are diagnosed here, not optional register writes. Records
are latest-per-sensor snapshots, not a historical queue. If USB is offline or
busy, that round is skipped without blocking or overwriting an in-flight TX
buffer; the next round reports its own results. Call the CDC sender from only
the existing polling task.

## CubeMX status

No CubeMX configuration change is required for this code-only update. The existing USART2, DMA1 channel 1/2, PA2/PA3, and USART2 interrupt settings are used as generated.

## CubeMX code preservation note

CubeMX may overwrite generated source files when code is regenerated. Keep
application/private additions in the matching `USER CODE BEGIN/END` sections,
or in dedicated user-owned source/header files. Do not add custom code in the
generated regions between those markers. Before and after CubeMX regeneration,
review the affected-file diff and verify all custom code remains; if a required
user-code insertion point is absent, report it before making dependent changes.

Compilation and hardware verification were intentionally not run, per the change-stage constraint.

## ZLAC8015D / FDCAN2（通信与标准 CiA402 接口）

新增 `Core/Inc/zlac8015d.h`、`Core/Src/zlac8015d.c`。当前版本完成通信基础层，
尚未完成左右轮速度闭环应用。所附 Version 1.02 用户手册确认支持 CiA301/CiA402，
默认 CAN 波特率 500 kbit/s，但没有对象字典、双轴映射、速度单位及完整使能流程。
不能据此假设两个轮子的节点号、对象偏移或转速换算。

### 当前运行行为

- 使用既有 FDCAN2：PB5 RX、PB6 TX、经典 CAN、500 kbit/s。
  170 MHz / [17 × (1 + 15 + 4)] = 500 kbit/s。沿用用户已修改的配置，
  无需 CubeMX 改动，不控制外接收发器 GPIO。
- 初始化全局标准数据帧接收、启动 FDCAN2、开启 FIFO0 接收/丢帧通知。
  `StdFiltersNbr=0` 可以使用全局接收规则，无需增设过滤器元素。
- 接收中断只入队；独立静态任务每 5 ms 处理有界接收队列与 500 ms SDO 超时。
  任务优先级高于 DYP 默认任务，避免其阻塞等待造成 CAN 服务饥饿；
  任务使用静态 1024 字节栈和 TCB，不增加动态堆分配。
- 上电不自动发送 CAN 指令。原周期设备类型探测已取消，改为通过 CDC 显式
  指定节点、对象及子索引；不自动发送 NMT 启动、控制字或速度命令。
- 支持 1～4 字节 expedited SDO 读写、响应节点/索引/子索引匹配、SDO Abort、
  超时、心跳/启动帧、EMCY、原始最近接收帧及 CAN 错误状态记录。
  不支持 segmented/block SDO、PDO 映射、SYNC 和厂家双轴控制。

### 标准控制接口与限制

所有 CANopen 收发接口只允许由 `StartMotorTask` 的调用链执行；异步发起一次
SDO 后，必须等待 `ZlacDiagnostic.result` 结束，确认成功再发下一条。
`HAL_OK` 只表示进入发送队列。CDC 命令服务已按单事务方式串行调用。

| 接口 | 标准对象 / 行为 |
| --- | --- |
| `Zlac_ReadObject` / `Zlac_WriteObject` | 按指定节点、索引、子索引读写 |
| `Zlac_SendNmt` | 单播 start / stop / pre-operational，不允许广播或复位 |
| `Zlac402_ReadStatusword` | `6041:00`，读取状态字 |
| `Zlac402_WriteControlword` | `6040:00`，写入控制字 |
| `Zlac402_SetMode` | `6060:00`，设置模式，需读取 `6061:00` 确认 |
| `Zlac402_SetTargetVelocityRaw` | `60FF:00`，写入有符号原始速度，**不是已确认的 RPM** |
| `Zlac402_ReadActualVelocity` | `606C:00`，读取原始实际速度 |

这些 CiA402 帮助函数针对标准逻辑轴，未被启动任务调用。需先用厂家 CANopen
协议手册或 EDS 核实该型号的左右轴映射、支持模式与速度比例，再接入运动控制。
控制字使能应按状态字逐步推进，不能把多条异步写入连续调用当作完整使能流程。
写入成功也不等于驱动状态转换完成。CiA402 NMT stop 也不等价于安全急停。

### CDC 诊断与通信验证

独立 CDC 模块在既有 DYP 数据后追加 `ZLAC,...` 行，复用同一次 USB 传输，
电机任务不发送 USB。输出频率仍由 DYP 轮询决定，不是实时运动反馈；USB 忙时
跳过该轮。诊断是最新快照，后续 SDO 会覆盖上一次结果；命令结果另有独立回执。

- `RESULT`：0 IDLE、1 WAITING、2 OK、3 TX_ERROR、4 TIMEOUT、5 SDO_ABORT、
  6 BAD_RESPONSE、7 BUS_OFF。`OBJ`/`VALUE` 是当前或最近一次 SDO 的对象/返回值。
- `ABORT` 是驱动返回的 32 位 SDO 中止码；`MS` 是事务处理耗时。
- `TX_HAL` 是 SDO 发送/初始化 HAL 状态；`HAL` 是累计 HAL 错误位。
  `BO` 为当前 bus-off，`LEC` 为硬件 LastErrorCode，`TEC`/`REC` 为错误计数。
  `LEC=7` 表示自上次读取以来无变化，不是新的总线错误。
- `RX` 为软件已处理标准帧数，`DROP` 包含队列丢帧计数和硬件丢帧事件数，
  后者不是精确丢帧总数。硬件 FIFO 仅 3 帧，软件队列可容纳 15 帧；大量无关
  总线流量时需另行增加针对本节点的硬件过滤。
- `HB=节点:状态` 为最近心跳/启动帧（0 启动、4 stopped、5 operational、
  127 pre-operational），无心跳不能单独判定掉线，设备可能未启用生产者心跳。
- `EMCY=节点:错误码:错误寄存器` 是最近紧急报文，保留至下一次 EMCY。
  `LAST`/`LEN`/`HEX` 是最近任意已处理标准帧，并非保证是当前 SDO 响应。

可用 USB-CAN 在 **500 kbit/s、标准数据帧、8 字节** 下独立验证节点 1：

```text
TX ID=601 DATA=40 00 10 00 00 00 00 00   # 读取设备类型 1000:00
RX ID=581 DATA=43 00 10 00 xx xx xx xx   # 4 字节小端返回值
TX ID=601 DATA=40 18 10 01 00 00 00 00   # 读取 1018:01 厂商 ID
TX ID=601 DATA=40 18 10 02 00 00 00 00   # 读取 1018:02 产品代码
```

不同节点 N 的请求 ID 为 `600+N`、响应 ID 为 `580+N`（十六进制基址），
必须先确认真实节点号。避免 USB-CAN 与 STM32 同时对同一节点发送 SDO。
收到合法响应只能证明 CANopen 通信；需要核对厂商/产品代码及铭牌确认型号。
超时且无 RX 时检查节点号、实际波特率、CANH/CANL、参考地与总线端接；
TX_HAL=0 不能证明物理发送已获 ACK。现有自动重传为关闭状态，未擅自改变。
当前不自动重试写入、不自动故障复位或 bus-off 恢复，避免意外重复动作。
SDO 无事务编号，超时后迟到的同对象响应无法严格区分；应用应留出隔离时间。

本次只做代码及文档修改和静态检查，未编译、未下载、未进行硬件验证。

## CDC 输入控制 FDCAN2

新增 `Core/Inc/cdc_can_command.h` 和 `Core/Src/cdc_can_command.c`：
`CDC_Receive_FS` 缓存命令行 → 电机任务解析 → CANopen SDO/NMT 编码 → FDCAN2。
CAN 接收仍由中断队列交给电机任务处理；结果通过独立 CDC 输出模块返回。
不需要新增 CubeMX 配置，生成文件中的接入代码均放在 USER CODE 区域。

用串口助手打开 USB CDC 虚拟串口，选择 **ASCII/文本发送**，每条命令末尾加
换行（LF、CR 或 CRLF 均可）。USB CDC 的串口波特率选项不改变 CAN 波特率；
FDCAN2 仍为 500 kbit/s。命令关键字大写，所有数字为十六进制，可带 `0x` 前缀。

```text
SDO R <节点> <索引> <子索引>
SDO W <节点> <索引> <子索引> <字节数1~4> <原始值>
NMT <节点> <命令01/02/80>
```

例如节点 1 的通信/状态查询：

```text
SDO R 01 1000 00
SDO R 01 1018 01
SDO R 01 6041 00
SDO R 01 6061 00
```

**逐条发送并等待各自 `CAN_REPLY`，不要把示例整段发送当作自动使能流程。**
例如 `SDO R 01 6041 00` 被编码为标准帧 ID `601`、
数据 `40 41 60 00 00 00 00 00`，等待 ID `581` 的匹配响应。

写入示例（需要先确认设备对象定义及状态）：

```text
SDO W 01 6060 00 1 03       # 6060:00=3，选择 profile velocity 模式
SDO W 01 6040 00 2 0006     # 6040:00=6，Shutdown 控制字
SDO W 01 60FF 00 4 00000000 # 60FF:00=0，目标原始速度为零
NMT 01 80                  # 请求 pre-operational
NMT 01 01                  # 请求 operational
```

上面的 `#` 注释仅用于文档说明，**发送时去掉注释**，否则语法检查会拒绝。
这些是独立对象访问示例，不构成完整运动流程，不会自动推进使能状态机。
子索引可以按已确认的双轴对象字典改为 `01`、`02`、`03`，例如
`SDO R 01 606C 03`；固件不猜测左右轮映射。带符号数按对应位宽的补码输入，
例如 32 位原始值 -100 写作 `FFFFFF9C`，不代表 -100 RPM。

回执示例（设备类型值仅为示意）：

```text
CAN_REPLY,CMD=1,RESULT=OK,ID=1,OBJ=1000:00,SIZE=4,VALUE=0x00020192,ABORT=0x00000000,TX_HAL=0,MS=5,RX_DROP=0
```

- `CMD` 是固件为每条取出的非空命令行分配的递增编号，不是 CANopen 事务号。
- `RESULT`：OK、SDO_ABORT、TIMEOUT、BUS_OFF、BAD_RESPONSE、SEND_FAILED、
  SYNTAX 或 NMT_QUEUED。SDO 的 OK 表示收到匹配成功响应，写入后仍需读状态
  确认实际模式/使能状态。NMT_QUEUED 仅表示入队，需心跳或状态查询确认。
- `VALUE`/`SIZE` 用于成功的 SDO 读取；写响应没有返回值。NMT 的 VALUE
  显示所请求的命令码。ABORT 是驱动器拒绝访问时的原始中止码。
- `RX_DROP` 是累计丢弃的输入行数：队列满、超过 79 字符或含非法字符的行
  整行丢弃，不执行截断内容。队列最多暂存四行；正常操作采用一问一答。
- USB 回调支持分包和一次多行。无换行的内容保留等待后续字节，不会执行。
  输入不支持退格编辑、中文字符、逗号分隔或行内注释。
- 命令回执独立于 DYP 数据生成，USB 忙时保留并重试输出，绝不重发 CAN 写入。
  单一回执邮箱未交给 USB 前暂停后续命令，USB 断开时队列也不会被自动清空；
  已入队命令可能在恢复连接后继续执行，主机应避免提前批量下发运动命令。
- 现有默认任务仍负责全部 CDC TX，每 20 ms 尝试输出回执；DYP 自身阻塞读取
  期间可能延迟一轮输出。电机任务的收发/超时服务仍每 5 ms 运行。

本次静态检查不代替编译或实机测试；按要求未进行编译验证。
