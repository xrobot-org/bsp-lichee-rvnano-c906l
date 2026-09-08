# SG2002 → SPI DMA → ESP32-C5 图传

数据路径是 SC035HGS → Linux H.264 编码 → 共享 DDR 邮箱 → C906 SPI2 DMA →
ESP32-C5 SPI slave DMA → Wi-Fi RTP → 电脑解码。USB RNDIS 用于管理板卡。

`User/main.cpp` 的发送端与 独立 `razver` 工作区中的 C5 固件配套：

- SPI mode 1，时钟 5.859375 MHz，C906 RX/TX 使用 DMA 通道 4/5。
- Linux 的 DW DMA 必须限制为通道 0–3；C906 驱动保留其他通道和 CPU 的路由。
- 每条记录最长 2048 字节，28 字节协议头；按实际载荷向上对齐到 64 字节。
- HELLO、POLL、元数据通常只发送 64 字节。原 RZ/OK 协议和 CRC-32 保持兼容。
- 当前四线连接没有 READY 信号，记录间等待三个 1 kHz tick，保证至少 2 ms
  从机处理／重装时间；每个图像单元结束时核对累计 ACK 序号。
- 编码建议先使用传感器原生的 640×480、30 fps、512 kbit/s。

已有 `razver-camera.service` 的板卡可使用 `/etc/default/razver-camera` 配置：

```sh
RAZVER_FPS=30
RAZVER_BITRATE=512
```

修改后重启该服务。C5 默认在启动后 15 秒开启 SPI；C906 启动等待 20 秒。
若更新任一端固件，先停止相机服务，更新并启动两端固件，再启动相机服务。
板上旧固件和设备树应保留备份。

## 无线接收验证

电脑连接 C5 的 `RZ-C5-FPV` 接入点，配置 IPv4 为 `192.168.4.2/24`。运行脚本
的操作系统必须能够绑定该地址；使用 Windows Wi-Fi 网卡时应运行 Windows Python。
安装 `av`、`opencv-python` 和 `numpy`，然后执行：

```sh
python verify_wifi_stream.py --seconds 90 --output validation
```

`verify_wifi_stream.py` 仅接收 `192.168.4.1:任意源端口` 发往 UDP 5600 的 RTP，
区分 H.264 和元数据，重组 FU-A，实际解码并测量到达帧率。输出包括：

- `report.json`：帧率、分辨率、源地址、RTP 丢包、解码错误及到达间隔。
- `stream.h264`：重组后的码流，可独立解码复核。
- `first-frame.png`、`preview.png`：真实接收画面。

同一时间只运行一个 UDP 5600 接收程序。相机运行中加入时，需等待下一组
SPS/PPS 和 IDR；冷启动测试可以先启动接收端，再重启相机服务。
报告分别列出启动期解码错误和首帧后的解码错误，同时保留总错误数。
帧到达间隔包含编码、SPI、Wi-Fi 和调度抖动，不能直接当作端到端拍摄延迟。

C5 每两秒输出 `records/accepted/bad/short/seq/au/bytes/udp/errors/retries/queue/full`
统计。比较测试窗口前后计数，避免把前次烧录或重启的历史错误计入当前测试。
C906 在共享调试区 `0x8ffff100` 中保留阶段和计数：字 5 为 RX DMA 完成数，
字 6 为传输记录数，字 7/8 为成功／失败图像单元，字 9 为 ACK 失败数，
字 10 为成功载荷字节数，字 11 为最大记录长度，字 12 为发送字节数，
字 13 为额外 ACK 轮询次数。
