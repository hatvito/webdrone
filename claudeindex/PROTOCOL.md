# CoDrone EDU 控制器通訊協定（Web Serial 移植用）

來源：`codrone-edu` 2.11（PyPI），檔案 `drone.py`、`protocol.py`、`system.py`、`receiver.py`、`crc.py`。
所有數值皆直接取自原始碼；**尚未用實機驗證**（標示「待實機確認」之處除外）。

---

## 1. 序列埠

| 項目 | 值 | 出處 |
|---|---|---|
| Baud rate | **57600** | `drone.py` `_open_desktop` |
| 資料位元 / 同位 / 停止位 | 8 / None / 1（pyserial 預設） | 同上，未另行設定 |
| 流量控制 | 無（pyserial 預設） | 同上 |
| USB VID | **1155（0x0483）**，用來自動找控制器 | 同上 |
| 讀取 | 持續逐批讀取，餵給狀態機解析 | `_receiving` |

Web Serial：`requestPort({ filters: [{ usbVendorId: 0x0483 }] })`，`open({ baudRate: 57600 })`。
**待實機確認**：pyserial 開埠時預設會拉高 DTR/RTS；Web Serial 若連不上，可嘗試 `port.setSignals({ dataTerminalReady: true, requestToSend: true })`。

---

## 2. 封包格式

所有多位元組欄位為 **little-endian**。

```
+------+------+----------+--------+------+------+-----------+-----------+
| 0x0A | 0x55 | dataType | length | from | to   | data[len] | CRC16 (LE)|
+------+------+----------+--------+------+------+-----------+-----------+
  起始碼(2)      標頭(4 bytes)                    資料(0~128)   2 bytes
```

- `length` = data 的長度，上限 128。
- 電腦端當作 `from = 0x70 (Base)`；目標 `to = 0x10 (Drone)`。控制器負責中繼到無人機。
- 接收端回傳的封包 `from = 0x10 (Drone)`、`to = 0x70 (Base)`。

### 校驗：CRC-16/XMODEM

- 多項式 0x1021、初始值 **0**、不反轉、不 XOR。
- 計算範圍：**標頭 4 bytes + data**（不含 `0x0A 0x55`）。
- 附加在封包尾端，低位元組在前。
- 驗證：`CRC16("123456789") = 0x31C3`。

```js
function crc16(bytes, crc = 0) {
  for (const b of bytes) {
    crc ^= b << 8;
    for (let i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
  }
  return crc;
}
```

### 接收狀態機（`receiver.py`）

1. 等 `0x0A`，再等 `0x55`，否則重置。
2. 讀標頭 4 bytes（dataType、length、from、to），`length > 128` 視為錯誤。
3. 讀 `length` bytes 資料。
4. 讀 2 bytes CRC，與自行計算的比對；不符就丟棄。
5. 開始接收後 **600 ms** 內未收完視為逾時。

---

## 3. 代碼表（只列本專案會用到的）

### DeviceType

| 名稱 | 值 |
|---|---|
| Drone | 0x10 |
| Controller | 0x20 |
| Base（電腦端） | 0x70 |

### DataType

| 名稱 | 值 | 用途 |
|---|---|---|
| Ping | 0x01 | 通訊確認（8 bytes 時間戳） |
| Ack | 0x02 | 資料收到確認 |
| Request | 0x04 | 要求對方回傳指定 DataType |
| Information | 0x07 | 型號、韌體版本 |
| Control | 0x10 | 搖桿式控制或位置控制 |
| Command | 0x11 | 指令（起飛、降落、停止、設定） |
| State | 0x40 | 飛行狀態與電池 |
| Altitude | 0x43 | 氣壓高度（本專案不用） |
| Range | 0x45 | 距離感測器（含底部高度） |

### CommandType

| 名稱 | 值 |
|---|---|
| Stop | 0x01 |
| ModeControlFlight | 0x02 |
| Headless | 0x03 |
| ControlSpeed | 0x04 |
| FlightEvent | 0x07 |

### FlightEvent（`Command.option`，搭配 `CommandType.FlightEvent`）

| 名稱 | 值 |
|---|---|
| Stop | 0x10 |
| TakeOff | 0x11 |
| Landing | 0x12 |

### ModeFlight（State 內的飛行狀態）

Ready 0x10 · Start 0x11 · **TakeOff 0x12** · Flight 0x13 · **Landing 0x14** · Flip 0x15 · Reverse 0x16 · Stop 0x20 · Accident 0x30 · Error 0x31

### ModeControlFlight

Attitude 0x10 · **Position 0x11**（函式庫連線後預設設成這個） · Manual 0x12 · Rate 0x13

---

## 4. 各 payload 結構

### Request（1 byte）
`<B dataType>`

### Command（2 bytes）
`<BB commandType, option>`

### Control — 搖桿式（`ControlQuad8`，4 bytes）
`<bbbb roll, pitch, yaw, throttle>`，各為 **int8，範圍 −100 ~ 100**。
`(0,0,0,0)` = 懸停。持續送才會維持動作（函式庫在 `duration` 期間約每 3 ms 送一次）。

### Control — 位置式（`ControlPosition`，20 bytes）
`<ffffhh positionX, positionY, positionZ, velocity, heading, rotationalVelocity>`
X 前(+)後(−)、Y 左(+)右(−)、Z 上(+)下(−)，單位公尺，±10.0；velocity 0.5~2.0 m/s；heading −360~360 度；rotationalVelocity 10~360 度/秒。
`move_forward()` 等高階指令即用此格式。

### State 回應（8 bytes）
`<BBBBBBBB>` 依序：
`modeSystem, modeFlight, modeControlFlight, modeMovement, headless, controlSpeed, sensorOrientation, battery`
**battery = 第 8 個 byte（offset 7），0~100（%）**。

### Range 回應（12 bytes）
`<hhhhhh left, front, right, rear, top, bottom>`，int16，**單位 mm**。
**高度 = `bottom`（offset 10）**，函式庫 `get_height()` 即 `bottom / 10` 換成 cm。
原始碼註解：底部感測器未飛行時不更新，預設顯示 −1000。

### IMU 相關（唯讀，皆以 Request 取得）

| DataType | 值 | Payload | 說明 |
|---|---|---|---|
| RawMotion | 0x30 | `<hhhhhh` 12 bytes：accelX/Y/Z, gyroRoll/Pitch/Yaw | 感測器原始 int16，**原始碼未標示單位與換算** |
| Attitude | 0x41 | `<hhh` 6 bytes：roll, pitch, yaw | 姿態角，度 |
| Motion | 0x44 | `<hhhhhhhhh` 18 bytes：accelX/Y/Z, gyroRoll/Pitch/Yaw, angleRoll/Pitch/Yaw | accel 為 m/s²×10（±1568）、gyro 為 °/s（±2000）、angle 為度（±180） |

函式庫的 `get_accel_*()` / `get_angular_speed_*()` / `get_angle_*()` 讀的是 Motion，不是 RawMotion。
Request 測試向量：RawMotion `0A 55 04 01 70 10 30 9A 12`、Attitude `0A 55 04 01 70 10 41 2C 7C`、Motion `0A 55 04 01 70 10 44 89 2C`。
**待實機確認**：RawMotion 的單位／比例，以及各軸正負方向。

---

## 5. 功能對照

| 功能 | 送出內容（from=0x70, to=0x10） | 說明 |
|---|---|---|
| 讀電池 | Request(State) → 解析回應 offset 7 | `get_battery()` |
| 讀高度 | Request(Range) → 解析回應 bottom | `get_height()` |
| 起飛 | Command(FlightEvent, 0x11) | `takeoff()` 先送 3 次懸停控制，再送起飛；若 State.modeFlight 尚未變成 TakeOff 就重送（最多 4 秒），之後等 4 秒 |
| 降落 | Command(FlightEvent, 0x12) | `land()` 先送懸停控制，再送降落並重送到 modeFlight 變成 Landing，之後等 4 秒 |
| 緊急停止 | Command(Stop, 0) | `emergency_stop()` 先送懸停控制，再送 Stop；馬達立即停轉 |
| 懸停 | Control(0,0,0,0) | `hover(duration)` |
| 移動/轉向 | Control(roll,pitch,yaw,throttle) | `set_*()` + `move()`；轉向用 yaw |
| 位置移動 | ControlPosition | `move_forward()` 等 |

---

## 6. 函式庫連線流程（`open()`）與本專案的取捨

函式庫連線時會依序：

1. 以 57600 開埠。
2. 反覆 Request(State)，最多約 10 次，直到 `modeFlight == Ready`。
3. Request(Information) 取得型號。
4. **送出設定指令**：`ControlSpeed=2`、`ModeControlFlight=Position`、`Headless=Normal`，再送 `Control(0,0,0,0)`。

本專案的「只讀」階段（步驟 2 腳本與網頁連線）**只做 1~3**，不送任何設定或控制指令。
第 4 項的設定指令等到你確認、準備首飛時才由網頁送出。

---

## 7. 離線測試向量（由函式庫產生，供 JS 逐位元組比對）

| 封包 | Hex |
|---|---|
| Request State | `0A 55 04 01 70 10 40 0D 6C` |
| Request Range | `0A 55 04 01 70 10 45 A8 3C` |
| Request Information | `0A 55 04 01 70 10 07 2E 54` |
| Command TakeOff | `0A 55 11 02 70 10 07 11 36 81` |
| Command Landing | `0A 55 11 02 70 10 07 12 55 B1` |
| Command Stop（緊急停止） | `0A 55 11 02 70 10 01 00 80 29` |
| Control 懸停 (0,0,0,0) | `0A 55 10 04 70 10 00 00 00 00 B4 BD` |
| Control (10,20,−30,40) | `0A 55 10 04 70 10 0A 14 E2 28 06 99` |
| 範例 State 回應（電池 87%） | `0A 55 40 08 10 70 12 10 11 01 02 02 01 57 B4 AA` |
| 範例 Range 回應（front 1000, bottom 123） | `0A 55 45 0C 10 70 00 00 E8 03 00 00 00 00 00 00 7B 00 5D A3` |

---

## 8. 刻意排除（依你的限制）

原始碼中與韌體更新、加密、註冊、管理員權限相關的 DataType（`Update 0x08`、`UpdateLocation 0x09`、`Encrypt 0x0A`、`Registration 0x0D`、`Administrator 0x0E`）以及 `DeviceType.Updater`，**不會**出現在任何腳本或網頁中。
同時不使用：`SetDefault`、`ClearBias`、`ClearTrim`、`Trim`、馬達單獨控制（`Motor` / `MotorSingle`）、`Pairing`、`Link` 等會改動裝置設定的指令。

## 9. 尚未驗證、需實機確認的事項

- DTR/RTS 在 Web Serial 是否需要手動拉高（見第 1 節）。
- 控制器回應 Request(State) 的實際延遲（函式庫等 10 ms 後讀取快取值；網頁版改為等待回應事件）。
- `Range.bottom` 在落地、未起飛狀態的實際讀值。
- 控制器是否會主動推送 State/Ack（函式庫的處理器都能接收，但本文件只確認「請求→回應」路徑）。
