# CoDrone EDU 控制器通訊協定規格書 (PROTOCOL.md)

本文件整理自官方 Python 套件 `codrone-edu` (v2.11) 原始碼，詳述實體控制器（Controller）與無人機（Drone）之間的序列埠通訊協定，供 Web Serial API 及 Python 實機連線控制使用。

---

## 1. 序列埠連線參數 (Serial Port Parameters)

* **鮑率 (Baud Rate)**: `57600`
* **資料位元 (Data Bits)**: `8`
* **停止位元 (Stop Bits)**: `1`
* **同位檢查 (Parity)**: `None`
* **流量控制 (Flow Control)**: `None`
* **控制器 USB 識別碼 (USB VID)**: `0x0483` (十進制 `1155`，STMicroelectronics Virtual COM Port)
* **硬體控制信號 (DTR / RTS)**:
  * `pyserial` 開啟連接埠時預設會拉高 DTR/RTS。
  * 在 Web Serial API 下，建議連線後執行 `await port.setSignals({ dataTerminalReady: true, requestToSend: true })` 確保虛擬 COM 埠順暢收發。

---

## 2. 封包格式 (Packet Structure)

所有多位元組欄位皆為 **Little-Endian**（低位元組在前）。

```
+------+------+----------+--------+------+------+------------------+------------+
| 0x0A | 0x55 | dataType | length | from |  to  | data[0...len-1]  | CRC16 (LE) |
+------+------+----------+--------+------+------+------------------+------------+
  起始碼(2B)       標頭 Header (4 Bytes)           資料 Payload(0~128B)    2 Bytes
```

* **總封包長度**: `2 (Start) + 4 (Header) + length + 2 (CRC16) = length + 8` 位元組。
* **CRC16 計算範圍**: **僅涵蓋標頭 (4B) 與 資料負載 (length Bytes)**，不包含起始碼 `0x0A, 0x55`。

### 標頭結構 (Header: 4 Bytes, `<BBBB`)
1. **dataType** (1 Byte, uint8): 資料類型代碼。
2. **length** (1 Byte, uint8): 資料負載長度 `len` (0 ~ 128)。
3. **from** (1 Byte, uint8): 發送端裝置代碼（主控電腦 PC 為 `0x70` Base）。
4. **to** (1 Byte, uint8): 接收端裝置代碼（無人機為 `0x10` Drone，控制器為 `0x20` Controller）。

---

## 3. 校驗演算法 (CRC-16-CCITT / XMODEM)

* **多項式 (Polynomial)**: `0x1021` (`x^16 + x^12 + x^5 + 1`)
* **初始值 (Initial Value)**: `0x0000`
* **位元反轉 / 輸出異或**: 無反轉、不 XOR。
* **驗證向量**: `CRC16("123456789") == 0x31C3`。

### JavaScript 實作
```javascript
function crc16(bytes, crc = 0) {
  for (const b of bytes) {
    crc ^= b << 8;
    for (let i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
  }
  return crc;
}
```

---

## 4. 常用常數代碼表 (Enums)

### 裝置類型 (DeviceType)
* `Drone`: `0x10` (無人機本體)
* `Controller`: `0x20` (實體遙控器)
* `Base`: `0x70` (電腦端 / Web Serial 主控台)

### 資料類型 (DataType)
* `Ping`: `0x01`
* `Ack`: `0x02`
* `Request`: `0x04` (向裝置請求特定資料)
* `Information`: `0x07` (硬體型號與韌體版本)
* `Control`: `0x10` (無人機即時姿態與推力控制)
* `Command`: `0x11` (動作指令)
* `RawMotion`: `0x30` (IMU 原始感測資料)
* `State`: `0x40` (系統狀態與電池資訊)
* `Attitude`: `0x41` (姿態角)
* `Position`: `0x42` (Optical Flow 計算之相對位置)
* `Altitude`: `0x43` (氣壓計與測距高度)
* `Motion`: `0x44` (處理後 IMU：加速度、角速度、姿態角)
* `Range`: `0x45` (紅外線距離感測，含底部測距)
* `Joystick`: `0x71` (遙控器實體搖桿輸入)

### 指令類型 (CommandType)
* `Stop`: `0x01` (緊急停止所有馬達)
* `ModeControlFlight`: `0x02` (設定飛行控制模式，如 Position `0x11`)
* `Headless`: `0x03` (無頭模式設定)
* `ControlSpeed`: `0x04` (設定飛行速度檔位 1~3)
* `FlightEvent`: `0x07` (飛行事件：起飛、降落等)
* `ModeController`: `0x0A` (控制器操作模式設定)

### 飛行事件 (FlightEvent)
* `Stop`: `0x10`
* `TakeOff`: `0x11` (起飛)
* `Landing`: `0x12` (降落)

### 飛行狀態 (ModeFlight，位於 State 回傳中)
* `Ready`: `0x10` (待命)
* `Start`: `0x11`
* `TakeOff`: `0x12` (起飛中)
* `Flight`: `0x13` (飛行中)
* `Landing`: `0x14` (降落中)
* `Stop`: `0x20` (停止)
* `Accident`: `0x30` (撞擊 / 意外)
* `Error`: `0x31` (錯誤)

---

## 5. 核心控制與通訊流程

### 5.1 連線初始化與握手
1. 開啟連接埠 (57600 8N1)。
2. 發送控制器切換 Link 模式：
   * `0A 55 11 02 70 20 0A 80 57 A1`
3. 輪詢 Request(State)，直到 `modeFlight == 0x10 (Ready)` 表示無人機與控制器已配對並連線。

### 5.2 飛行前置設定 (Prepare Sequence)
官方函式庫在起飛前均會依序套用標準安全設定：
1. `ControlSpeed = 2`: `0A 55 11 02 70 10 04 02 [CRC]`
2. `ModeControlFlight = Position (0x11)`: `0A 55 11 02 70 10 02 11 [CRC]`
3. `Headless = Normal (0x02)`: `0A 55 11 02 70 10 03 02 [CRC]`
4. 傳送懸停控制量 `Control(0, 0, 0, 0)`。

### 5.3 起飛程序 (TakeOff)
* 發送指令：`0A 55 11 02 70 10 07 11 36 81`
* 握手邏輯：持續以約 100ms 間隔重送，直到 State 回報 `modeFlight` 轉變為 `0x12 (TakeOff)` 或 `0x13 (Flight)`（最多等 4 秒），確認起飛後平穩等待 3.5~4 秒完成爬升。

### 5.4 降落程序 (Landing)
* 發送指令：`0A 55 11 02 70 10 07 12 55 B1`
* 握手邏輯：重送直到 State 回報 `0x14 (Landing)` 或 `0x10 (Ready)`，確認後等待 3~4 秒完成接地。

### 5.5 緊急停止 (Emergency Stop)
立即切斷馬達動力：
* 發送指令：`0A 55 11 02 70 10 01 00 80 29`
* 建議：連續發送 3 次確保無線通訊可靠抵達。

### 5.6 即時飛行姿態控制 (ControlQuad8)
* Header: `[0x10, 0x04, 0x70, 0x10]`
* Payload (4 Bytes signed int8, -100 ~ 100):
  * `roll`: 向左負、向右正
  * `pitch`: 向後負、向前正
  * `yaw`: 逆時針轉負、順時針轉正
  * `throttle`: 下降負、上升正
* **懸停 (Hover: 0, 0, 0, 0)**:
  `0A 55 10 04 70 10 00 00 00 00 B4 BD`

---

## 6. 感測數據查詢與解碼

### 6.1 查詢請求封包 (Request)
* 查詢狀態/電池 (State `0x40`): `0A 55 04 01 70 10 40 0D 6C`
* 查詢距離/高度 (Range `0x45`): `0A 55 04 01 70 10 45 A8 3C`
* 查詢 IMU 姿態 (Motion `0x44`): `0A 55 04 01 70 10 44 89 2C`

### 6.2 封包解碼規範
* **State (`0x40`, 8 Bytes)**:
  `<BBBBBBBB`: `modeSystem`, `modeFlight`, `modeControlFlight`, `modeMovement`, `headless`, `controlSpeed`, `sensorOrientation`, **`battery` (Offset 7, 0~100%)**。
* **Range (`0x45`, 12 Bytes)**:
  `<hhhhhh`: 6 個 16-bit 整數（left, front, right, rear, top, **`bottom`**），單位 **mm**。
  `高度 (cm) = bottom / 10`。
* **Motion (`0x44`, 18 Bytes)**:
  `<hhhhhhhhh`:
  - `accelX, accelY, accelZ`: 加速度（單位 0.1 m/s²）
  - `gyroRoll, gyroPitch, gyroYaw`: 角速度（單位 °/s）
  - **`roll, pitch, yaw` (Offset 12, 14, 16)**: 姿態角度（單位 °）

---

## 7. 安全限制 (Safety Constraints)

1. **嚴格禁止韌體與刷機指令**：
   嚴格排除 `Update (0x08)`、`UpdateLocation (0x09)`、`Encrypt (0x0A)`、`Administrator (0x0E)`。
2. **緊急中斷機制**：
   介面隨時提供最高優先級之「緊急停止」按鈕與鍵盤 `Esc` 快捷鍵。
3. **實體操作確認**：
   任何實體起飛、移動或執行自訂腳本前，均須經過使用者二次確認。
