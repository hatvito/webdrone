# CoDrone EDU Web Serial 控制介面

這是一個基於 **Web Serial API** 的網頁版 CoDrone EDU 無人機真實連線控制系統（類似 `codrone.robolink.com/edu/python/`）。純真實連線，無任何模擬器或假資料。

---

## 專案特色

1. **純 Web Serial 原生連線**：
   * 無需安裝桌面客戶端或中繼驅動，直接在支援 Web Serial 的瀏覽器（Chrome / Edge）連線 CoDrone EDU 實體控制器（Baud Rate: 57600 8N1）。
2. **完整通訊協定移植**：
   * 詳實整理官方 `codrone-edu` Python 套件底層通訊協定（見 [PROTOCOL.md](PROTOCOL.md)）。
   * 實作 Little-Endian 標頭、CRC-16-CCITT/XMODEM 查表校驗與流式滑動視窗解析器。
3. **安全機制與緊急停止**：
   * 雙重緊急停止：網頁鮮紅急停按鈕 + 鍵盤 **`Esc`** 快捷鍵，立即切斷馬達動力。
   * 實機安全二次確認對話框，起飛或位移動作前強制確認環境安全。
4. **即時遙測與姿態監控 (Telemetry & IMU)**：
   * 即時顯示電池百分比、底部測距高度（公分）、氣壓高度、飛行狀態與通訊採樣頻率。
   * 即時 IMU 姿態角（Roll / Pitch / Yaw）動態視覺化。
5. **Python 風格指令腳本編輯器**：
   * 支援在網頁上編寫 Python 風格指令控制無人機：
     ```python
     # 第一次飛行測試安全腳本
     drone.takeoff()
     drone.hover(2)
     drone.land()
     ```
   * 支援 `drone.takeoff()`、`drone.hover(sec)`、`drone.land()`、`drone.move(sec)`、`drone.set_pitch(val)`、`drone.emergency_stop()` 等。
6. **獨立單檔架構 (Single File)**：
   * [index.html](index.html) 採完全自包含設計，無外部依賴，可離線運作。

---

## 專案結構

* **`index.html`**：Web Serial 單檔網頁控制介面。
* **`PROTOCOL.md`**：CoDrone EDU 控制器序列埠通訊協定規格書。
* **`check_battery.py`**：最小 Python 序列埠測試腳本（僅連線與讀取電池電量，不做飛行動作）。
* **`claudeindex/`**：先前版本參考檔案（含 Three.js 數位雙生遙測概念）。

---

## 本地快速啟動

因瀏覽器 Web Serial 安全策略要求網頁必須在 `localhost` 或 `https://` 環境執行：

```bash
# 啟動本地伺服器
python -m http.server 8000

# 在 Chrome 或 Edge 瀏覽器開啟
http://localhost:8000
```
