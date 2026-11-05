#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CoDrone EDU 最小連線與讀取電池測試腳本 (pyserial)
注意：本腳本只做「連線 + 讀取電池」，不送任何飛行或移動指令。
"""

import sys
import time
from struct import pack, unpack
import serial
from serial.tools import list_ports

# CRC-16-CCITT 查表
CRC_TABLE = (
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
)

def calc_crc16(data: bytes, crc: int = 0) -> int:
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC_TABLE[idx]) & 0xFFFF
    return crc

def build_packet(data_type: int, length: int, from_dev: int, to_dev: int, payload: bytes) -> bytes:
    header = bytes([data_type, length, from_dev, to_dev])
    crc = calc_crc16(header, 0)
    crc = calc_crc16(payload, crc)
    return bytes([0x0A, 0x55]) + header + payload + pack('<H', crc)

# 常數定義
DEV_DRONE = 0x10
DEV_CONTROLLER = 0x20
DEV_BASE = 0x70

DATA_REQUEST = 0x04
DATA_COMMAND = 0x11
DATA_STATE = 0x40

CMD_MODE_CONTROLLER = 0x0A
OPTION_LINK_MODE = 0x80

MODE_FLIGHT_MAP = {
    0x00: "None",
    0x10: "Ready (待命)",
    0x11: "Start",
    0x12: "TakeOff (起飛中)",
    0x13: "Flight (飛行中)",
    0x14: "Landing (降落中)",
    0x20: "Stop (已停止)",
    0x30: "Accident (異常/撞擊)",
    0x31: "Error (錯誤)"
}

def find_controller_port():
    ports = list_ports.comports()
    cde_vid = 1155  # 0x0483 STMicroelectronics
    for p in ports:
        if p.vid == cde_vid:
            return p.device
    if ports:
        return ports[0].device
    return None

def parse_incoming_stream(ser, timeout_sec=5.0):
    start_time = time.time()
    rx_state = "START1"
    header = []
    payload = bytearray()
    expected_len = 0
    crc_bytes = []

    while time.time() - start_time < timeout_sec:
        if ser.in_waiting == 0:
            time.sleep(0.01)
            continue
        
        b = ser.read(1)[0]

        if rx_state == "START1":
            if b == 0x0A:
                rx_state = "START2"
        elif rx_state == "START2":
            if b == 0x55:
                rx_state = "HEADER"
                header = []
            else:
                rx_state = "START1"
        elif rx_state == "HEADER":
            header.append(b)
            if len(header) == 4:
                data_type, length, from_dev, to_dev = header
                expected_len = length
                payload.clear()
                if expected_len > 128:
                    rx_state = "START1"
                elif expected_len == 0:
                    rx_state = "CRC"
                    crc_bytes = []
                else:
                    rx_state = "PAYLOAD"
        elif rx_state == "PAYLOAD":
            payload.append(b)
            if len(payload) == expected_len:
                rx_state = "CRC"
                crc_bytes = []
        elif rx_state == "CRC":
            crc_bytes.append(b)
            if len(crc_bytes) == 2:
                received_crc = crc_bytes[0] | (crc_bytes[1] << 8)
                header_bytes = bytes(header)
                payload_bytes = bytes(payload)
                calc_crc = calc_crc16(header_bytes, 0)
                calc_crc = calc_crc16(payload_bytes, calc_crc)

                rx_state = "START1"
                if received_crc == calc_crc:
                    return header[0], payload_bytes  # data_type, payload
                else:
                    print(f"[CRC 錯誤] 收到: 0x{received_crc:04X}, 計算值: 0x{calc_crc:04X}")

    return None, None

def main():
    target_port = None
    if len(sys.argv) > 1:
        target_port = sys.argv[1]
    else:
        target_port = find_controller_port()

    if not target_port:
        print("[錯誤] 未找到任何可用的序列埠。請確認控制器已連接 USB 線。")
        sys.exit(1)

    print(f"[資訊] 正在開啟序列埠: {target_port} (Baud: 57600)...")
    try:
        ser = serial.Serial(port=target_port, baudrate=57600, timeout=1.0)
    except Exception as e:
        print(f"[錯誤] 無法開啟序列埠 {target_port}: {e}")
        sys.exit(1)

    try:
        # 1. 傳送控制器 Link 模式切換指令
        link_packet = build_packet(DATA_COMMAND, 2, DEV_BASE, DEV_CONTROLLER, bytes([CMD_MODE_CONTROLLER, OPTION_LINK_MODE]))
        print(f"[發送] 控制器切換 Link Mode: {link_packet.hex(' ')}")
        ser.write(link_packet)
        time.sleep(0.15)

        # 2. 請求狀態與電池資料
        req_packet = build_packet(DATA_REQUEST, 1, DEV_BASE, DEV_DRONE, bytes([DATA_STATE]))
        print(f"[發送] 請求 State 資料: {req_packet.hex(' ')}")

        # 進行最多 5 次查詢重試
        battery_found = False
        for attempt in range(5):
            ser.write(req_packet)
            data_type, payload = parse_incoming_stream(ser, timeout_sec=1.0)
            if data_type == DATA_STATE and len(payload) >= 8:
                mode_system, mode_flight, mode_ctrl, mode_move, headless, speed, sensor_orient, battery = payload[:8]
                flight_str = MODE_FLIGHT_MAP.get(mode_flight, f"未知 (0x{mode_flight:02X})")
                print("\n===============================")
                print(f"連線成功！")
                print(f"無人機飛行狀態: {flight_str}")
                print(f"無人機電池電量: {battery} %")
                print("===============================\n")
                battery_found = True
                break
            time.sleep(0.2)

        if not battery_found:
            print("[警告] 尚未收到無人機狀態回應。請確認無人機已開機並與控制器成功配對。")

    finally:
        ser.close()
        print(f"[資訊] 序列埠 {target_port} 已關閉。")

if __name__ == "__main__":
    main()
