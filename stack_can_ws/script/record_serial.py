#!/usr/bin/env python3
import serial
import sys
import time

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
BAUD = 115200
OUTPUT_FILE = 'serial_recording.bin'

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"📡 Recording from {ser.name} at {BAUD} baud → {OUTPUT_FILE}")
    print("Press Ctrl+C to stop recording.")
    
    with open(OUTPUT_FILE, 'wb') as f:
        while True:
            data = ser.read(100)  # 每次最多读100字节（非阻塞）
            if data:
                f.write(data)
                f.flush()  # 确保写入磁盘
                # 可选：同时打印到终端（用于监控）
                # sys.stdout.buffer.write(data)
except KeyboardInterrupt:
    print("\n⏹️  Recording stopped.")
except Exception as e:
    print(f"❌ Error: {e}")
finally:
    ser.close()
