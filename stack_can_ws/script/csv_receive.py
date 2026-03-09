#!/usr/bin/env python3
import asyncio
import csv
import io
import pathlib
import websockets

HOST = '0.0.0.0'
PORT = 9002
SAVE_DIR = pathlib.Path('/root/stack_can_ws/data')

async def handle(ws, path):
    async for message in ws:
        if not isinstance(message, str):
            await ws.send('error: expected text (CSV) payload')
            continue
        # Basic CSV parse/validation
        try:
            rdr = csv.reader(io.StringIO(message))
            rows = [r for r in rdr]
        except Exception as e:
            await ws.send(f'error: csv parse failed: {e}')
            continue

        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        latest = SAVE_DIR / 'points.csv'
        try:
            if latest.exists():
                latest.unlink()  # 避免因文件属主导致的写权限问题
            with latest.open('w', encoding='utf-8', newline='') as f:
                f.write(message)
        except Exception as e:
            await ws.send(f'error: save points.csv failed: {e}')
            continue

        # Validate rows: expect at least 6 columns per row (index,lat,lon,angle,type,segment)
        valid = len(rows) > 0 and all(len(r) >= 6 for r in rows)
        await ws.send(f'saved:points.csv, rows={len(rows)}, valid={valid}')

async def main():
    print(f'WebSocket CSV server listening on {HOST}:{PORT}, saving to {SAVE_DIR}')
    async with websockets.serve(handle, HOST, PORT, ping_interval=20, ping_timeout=10):
        await asyncio.Future()  # run forever

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('Server stopped')
