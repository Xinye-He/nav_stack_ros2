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
    # 根据路径决定处理哪种数据
    if path == '/fence':
        expected_file = 'fence.csv'
        min_columns = 3
        expected_header = ['idx', 'lat', 'lon']
    elif path == '/points':
        expected_file = 'points.csv'
        min_columns = 6
        expected_header = None  # 不强制校验 header，只校验列数
    else:
        await ws.send('error: unsupported path. Use /fence or /points')
        return

    async for message in ws:
        if not isinstance(message, str):
            await ws.send('error: expected text (CSV) payload')
            continue

        try:
            rdr = csv.reader(io.StringIO(message))
            rows = [r for r in rdr]
        except Exception as e:
            await ws.send(f'error: csv parse failed: {e}')
            continue

        if not rows:
            await ws.send('error: empty CSV')
            continue

        # 可选：校验表头（仅 fence）
        if expected_header is not None:
            if rows[0] != expected_header:
                await ws.send(f'error: invalid header. Expected: {expected_header}, got: {rows[0]}')
                continue
            data_rows = rows[1:]
        else:
            data_rows = rows

        # 校验列数
        if not all(len(r) >= min_columns for r in data_rows):
            await ws.send(f'error: some rows have fewer than {min_columns} columns')
            continue

        # 保存文件
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        save_path = SAVE_DIR / expected_file

        try:
            if save_path.exists():
                save_path.unlink()
            with save_path.open('w', encoding='utf-8', newline='') as f:
                f.write(message)
        except Exception as e:
            await ws.send(f'error: save {expected_file} failed: {e}')
            continue

        await ws.send(f'saved:{expected_file}, rows={len(rows)}, valid=True')

async def main():
    print(f'WebSocket CSV server listening on {HOST}:{PORT}, saving to {SAVE_DIR}')
    print('Use /points for path points (6+ cols), /fence for boundary (idx,lat,lon)')
    async with websockets.serve(handle, HOST, PORT, ping_interval=20, ping_timeout=10):
        await asyncio.Future()  # run forever

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('Server stopped')
