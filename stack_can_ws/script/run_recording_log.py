#!/usr/bin/env python3
"""Mirror console output while limiting persistent console logs per run."""
import sys

PREFIX = sys.argv[1]
LIMIT = 50 * 1024 * 1024
FILES = 10
stream = None
written = 0
index = 0
try:
    while True:
        chunk = sys.stdin.buffer.read1(65536)
        if not chunk:
            break
        try:
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()
        except (BrokenPipeError, OSError):
            pass
        if index >= FILES:
            continue
        try:
            if stream is None:
                stream = open(f'{PREFIX}.{index:02d}.log', 'ab', buffering=0)
                written = stream.tell()
            while chunk and index < FILES:
                part, chunk = chunk[:LIMIT-written], chunk[LIMIT-written:]
                stream.write(part)
                written += len(part)
                if written >= LIMIT:
                    stream.close()
                    stream = None
                    index += 1
                    written = 0
                    if chunk and index < FILES:
                        stream = open(f'{PREFIX}.{index:02d}.log', 'ab', buffering=0)
        except OSError as exc:
            print(f'[recording] 日志写入失败: {exc}', file=sys.stderr)
            index = FILES
finally:
    if stream is not None:
        stream.close()
