#!/usr/bin/env python3
import argparse
import struct
import time

import can


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_payload_exact(
    speed_kmh,
    angle_deg,
    dist_m,
    pick=False,
    unload=False,
    dump=False,
    pick_action=False,
    remote_bit=False,
    estop=False,
    drive=True,
):
    """
    严格按 stack_can_executor.py 的 send_can() 方式打包

    字段来源（与你仓库一致）：
    - dist_raw  = round(max(0, dist_m) / 0.2), 限幅 0~150
    - angle_raw = round(angle_deg + 180), 限幅 0~359
      注：正=右转，负=左转
    - speed_raw = round((speed_kmh - (-50.0)) / 0.5), 限幅 0~255

    byte5:
      bit0 pick
      bit2 unload
      bit4 remote_bit
      bit5 dump
      bit6 pick_action

    byte6:
      bit0 estop
      bit1 drive

    pack_into('<H H B B B', payload, 0, dist_raw, angle_raw, speed_raw, byte5, byte6)
    """
    # 1) 距离
    dist_raw = int(round(max(0.0, float(dist_m)) / 0.2))
    dist_raw = clamp(dist_raw, 0, 150)

    # 2) 方向角：正=右转，负=左转
    steer_deg = float(angle_deg)
    if steer_deg > 180.0:
        steer_deg = 180.0
    if steer_deg < -180.0:
        steer_deg = -180.0
    angle_raw = int(round(steer_deg + 180.0))
    angle_raw = clamp(angle_raw, 0, 359)

    # 3) 速度
    speed_raw = int(round((float(speed_kmh) - (-50.0)) / 0.5))
    speed_raw = clamp(speed_raw, 0, 255)

    # 4) byte5
    byte5 = 0
    if pick:
        byte5 |= 0x01
    if unload:
        byte5 |= (0x01 << 2)
    if remote_bit:
        byte5 |= (0x01 << 4)
    if dump:
        byte5 |= (0x01 << 5)
    if pick_action:
        byte5 |= (0x01 << 6)

    # 5) byte6
    byte6 = 0
    if estop:
        byte6 |= 0x01
    if drive:
        byte6 |= (0x01 << 1)

    payload = bytearray(8)
    struct.pack_into('<H H B B B', payload, 0, dist_raw, angle_raw, speed_raw, byte5, byte6)
    return payload, dist_raw, angle_raw, speed_raw, byte5, byte6


def send_frame(bus, can_id, payload):
    msg = can.Message(
        arbitration_id=can_id,
        is_extended_id=True,
        data=payload
    )
    bus.send(msg, timeout=0.001)


def run_continuous(bus, can_id, angle_deg, speed_kmh, dist_m, period_s, total_s, verbose):
    t0 = time.time()
    count = 0

    while True:
        now = time.time()
        if total_s > 0 and (now - t0) >= total_s:
            break

        payload, dist_raw, angle_raw, speed_raw, byte5, byte6 = build_payload_exact(
            speed_kmh=speed_kmh,
            angle_deg=angle_deg,
            dist_m=dist_m,
            pick=False,
            unload=False,
            dump=False,
            pick_action=False,
            remote_bit=False,
            estop=False,
            drive=True,
        )
        send_frame(bus, can_id, payload)
        count += 1

        if verbose:
            print(
                f"[{count:04d}] "
                f"angle={angle_deg:+.1f} speed={speed_kmh:.1f} "
                f"dist_raw={dist_raw} angle_raw={angle_raw} speed_raw={speed_raw} "
                f"byte5=0x{byte5:02X} byte6=0x{byte6:02X} "
                f"payload={[hex(b) for b in payload]}"
            )

        time.sleep(period_s)

    print(f"done: sent {count} frames in continuous mode")


def run_pulse_pause(bus, can_id, angle_deg, speed_kmh, dist_m,
                    frame_period_s, pulse_frames, pause_frames, total_s, verbose):
    t0 = time.time()
    cycle_count = 0
    sent_count = 0

    while True:
        now = time.time()
        if total_s > 0 and (now - t0) >= total_s:
            break

        cycle_count += 1

        # 脉冲阶段：发送原地左/右转
        for _ in range(pulse_frames):
            now = time.time()
            if total_s > 0 and (now - t0) >= total_s:
                break

            payload, dist_raw, angle_raw, speed_raw, byte5, byte6 = build_payload_exact(
                speed_kmh=speed_kmh,
                angle_deg=angle_deg,
                dist_m=dist_m,
                pick=False,
                unload=False,
                dump=False,
                pick_action=False,
                remote_bit=False,
                estop=False,
                drive=True,
            )
            send_frame(bus, can_id, payload)
            sent_count += 1

            if verbose:
                print(
                    f"[cycle {cycle_count:04d}] PULSE "
                    f"angle={angle_deg:+.1f} speed={speed_kmh:.1f} "
                    f"dist_raw={dist_raw} angle_raw={angle_raw} speed_raw={speed_raw} "
                    f"byte5=0x{byte5:02X} byte6=0x{byte6:02X} "
                    f"payload={[hex(b) for b in payload]}"
                )

            time.sleep(frame_period_s)

        # 暂停阶段：发送中性命令（停住、角度归中）
        for _ in range(pause_frames):
            now = time.time()
            if total_s > 0 and (now - t0) >= total_s:
                break

            payload, dist_raw, angle_raw, speed_raw, byte5, byte6 = build_payload_exact(
                speed_kmh=0.0,
                angle_deg=0.0,
                dist_m=dist_m,
                pick=False,
                unload=False,
                dump=False,
                pick_action=False,
                remote_bit=False,
                estop=False,
                drive=True,
            )
            send_frame(bus, can_id, payload)
            sent_count += 1

            if verbose:
                print(
                    f"[cycle {cycle_count:04d}] PAUSE "
                    f"angle=0.0 speed=0.0 "
                    f"dist_raw={dist_raw} angle_raw={angle_raw} speed_raw={speed_raw} "
                    f"byte5=0x{byte5:02X} byte6=0x{byte6:02X} "
                    f"payload={[hex(b) for b in payload]}"
                )

            time.sleep(frame_period_s)

    print(f"done: sent {sent_count} frames in pulse-pause mode")


def main():
    ap = argparse.ArgumentParser(description="CAN spin test aligned to stack_can_executor.py exactly")
    ap.add_argument("--if", dest="iface", default="can0", help="SocketCAN interface, e.g. can0")
    ap.add_argument("--id", dest="canid", default="0x18FED188",
                    help="Extended CAN ID, default matches stack_can_executor.py")
    ap.add_argument("--angle", type=float, default=-10.0,
                    help="Steering angle in deg. IMPORTANT: negative=left, positive=right. Default=-10")
    ap.add_argument("--speed", type=float, default=0.0,
                    help="Speed in km/h. Default=0.0 for in-place rotation test")
    ap.add_argument("--dist", type=float, default=2.0,
                    help="Distance field, default=2.0m")
    ap.add_argument("--period", type=float, default=0.05,
                    help="Frame period in seconds. Default=0.05 (20Hz)")
    ap.add_argument("--total", type=float, default=5.0,
                    help="Total duration in seconds. <=0 means forever")
    ap.add_argument("--mode", choices=["continuous", "pulse"], default="continuous",
                    help="continuous or pulse")
    ap.add_argument("--pulse-frames", type=int, default=1,
                    help="Number of turning frames in one pulse")
    ap.add_argument("--pause-frames", type=int, default=4,
                    help="Number of neutral frames between pulses")
    ap.add_argument("--verbose", action="store_true", help="Print every frame detail")
    args = ap.parse_args()

    can_id = int(args.canid, 0)

    print("=== CAN spin test (executor exact) ===")
    print(f"iface         : {args.iface}")
    print(f"can id        : 0x{can_id:08X}")
    print(f"mode          : {args.mode}")
    print(f"angle         : {args.angle:+.1f} deg   (negative=left, positive=right)")
    print(f"speed         : {args.speed:.1f} km/h")
    print(f"dist          : {args.dist:.1f} m")
    print(f"period        : {args.period:.3f} s")
    print(f"total         : {args.total:.1f} s")
    if args.mode == "pulse":
        print(f"pulse_frames  : {args.pulse_frames}")
        print(f"pause_frames  : {args.pause_frames}")

    bus = can.interface.Bus(channel=args.iface, bustype="socketcan")

    try:
        if args.mode == "continuous":
            run_continuous(
                bus=bus,
                can_id=can_id,
                angle_deg=args.angle,
                speed_kmh=args.speed,
                dist_m=args.dist,
                period_s=args.period,
                total_s=args.total,
                verbose=args.verbose,
            )
        else:
            run_pulse_pause(
                bus=bus,
                can_id=can_id,
                angle_deg=args.angle,
                speed_kmh=args.speed,
                dist_m=args.dist,
                frame_period_s=args.period,
                pulse_frames=args.pulse_frames,
                pause_frames=args.pause_frames,
                total_s=args.total,
                verbose=args.verbose,
            )
    except KeyboardInterrupt:
        print("\ninterrupted by user")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
