import argparse
import json
import socket
import struct
import time


def build_json_payload(args, include_actor_id=True):
    payload = {
        "actor_id": args.actor_id,
        "throttle": args.throttle,
        "elevator": args.elevator,
        "aileron": args.aileron,
        "rudder": args.rudder,
        "wheel_brake": args.wheel_brake,
        "parking_brake": args.parking_brake,
        "engine_start": args.engine_start,
        "control_enabled": args.control_enabled,
    }
    if not include_actor_id:
        payload.pop("actor_id", None)
    return payload


def build_text_fields(args, include_actor_id, include_aux_fields):
    fields = []
    if include_actor_id:
        fields.append(str(args.actor_id))
    fields.extend(
        [
            f"{args.aileron}",
            f"{args.elevator}",
            f"{args.rudder}",
            f"{args.throttle}",
        ]
    )
    if include_aux_fields:
        fields.extend(
            [
                f"{args.wheel_brake}",
                "1" if args.parking_brake else "0",
                "1" if args.engine_start else "0",
                "1" if args.control_enabled else "0",
            ]
        )
    return fields


def encode_payload(args):
    if args.format == "json":
        payload = build_json_payload(args, include_actor_id=True)
        return json.dumps(payload).encode("utf-8"), payload

    if args.format == "json_no_actor":
        payload = build_json_payload(args, include_actor_id=False)
        return json.dumps(payload).encode("utf-8"), payload

    if args.format == "csv4":
        text = args.delimiter.join(build_text_fields(args, False, False))
        return text.encode("utf-8"), text

    if args.format == "csv8":
        text = args.delimiter.join(build_text_fields(args, False, True))
        return text.encode("utf-8"), text

    if args.format == "csv5_actor_prefix":
        text = args.delimiter.join(build_text_fields(args, True, False))
        return text.encode("utf-8"), text

    if args.format == "csv9_actor_prefix":
        text = args.delimiter.join(build_text_fields(args, True, True))
        return text.encode("utf-8"), text

    if args.format == "f32x4":
        values = [args.aileron, args.elevator, args.rudder, args.throttle]
        return struct.pack("<ffff", *values), values

    if args.format == "f32x8":
        values = [
            args.aileron,
            args.elevator,
            args.rudder,
            args.throttle,
            args.wheel_brake,
            float(args.parking_brake),
            float(args.engine_start),
            float(args.control_enabled),
        ]
        return struct.pack("<ffffffff", *values), values

    raise ValueError(f"Unsupported format: {args.format}")


def main():
    parser = argparse.ArgumentParser(
        description="Send a minimal UDP command payload to the Unreal F-16 bridge."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--actor-id", default="EnemyF16_01")
    parser.add_argument("--throttle", type=float, default=0.8)
    parser.add_argument("--elevator", type=float, default=0.0)
    parser.add_argument("--aileron", type=float, default=0.0)
    parser.add_argument("--rudder", type=float, default=0.0)
    parser.add_argument("--wheel-brake", type=float, default=0.0)
    parser.add_argument("--parking-brake", action="store_true")
    parser.add_argument(
        "--format",
        choices=[
            "json",
            "json_no_actor",
            "csv4",
            "csv8",
            "csv5_actor_prefix",
            "csv9_actor_prefix",
            "f32x4",
            "f32x8",
        ],
        default="csv4",
    )
    parser.add_argument("--delimiter", default=",")
    parser.add_argument("--engine-start", action="store_true", default=True)
    parser.add_argument(
        "--no-engine-start", dest="engine_start", action="store_false"
    )
    parser.add_argument("--control-enabled", action="store_true", default=True)
    parser.add_argument(
        "--no-control-enabled", dest="control_enabled", action="store_false"
    )
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval-s", type=float, default=0.2)
    parser.add_argument("--terminator", default="")
    args = parser.parse_args()

    encoded, payload = encode_payload(args)
    if args.terminator:
        encoded += args.terminator.encode("utf-8")

    print(f"[udp_command_smoke_test] target={args.host}:{args.port}")
    print(f"[udp_command_smoke_test] format={args.format}")
    print(
        "[udp_command_smoke_test] receiver_mapping="
        "roll<-aileron pitch<-elevator yaw<-rudder throttle<-throttle"
    )
    print(f"[udp_command_smoke_test] delimiter={args.delimiter!r}")
    print(f"[udp_command_smoke_test] terminator={args.terminator!r}")
    print(f"[udp_command_smoke_test] payload={payload}")
    print(f"[udp_command_smoke_test] bytes={encoded!r}")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for step in range(args.repeat):
            sent = sock.sendto(encoded, (args.host, args.port))
            print(
                f"[udp_command_smoke_test] step={step + 1}/{args.repeat} bytes_sent={sent}"
            )
            if step + 1 < args.repeat:
                time.sleep(args.interval_s)


if __name__ == "__main__":
    main()
