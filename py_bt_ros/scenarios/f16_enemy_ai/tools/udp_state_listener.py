import argparse
import json
import socket
import struct
import time


def try_decode_json(data):
    try:
        text = data.decode("utf-8").strip()
    except UnicodeDecodeError:
        return None, None

    try:
        return text, json.loads(text)
    except json.JSONDecodeError:
        return text, None


def float32_preview(data, max_values=16):
    usable = (len(data) // 4) * 4
    if usable <= 0:
        return []
    count = min(max_values, usable // 4)
    return list(struct.unpack("<" + ("f" * count), data[: count * 4]))


def main():
    parser = argparse.ArgumentParser(
        description="Listen for UDP state packets from Unreal and print JSON/text/raw previews."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5007)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--max-packets", type=int, default=10)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.host, args.port))
        sock.settimeout(args.timeout_s)

        print(f"[udp_state_listener] listening on {args.host}:{args.port}")
        print(
            "[udp_state_listener] waiting for Unreal UDP state packets "
            f"for up to {args.timeout_s:.1f}s"
        )

        received = 0
        start = time.time()
        while received < args.max_packets and (time.time() - start) < args.timeout_s:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                print("[udp_state_listener] timeout with no more packets.")
                break

            received += 1
            print(
                f"[udp_state_listener] packet={received} from={addr} bytes={len(data)}"
            )

            text, json_obj = try_decode_json(data)
            if json_obj is not None:
                print("[udp_state_listener] decoded_json=", json.dumps(json_obj, indent=2))
                continue

            if text is not None:
                print(f"[udp_state_listener] utf8_text={text}")

            print(f"[udp_state_listener] hex={data.hex()}")
            preview = float32_preview(data)
            if preview:
                print(f"[udp_state_listener] float32_le_preview={preview}")


if __name__ == "__main__":
    main()
