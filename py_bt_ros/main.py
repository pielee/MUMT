import argparse
import asyncio
import cProfile
import signal

from modules.utils import set_config


parser = argparse.ArgumentParser(description="py_bt_ros")
parser.add_argument(
    "--config",
    type=str,
    default="scenarios/simple/configs/grape.yaml",
    help="Path to the configuration file (default: --config=scenarios/simple/configs/config.yaml)",
)
parser.add_argument(
    "--ns",
    type=str,
    default=None,
    help="Override agent namespace, e.g. --ns /Fire_UGV_2",
)
args = parser.parse_args()

set_config(args.config)
from modules.utils import config

if args.ns is not None:
    config["agent"]["namespaces"] = args.ns

from modules.bt_runner import BTRunner

bt_runner = BTRunner(config)


def _request_shutdown(*_):
    bt_runner.running = False


def _register_shutdown_handlers():
    sigterm = getattr(signal, "SIGTERM", None)
    if sigterm is None:
        return

    try:
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(sigterm, _request_shutdown)
    except (NotImplementedError, RuntimeError):
        try:
            signal.signal(sigterm, _request_shutdown)
        except (OSError, RuntimeError, ValueError):
            pass


async def loop():
    _register_shutdown_handlers()
    try:
        while bt_runner.running:
            bt_runner.handle_keyboard_events()
            if not bt_runner.paused:
                await bt_runner.step()
            bt_runner.render()
    finally:
        bt_runner.close()


def run():
    try:
        asyncio.run(loop())
    except KeyboardInterrupt:
        _request_shutdown()


if __name__ == "__main__":
    if config["bt_runner"]["profiling_mode"]:
        cProfile.run("run()", sort="cumulative")
    else:
        run()
