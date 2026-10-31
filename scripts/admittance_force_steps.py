#!/usr/bin/env python3
"""Apply a signed mN staircase to one admittance target; never enable motors.

Example: python3 scripts/admittance_force_steps.py --axis fy --interval 3
Targets use the GUI convention: signed force ON the HRM, not raw model output.
Ctrl+C requests zero on the selected axis; it is NOT an emergency stop.
"""

import argparse
import math
import os
import signal
import sys
import time


STEPS_MN = (0, 100, 250, 350, 470, 350, 250, 100, 0)
TARGET_NAMES = ("admittance/target_force_y_N", "admittance/target_force_z_N")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", choices=("fy", "fz"), required=True)
    parser.add_argument("--interval", type=float, default=5.0,
                        help="Seconds to hold EACH acknowledged target (default: 3).")
    parser.add_argument("--sign", type=int, choices=(-1, 1), default=1,
                        help="Target sign in hrm_base; same convention as GUI (default: +1).")
    parser.add_argument("--node", default="/robot_control", help="Controller node name.")
    parser.add_argument("--timeout", type=float, default=3.0, help="ROS service timeout [s].")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print sequence only: no ROS connection or parameter writes.")
    return parser


def validate_args(args):
    if args.axis not in ("fy", "fz") or args.sign not in (-1, 1):
        raise ValueError("axis must be fy/fz and sign must be +1/-1")
    for name in ("interval", "timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not args.node.strip("/") or any(c.isspace() for c in args.node):
        raise ValueError("Specify a nonempty ROS node name without whitespace")
    return tuple(args.sign * value / 1000.0 for value in STEPS_MN)


def run_sequence(values_N, interval, write_target, wait, emit=print):
    """Wait after ACK; never catch up missed steps with rapid commands."""
    for index, value in enumerate(values_N, 1):
        write_target(value)
        emit(f"[{index}/{len(values_N)}] target={value * 1000:+.0f} mN; "
             f"hold {interval:g} s")
        wait(interval)


class ParameterTarget:
    """ROS adapter: read settings and write ONLY the selected force target."""

    def __init__(self, args):
        import rclpy
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically

        self.ros = rclpy
        self.timeout = args.timeout
        self.axis_index = ("fy", "fz").index(args.axis)
        self.name = TARGET_NAMES[self.axis_index]
        self.node = rclpy.create_node(f"admittance_force_steps_{os.getpid()}")
        remote = "/" + args.node.strip("/")
        self.reader = self.node.create_client(GetParameters, remote + "/get_parameters")
        self.writer = self.node.create_client(
            SetParametersAtomically, remote + "/set_parameters_atomically")

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        self.ros.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done():
            raise RuntimeError(f"Service response timeout; target state unknown: {client.srv_name}")
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        result = future.result()
        if result is None:
            raise RuntimeError("Empty parameter service response")
        return result

    def preflight(self, values_N):
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters

        names = list(TARGET_NAMES) + ["admittance/max_force_N"]
        response = self._call(self.reader, GetParameters.Request(names=names))
        if len(response.values) != 3 or any(
                v.type != ParameterType.PARAMETER_DOUBLE for v in response.values):
            raise RuntimeError("Controller must expose both force targets and max_force_N as doubles")
        y, z, maximum = (v.double_value for v in response.values)
        if not all(math.isfinite(v) for v in (y, z, maximum)) or maximum <= 0:
            raise RuntimeError("Invalid controller force settings")
        other = (z, y)[self.axis_index]
        if any(math.hypot(value, other) > maximum for value in values_N):
            raise RuntimeError("Sequence plus unchanged other-axis target exceeds max_force_N")
        print(f"Current Fy={y * 1000:+g}, Fz={z * 1000:+g} mN; "
              f"vector limit={maximum * 1000:g} mN", flush=True)
        print(f"Writing only {self.name}; other axis unchanged.", flush=True)

    def write(self, value):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter

        request = SetParametersAtomically.Request(parameters=[
            Parameter(self.name, value=float(value)).to_parameter_msg()])
        result = self._call(self.writer, request).result
        if not result.successful:
            raise RuntimeError(f"Target rejected: {result.reason}")

    def close(self):
        self.node.destroy_node()


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        values = validate_args(args)
    except ValueError as error:
        parser.error(str(error))
    print(f"{args.axis.upper()} [mN]: " + " -> ".join(f"{v * 1000:+.0f}" for v in values),
          flush=True)
    print(f"Each step {args.interval:g} s, including first/last zero "
          f"(~{len(values) * args.interval:g} s plus service latency).", flush=True)
    if args.dry_run:
        print("Dry run: no ROS connection or writes.")
        return 0

    # Leave Python's Ctrl+C handler active so ROS remains usable for zero cleanup.
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    adapter = None
    attempted = completed = initialized = False
    exit_code = 0
    old_term = signal.getsignal(signal.SIGTERM)

    def interrupted(unused_signal, unused_frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    print("WARNING: target changes affect motion immediately in enabled Admittance mode.\n"
          "No mode/enable/gain changes. Do not edit the same target in GUI concurrently.\n"
          "Ctrl+C requests selected-axis zero, NOT a motor stop.", flush=True)
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        adapter = ParameterTarget(args)
        adapter.preflight(values)
        # A timed-out set may still have been applied: attempt cleanup in that case too.
        attempted = True
        run_sequence(values, args.interval, adapter.write, time.sleep,
                     emit=lambda text: print(text, flush=True))
        completed = True
        print("Completed; selected-axis target remains 0 mN. Controller is still running.",
              flush=True)
    except KeyboardInterrupt:
        exit_code = 130
        print("Interrupted; no further sequence steps.", file=sys.stderr, flush=True)
    except Exception as error:
        exit_code = 1
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
    finally:
        if adapter is not None:
            try:
                if attempted and not completed:
                    try:
                        adapter.write(0.0)
                        print("Selected-axis 0 mN acknowledged; this is NOT a motor stop.",
                              file=sys.stderr, flush=True)
                    except (Exception, KeyboardInterrupt) as error:
                        exit_code = 1
                        print(f"ZERO RESET FAILED: {error}. Last target may remain active; "
                              "disable Motor Enable/check GUI manually.", file=sys.stderr, flush=True)
            finally:
                adapter.close()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
