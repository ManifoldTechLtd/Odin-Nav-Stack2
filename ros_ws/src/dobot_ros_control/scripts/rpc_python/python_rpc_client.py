#!/usr/bin/env python3
"""Python client for the sim2real JSON-RPC TCP server."""

from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Any, Dict, Optional, Sequence


DEFAULT_HOST = "192.168.5.2"
DEFAULT_PORT = 51234


class RpcError(Exception):
    """Base class for RPC client errors."""


class RpcTransportError(RpcError):
    """Raised when the TCP connection or data transfer fails."""


class RpcProtocolError(RpcError):
    """Raised when the server response is not valid JSON-RPC."""


class RpcRemoteError(RpcError):
    """Raised when the server returns a JSON-RPC error object."""

    def __init__(self, code: Any, message: str, data: Any = None) -> None:
        self.code = code
        self.message = message
        self.data = data
        detail = f"RPC error {code}: {message}"
        if data is not None:
            detail += f" (data={data!r})"
        super().__init__(detail)


class Sim2RealRpcClient:
    """One-request-per-connection client matching BaseJsonRpcServer."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        timeout: float = 3.0,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout

    def call(
        self,
        method: str,
        params: Optional[Dict[str, Any]] = None,
        request_id: int = 1,
    ) -> Any:
        """Call a sim2real RPC method and return its ``result`` value."""
        if not method:
            raise ValueError("method must not be empty")
        if params is not None and not isinstance(params, dict):
            raise TypeError("params must be a JSON object")

        request = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params or {},
            "id": request_id,
        }
        payload = json.dumps(
            request, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")

        try:
            with socket.create_connection(
                (self.host, self.port), timeout=self.timeout
            ) as rpc_socket:
                rpc_socket.settimeout(self.timeout)
                rpc_socket.sendall(payload)
                response_data = bytearray()
                while True:
                    chunk = rpc_socket.recv(4096)
                    if not chunk:
                        break
                    response_data.extend(chunk)
        except (OSError, socket.timeout) as exc:
            raise RpcTransportError(
                f"cannot communicate with {self.host}:{self.port}: {exc}"
            ) from exc

        if not response_data:
            raise RpcProtocolError("server closed the connection without a response")

        try:
            response = json.loads(response_data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RpcProtocolError(f"invalid JSON response: {exc}") from exc

        if not isinstance(response, dict):
            raise RpcProtocolError("JSON-RPC response must be an object")
        if response.get("id") != request_id:
            raise RpcProtocolError(
                f"response id {response.get('id')!r} does not match {request_id!r}"
            )
        if "error" in response:
            error = response["error"]
            if not isinstance(error, dict):
                raise RpcProtocolError("JSON-RPC error must be an object")
            raise RpcRemoteError(
                error.get("code"),
                str(error.get("message", "unknown error")),
                error.get("data"),
            )
        if "result" not in response:
            raise RpcProtocolError("JSON-RPC response has no result or error")
        return response["result"]

    def get_fsm_id(self) -> Any:
        return self.call("GetFsmId")

    def set_fsm_id(self, fsm_id: int) -> Any:
        return self.call("SetFsmId", {"fsm_id": fsm_id})

    def set_velocity(
        self, vx: float, vy: float, vyaw: float, duration: float = 1.0
    ) -> Any:
        return self.call(
            "SetVel",
            {"vx": vx, "vy": vy, "vyaw": vyaw, "duration": duration},
        )

    def get_fsm_list(self) -> Any:
        return self.call("GetFsmList")

    def add_dance_policy(
        self,
        name: str,
        motion_loop_length: int,
        motion_hold_back_steps: int,
        motion_start_step: int = 0,
    ) -> Any:
        return self.call(
            "AddDancePolicy",
            {
                "name": name,
                "motion_loop_length": motion_loop_length,
                "motion_hold_back_steps": motion_hold_back_steps,
                "motion_start_step": motion_start_step,
            },
        )

    def trigger_trajectory_recording(self, is_on: bool) -> Any:
        return self.call("TriggerTrajectoryRecording", {"is_on": is_on})

    def reload_arm_skills_data(self) -> Any:
        return self.call("ReloadArmSkillsData")

    def execute_action(self, file_path: str) -> Any:
        return self.call("ExecuteAction", {"file_path": file_path})

    def send_action_mapping(self, action_mapping: Dict[str, str]) -> Any:
        if not all(
            isinstance(key, str) and isinstance(value, str)
            for key, value in action_mapping.items()
        ):
            raise TypeError("action_mapping keys and values must be strings")
        return self.call("SendActionMapping", {"action_mapping": action_mapping})


def _json_object(value: str) -> Dict[str, Any]:
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"invalid JSON: {exc}") from exc
    if not isinstance(parsed, dict):
        raise argparse.ArgumentTypeError("params must be a JSON object")
    return parsed


def _string_map(value: str) -> Dict[str, str]:
    parsed = _json_object(value)
    if not all(
        isinstance(key, str) and isinstance(item, str)
        for key, item in parsed.items()
    ):
        raise argparse.ArgumentTypeError("mapping keys and values must be strings")
    return parsed


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Call the sim2real JSON-RPC TCP service"
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="sim2real server IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="RPC TCP port")
    parser.add_argument(
        "--timeout", type=float, default=3.0, help="connect/read timeout in seconds"
    )

    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("get-fsm-id", help="call GetFsmId")

    set_fsm = commands.add_parser("set-fsm-id", help="call SetFsmId")
    set_fsm.add_argument("fsm_id", type=int)

    set_vel = commands.add_parser("set-vel", help="call SetVel")
    set_vel.add_argument("vx", type=float, help="forward velocity in m/s")
    set_vel.add_argument("vy", type=float, help="left velocity in m/s")
    set_vel.add_argument("vyaw", type=float, help="CCW angular velocity in rad/s")
    set_vel.add_argument("--duration", type=float, default=1.0, help="duration in seconds")

    commands.add_parser("get-fsm-list", help="list available FSM policies/skills")

    add_policy = commands.add_parser("add-dance-policy", help="add a dance policy")
    add_policy.add_argument("name", help="policy name")
    add_policy.add_argument("motion_loop_length", type=int)
    add_policy.add_argument("motion_hold_back_steps", type=int)
    add_policy.add_argument("--motion-start-step", type=int, default=0)

    record = commands.add_parser(
        "record-trajectory", help="start or stop trajectory recording"
    )
    record.add_argument("state", choices=("start", "stop"))

    commands.add_parser("reload-arm-skills", help="reload arm skill data")

    execute = commands.add_parser("execute-action", help="execute a skill file")
    execute.add_argument("file_path", help="skill path on the sim2real server")

    mapping = commands.add_parser(
        "send-action-mapping", help="map controller buttons to skill names"
    )
    mapping.add_argument(
        "mapping", type=_string_map, help='JSON object, for example \'{"R2Y":"wave"}\''
    )

    generic_call = commands.add_parser("call", help="call any RPC method")
    generic_call.add_argument("method")
    generic_call.add_argument(
        "params",
        nargs="?",
        default={},
        type=_json_object,
        help='JSON object, for example \'{"control_mode":2}\'',
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    client = Sim2RealRpcClient(args.host, args.port, args.timeout)

    try:
        if args.command == "get-fsm-id":
            result = client.get_fsm_id()
        elif args.command == "set-fsm-id":
            result = client.set_fsm_id(args.fsm_id)
        elif args.command == "set-vel":
            result = client.set_velocity(args.vx, args.vy, args.vyaw, args.duration)
        elif args.command == "get-fsm-list":
            result = client.get_fsm_list()
        elif args.command == "add-dance-policy":
            result = client.add_dance_policy(
                args.name,
                args.motion_loop_length,
                args.motion_hold_back_steps,
                args.motion_start_step,
            )
        elif args.command == "record-trajectory":
            result = client.trigger_trajectory_recording(args.state == "start")
        elif args.command == "reload-arm-skills":
            result = client.reload_arm_skills_data()
        elif args.command == "execute-action":
            result = client.execute_action(args.file_path)
        elif args.command == "send-action-mapping":
            result = client.send_action_mapping(args.mapping)
        else:
            result = client.call(args.method, args.params)
    except (RpcError, ValueError, TypeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
