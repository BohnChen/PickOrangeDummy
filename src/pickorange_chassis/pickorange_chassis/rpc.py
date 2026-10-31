"""Minimal msgpack-RPC client for the Arduino RouterBridge.

Transport is configurable so the same node works with or without group
membership on /var/run/arduino-router.sock:

    unix:/var/run/arduino-router.sock     # direct (needs arduino-router group)
    tcp:127.0.0.1:7501                    # via scripts/router_proxy.sh

Protocol (msgpack-rpc):
    request  = [0, msg_id, method, params]
    response = [1, msg_id, error, result]

The router may also push notifications ([2, method, params]); those are
ignored here.
"""

from __future__ import annotations

import socket
import threading
import time

import msgpack

REQ = 0
RESP = 1


class RpcError(RuntimeError):
    pass


class RpcClient:
    def __init__(self, target: str, default_timeout: float = 2.0):
        self._target = target
        self._timeout = default_timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._msg_id = 0
        self._unpacker = msgpack.Unpacker(raw=False)

    # ---------- transport ----------
    def _open(self) -> socket.socket:
        if self._target.startswith("unix:"):
            path = self._target[len("unix:"):]
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(self._timeout)
            s.connect(path)
        elif self._target.startswith("tcp:"):
            hostport = self._target[len("tcp:"):]
            host, _, port = hostport.rpartition(":")
            s = socket.create_connection((host or "127.0.0.1", int(port)),
                                         timeout=self._timeout)
        else:
            raise RpcError(f"bad rpc target: {self._target!r}")
        if s.family == socket.AF_INET:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._unpacker = msgpack.Unpacker(raw=False)
        return s

    def _ensure(self) -> socket.socket:
        if self._sock is None:
            self._sock = self._open()
        return self._sock

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                finally:
                    self._sock = None

    # ---------- rpc ----------
    def call(self, method: str, *params, timeout: float | None = None):
        tmo = self._timeout if timeout is None else timeout
        deadline = time.monotonic() + tmo

        with self._lock:
            last_exc: Exception | None = None
            for _attempt in range(2):  # one reconnect retry
                try:
                    sock = self._ensure()
                    self._msg_id += 1
                    mid = self._msg_id
                    sock.sendall(msgpack.packb([REQ, mid, method, list(params)]))

                    while True:
                        resp = self._next_message(sock, deadline)
                        if resp is None:
                            continue          # notification, ignore
                        _typ, rid, err, result = resp
                        if rid != mid:
                            continue          # stale response
                        if err is not None:
                            raise RpcError(f"{method}: {err}")
                        return result
                except (OSError, msgpack.UnpackException) as exc:
                    last_exc = exc
                    # drop the socket so the retry reconnects
                    try:
                        if self._sock:
                            self._sock.close()
                    except OSError:
                        pass
                    self._sock = None
                    continue
            raise RpcError(f"{method} failed: {last_exc}")

    def _next_message(self, sock: socket.socket, deadline: float):
        while True:
            for obj in self._unpacker:
                if isinstance(obj, (list, tuple)) and len(obj) == 4:
                    typ = obj[0]
                    if typ == RESP:
                        return obj
                    if typ == 2:  # notification
                        continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RpcError("response timeout")
            sock.settimeout(max(0.01, min(remaining, 0.5)))
            chunk = sock.recv(4096)
            if not chunk:
                raise OSError("connection closed by router")
            self._unpacker.feed(chunk)
