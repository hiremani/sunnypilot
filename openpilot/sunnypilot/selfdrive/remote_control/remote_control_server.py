#!/usr/bin/env python3
"""
Remote-control server for the fisker port.

Runs as a sunnypilot daemon, binds to 0.0.0.0:8899 on the Comma 3X. A phone on
the same wifi network opens http://<comma-ip>:8899/ in Chrome and uses the
served webapp to send steering commands. The commands are written to Params;
the fisker carcontroller reads them each tick and (when the safety gates pass)
substitutes the remote angle for its lateral planner's output.

Safety / design notes:
  * Pure local-network HTTP. No TLS, no auth. Trusted-wifi only.
  * Opt-in at process level: manager starts this daemon only while the
    RemoteControlEnabled param is true (process_config.remote_control). The
    param is CLEAR_ON_MANAGER_START, so every boot comes up with no server and
    no listening port. This server never writes that param.
  * Clients must be on a private, link-local or loopback address
    (client_allowed), the same rule the web HUD applies to its writes; POSTs
    from another web origin are refused too.
  * Arm / disarm is this server's own state (Handler.armed, off at start).
    /steer is refused until a client has POSTed /arm, so opening the page
    never steers by itself. /disarm zeroes the angle and the timestamp, which
    makes the carcontroller's freshness check fail on its next tick.
  * Hard angle cap (±MAX_ANGLE_DEG) enforced on EVERY command, server-side.
  * Freshness watchdog is enforced in the carcontroller — carstate reads the
    last-updated timestamp and treats commands older than ~300 ms as stale.
  * The actual lateral-control safety gates (MADS engaged, low speed,
    driver intervention) live in fisker/carcontroller.py and run every tick.
"""

from __future__ import annotations

import http.server
import ipaddress
import json
import os
import socketserver
import time
from pathlib import Path
from urllib.parse import urlparse

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


PORT = int(os.environ.get("REMOTE_CONTROL_PORT", "8899"))
STATIC_DIR = Path(__file__).parent / "static"

# Hard angle cap (deg). The fisker ANGLE_LIMITS.max_angle is 600° (physical
# EPS limit on this trim); 250° gives the phone roughly ±2/3 of a full wheel
# rotation per direction, enough for sharp parking lock-to-lock from a
# cold start. Still below the panda hard max so a stuck finger can't put
# the wheel to the hard stop.
MAX_ANGLE_DEG = 250.0
MAX_BODY_BYTES = 4096   # the app's POST bodies are a few dozen bytes


def client_allowed(address: str) -> bool:
  """Only a client on the local network may talk to this server: a private (RFC 1918 / ULA),
  link-local or loopback address. Same test as the web HUD's write check
  (sunnypilot/webhud/server.py), except that an address that doesn't parse is refused."""
  try:
    ip = ipaddress.ip_address(address.split("%", 1)[0])  # drop an IPv6 zone id
  except ValueError:
    return False
  if isinstance(ip, ipaddress.IPv6Address) and ip.ipv4_mapped is not None:
    ip = ip.ipv4_mapped
  return ip.is_private or ip.is_link_local or ip.is_loopback


class Handler(http.server.BaseHTTPRequestHandler):
  # Populated in main() before the server starts.
  params: Params = None  # type: ignore[assignment]
  # Set by /arm, cleared by /disarm and at daemon start. Shared by every request thread.
  armed: bool = False

  # Socket timeout per request. BaseHTTPRequestHandler sets no timeout by
  # default, so when a client ungracefully disconnects (wifi drop, tab
  # closed, phone backgrounded) its handler thread sits blocked in
  # socket.recv forever waiting on data that will never arrive — the OS
  # eventually reclaims the socket in CLOSE_WAIT but that can take minutes.
  # During that window: new clients see sluggish responses, in-flight
  # heartbeats stack up behind the kernel's accept queue, and the whole
  # thing looks like "server is broken."
  #
  # 5 s is way more than any real local-wifi roundtrip needs (which is
  # ~2-5 ms) but far shorter than the kernel's default; stuck threads die
  # cleanly and ThreadingMixIn spins up fresh ones for new clients.
  timeout = 5.0

  # Silence the default access-log spam (BaseHTTPRequestHandler prints every
  # request to stderr by default, which fills journald during active steering).
  def log_message(self, fmt, *args):
    pass

  def _send(self, status: int, body: bytes | str = b"", ctype: str = "application/json") -> None:
    self.send_response(status)
    self.send_header("Content-Type", ctype)
    # No CORS headers: the page is served from this same origin, and a page from
    # any other origin has no business steering the car.
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    if isinstance(body, str):
      body = body.encode()
    if body:
      self.wfile.write(body)

  def _client_ip(self) -> str:
    return str(self.client_address[0])

  def _client_ok(self) -> bool:
    """Every request: the client has to be on the local network."""
    if client_allowed(self._client_ip()):
      return True
    self._send(403, json.dumps({"error": "only accepted from the local network"}))
    return False

  def _same_origin(self) -> bool:
    """Writes: never cross-origin (a web page open on the phone must not be able to steer)."""
    origin = self.headers.get("Origin")
    if origin and urlparse(origin).netloc != self.headers.get("Host"):
      self._send(403, json.dumps({"error": "cross-origin request"}))
      return False
    return True

  def do_OPTIONS(self):
    if not self._client_ok():
      return
    self._send(204)

  def do_GET(self):
    if not self._client_ok():
      return
    if self.path in ("/", "/index.html"):
      self._serve_static("index.html", "text/html; charset=utf-8")
    elif self.path == "/style.css":
      self._serve_static("style.css", "text/css; charset=utf-8")
    elif self.path == "/app.js":
      self._serve_static("app.js", "application/javascript; charset=utf-8")
    elif self.path == "/status":
      self._send(200, self._status_json())
    else:
      self._send(404, json.dumps({"error": "not found"}))

  def do_POST(self):
    # Read the (small) body before any refusal: closing a socket with unread data resets the
    # connection and the client never sees the status.
    try:
      length = int(self.headers.get("Content-Length", 0))
    except ValueError:
      length = -1
    if not 0 <= length <= MAX_BODY_BYTES:
      self.close_connection = True
      self._send(400, json.dumps({"error": "invalid body length"}))
      return
    body = self.rfile.read(length) if length else b""
    if not self._client_ok() or not self._same_origin():
      return
    try:
      data = json.loads(body) if body else {}
    except json.JSONDecodeError:
      self._send(400, json.dumps({"error": "bad json"}))
      return

    if self.path == "/steer":
      if not Handler.armed:
        self._send(409, json.dumps({"error": "not armed"}))
        return
      try:
        angle = float(data.get("angle", 0.0))
      except (TypeError, ValueError):
        self._send(400, json.dumps({"error": "bad angle"}))
        return
      # Hard safety cap — clamp before writing anywhere.
      if angle > MAX_ANGLE_DEG:
        angle = MAX_ANGLE_DEG
      elif angle < -MAX_ANGLE_DEG:
        angle = -MAX_ANGLE_DEG
      # Convention flip: the webapp sends "drag-right = positive" because
      # that's what a human expects from a steering input. openpilot's
      # internal frame is left-positive (CCW = +), matching OEM EPS_SteerWhlAgSig
      # and ADAS_LatCtrl_SteerAnReq (see fisker-ocean-port-status memory).
      # Negate here so the stored param lands in openpilot's convention and
      # the carcontroller can use it directly, without having to carry a
      # magic "remote-is-reversed" flag through the control path.
      stored_angle = -angle
      self.params.put("RemoteSteerAngle", str(stored_angle))
      self.params.put("RemoteSteerLastUpdate", str(time.time_ns()))
      self._send(200, json.dumps({"ok": True, "angle": angle}))

    elif self.path == "/arm":
      # Arming also seeds a fresh zero angle + timestamp so the first few
      # ticks after enabling don't see a stale "last angle" from a previous
      # session.
      self.params.put("RemoteSteerAngle", "0")
      self.params.put("RemoteSteerLastUpdate", str(time.time_ns()))
      Handler.armed = True
      self._send(200, json.dumps({"ok": True}))

    elif self.path == "/disarm":
      # Timestamp 0 is stale for the carcontroller at once; RemoteControlEnabled is the
      # owner's process-level opt-in and is left alone, so the phone can arm again.
      Handler.armed = False
      self.params.put("RemoteSteerAngle", "0")
      self.params.put("RemoteSteerLastUpdate", "0")
      self._send(200, json.dumps({"ok": True}))

    else:
      self._send(404, json.dumps({"error": "not found"}))

  # ------------------------------------------------------------------- helpers

  def _status_json(self) -> str:
    now_ns = time.time_ns()
    try:
      last_ns = int((self.params.get("RemoteSteerLastUpdate") or b"0").decode())
    except (ValueError, AttributeError):
      last_ns = 0
    try:
      angle = float((self.params.get("RemoteSteerAngle") or b"0").decode())
    except (ValueError, AttributeError):
      angle = 0.0
    return json.dumps({
      "armed": Handler.armed,
      "angle": angle,
      "ageMs": (now_ns - last_ns) / 1_000_000.0 if last_ns > 0 else None,
      "maxAngleDeg": MAX_ANGLE_DEG,
      "serverTimeNs": now_ns,
    })

  def _serve_static(self, name: str, ctype: str) -> None:
    p = STATIC_DIR / name
    if not p.exists():
      self._send(404, json.dumps({"error": "static missing: " + name}))
      return
    try:
      self._send(200, p.read_bytes(), ctype)
    except OSError as e:
      self._send(500, json.dumps({"error": str(e)}))


class ThreadingServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
  """One thread per request. Phone + optional desktop-browser debug client at
  the same time is well within budget."""
  daemon_threads = True
  allow_reuse_address = True
  # Keep per-connection TCP keepalive on so the kernel notices half-open
  # connections (wifi drops, phone sleep) and reclaims the socket rather
  # than letting the handler thread sit on it. Combined with Handler.timeout
  # above, a client disconnect leads to a clean exception + thread exit
  # within a few seconds instead of up to tens of minutes.
  request_queue_size = 32

  def server_bind(self):
    super().server_bind()
    import socket as _s
    self.socket.setsockopt(_s.SOL_SOCKET, _s.SO_KEEPALIVE, 1)


def main():
  Handler.params = Params()
  # Sanity-reset state at daemon start, in case manager restarted us mid-drive
  # or a param write got half-committed. RemoteControlEnabled is not touched: it is
  # what manager starts and stops this process on.
  Handler.armed = False
  Handler.params.put("RemoteSteerAngle", "0")
  Handler.params.put("RemoteSteerLastUpdate", "0")

  cloudlog.info(f"remote_control_server: binding 0.0.0.0:{PORT}")
  with ThreadingServer(("0.0.0.0", PORT), Handler) as httpd:
    httpd.serve_forever()


if __name__ == "__main__":
  main()
