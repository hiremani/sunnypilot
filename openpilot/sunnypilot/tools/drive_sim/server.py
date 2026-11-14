#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

drive_sim: replay a recorded drive through the current Stop Sign Assist, lane-turn and turn-planner code and show it
on a map ("Turn planner" mode: the planner's ghost path from each point it would start a signaled turn).

  .venv/bin/python -m openpilot.sunnypilot.tools.drive_sim.server --root ~/Fisker/routes
  then open http://127.0.0.1:8765

Every "Run" reloads the stop-sign and lane-turn modules from disk, so an edit shows on the next run without
restarting. Logs are parsed once per route/segment range and cached (~/.cache/fisker_drive_sim). Listens on
127.0.0.1 only.
"""
import argparse
import importlib
import json
import os
import sys
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
RELOAD = (   # dependencies first
  "openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.road_geometry",
  "openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_direction",
  "openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_map",
  "openpilot.sunnypilot.selfdrive.controls.lib.stop_sign.stop_sign_controller",
  "openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire",
  "openpilot.sunnypilot.selfdrive.controls.lib.turn_planner",
  "openpilot.sunnypilot.selfdrive.controls.lib.narrow_street",
  "openpilot.sunnypilot.selfdrive.controls.lib.driver_profile",
  "openpilot.sunnypilot.tools.drive_sim.sim",
)


def reload_code():
  for name in RELOAD:
    mod = sys.modules.get(name)
    if mod is None:
      importlib.import_module(name)
    else:
      importlib.reload(mod)
  return sys.modules["openpilot.sunnypilot.tools.drive_sim.sim"]


def make_handler(root: str):
  class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
      sys.stderr.write("drive_sim: " + (fmt % args) + "\n")

    def _send(self, code: int, body: bytes, ctype: str):
      self.send_response(code)
      self.send_header("Content-Type", ctype)
      self.send_header("Content-Length", str(len(body)))
      self.send_header("Cache-Control", "no-store")
      self.end_headers()
      self.wfile.write(body)

    def _json(self, code: int, obj):
      self._send(code, json.dumps(obj, separators=(",", ":")).encode(), "application/json")

    def do_GET(self):
      url = urlparse(self.path)
      q = {k: v[0] for k, v in parse_qs(url.query).items()}
      try:
        if url.path in ("/", "/index.html"):
          with open(os.path.join(STATIC, "index.html"), "rb") as f:
            self._send(200, f.read(), "text/html; charset=utf-8")
        elif url.path == "/api/routes":
          sim = sys.modules.get("openpilot.sunnypilot.tools.drive_sim.sim") or reload_code()
          self._json(200, {"root": root, "routes": sim.list_routes(root)})
        elif url.path == "/api/sim":
          t0 = time.monotonic()
          sim = reload_code()
          data = sim.extract(root, q["route"], q.get("segs") or None)
          t1 = time.monotonic()
          if q.get("mode") == "closed":
            out = sim.simulate_closed_loop(data, proceed_wait_s=float(q.get("wait", 2.0)))
          elif q.get("mode") == "turns":
            out = sim.simulate_turns(data)
          elif q.get("mode") == "style":
            out = sim.simulate_style(data, profile=q.get("profile") or None)
          elif q.get("mode") == "narrow":
            out = sim.simulate_narrow(data, root)
          else:
            out = sim.simulate(data)
          out["timing"] = {"extract_s": round(t1 - t0, 2), "simulate_s": round(time.monotonic() - t1, 2)}
          self._json(200, out)
        else:
          self._send(404, b"not found", "text/plain")
      except Exception as e:
        traceback.print_exc()
        self._json(500, {"error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-3000:]})

  return Handler


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--root", default=os.path.expanduser("~/Fisker/routes"), help="folder holding the routes' segment folders")
  ap.add_argument("--port", type=int, default=8765)
  args = ap.parse_args()
  server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(os.path.abspath(args.root)))
  print(f"drive_sim: http://127.0.0.1:{args.port}  (routes from {args.root})", flush=True)
  try:
    server.serve_forever()
  except KeyboardInterrupt:
    pass
  return 0


if __name__ == "__main__":
  sys.exit(main())
