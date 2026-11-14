import http.client
import json
import os
import re
import threading
import time
from unittest import mock

from opendbc.car.structs import car

from openpilot.common import params as params_module
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.remote_control import remote_control_server as rcs
from openpilot.system.manager.process_config import managed_processes


class FakeParams:
  def __init__(self, store=None):
    self.store = dict(store or {})

  def get(self, key, return_default=False):
    return self.store.get(key)

  def put(self, key, value):
    self.store[key] = value.encode() if isinstance(value, str) else value

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def put_bool(self, key, value):
    self.store[key] = bool(value)


class TestClientAllowed(OpenpilotTestCase):
  def test_local_addresses(self):
    for addr in ("127.0.0.1", "10.0.0.7", "172.16.4.1", "172.31.255.254", "192.168.1.20", "169.254.10.3",
                 "::1", "fe80::1c2b:3dff:fe4e:5f60", "fe80::1%wlan0", "fd12:3456:789a::1", "::ffff:192.168.1.20"):
      assert rcs.client_allowed(addr), addr

  def test_public_and_bad_addresses(self):
    for addr in ("8.8.8.8", "172.32.0.1", "192.169.0.1", "100.64.0.1", "1.1.1.1", "2001:4860:4860::8888",
                 "::ffff:8.8.8.8", "", "not-an-address", "192.168.1"):
      assert not rcs.client_allowed(addr), addr


class TestRemoteControlServer(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = FakeParams({"RemoteControlEnabled": True})
    self.saved = (rcs.Handler.params, rcs.Handler.armed)
    rcs.Handler.params = self.params
    rcs.Handler.armed = False
    self.server = rcs.ThreadingServer(("127.0.0.1", 0), rcs.Handler)
    self.port = self.server.server_address[1]
    threading.Thread(target=self.server.serve_forever, daemon=True).start()

  def tearDown(self):
    self.server.shutdown()
    self.server.server_close()
    rcs.Handler.params, rcs.Handler.armed = self.saved
    super().tearDown()

  def request(self, method, path, body=None, headers=None):
    conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
    data = json.dumps(body).encode() if body is not None else None
    conn.request(method, path, body=data, headers={"Content-Type": "application/json", **(headers or {})})
    res = conn.getresponse()
    raw = res.read()
    conn.close()
    return res.status, (json.loads(raw) if raw and res.getheader("Content-Type") == "application/json" else raw), res

  def test_steer_needs_arm(self):
    status, body, _ = self.request("POST", "/steer", {"angle": 10})
    assert status == 409 and body == {"error": "not armed"}
    assert "RemoteSteerAngle" not in self.params.store and "RemoteSteerLastUpdate" not in self.params.store
    assert self.request("GET", "/status")[1]["armed"] is False

    assert self.request("POST", "/arm", {})[0] == 200
    assert self.request("GET", "/status")[1]["armed"] is True
    before = time.time_ns()
    status, body, _ = self.request("POST", "/steer", {"angle": 10})
    assert status == 200 and body == {"ok": True, "angle": 10.0}
    assert float(self.params.get("RemoteSteerAngle")) == -10.0   # stored left-positive
    assert int(self.params.get("RemoteSteerLastUpdate")) >= before

  def test_angle_cap(self):
    self.request("POST", "/arm", {})
    assert self.request("POST", "/steer", {"angle": 9999})[1]["angle"] == rcs.MAX_ANGLE_DEG
    assert float(self.params.get("RemoteSteerAngle")) == -rcs.MAX_ANGLE_DEG
    assert self.request("POST", "/steer", {"angle": -9999})[1]["angle"] == -rcs.MAX_ANGLE_DEG
    assert self.request("POST", "/steer", {"angle": "left"})[0] == 400

  def test_disarm_makes_the_command_stale_and_keeps_the_opt_in(self):
    self.request("POST", "/arm", {})
    self.request("POST", "/steer", {"angle": 25})
    assert self.request("POST", "/disarm", {})[0] == 200
    assert float(self.params.get("RemoteSteerAngle")) == 0.0
    assert int(self.params.get("RemoteSteerLastUpdate")) == 0   # older than any freshness window
    assert self.request("POST", "/steer", {"angle": 25})[0] == 409
    # the process-level opt-in is the owner's: the server never writes it
    assert self.params.store["RemoteControlEnabled"] is True

  def test_non_local_client_refused(self):
    self.request("POST", "/arm", {})
    stored = dict(self.params.store)
    with mock.patch.object(rcs.Handler, "_client_ip", return_value="8.8.8.8"):
      for method, path, body in (("GET", "/", None), ("GET", "/status", None), ("OPTIONS", "/steer", None),
                                 ("POST", "/arm", {}), ("POST", "/steer", {"angle": 5}), ("POST", "/disarm", {})):
        status, _, _ = self.request(method, path, body)
        assert status == 403, (method, path)
    assert self.params.store == stored and rcs.Handler.armed
    with mock.patch.object(rcs.Handler, "_client_ip", return_value="192.168.43.17"):
      assert self.request("POST", "/steer", {"angle": 5})[0] == 200

  def test_cross_origin_post_refused(self):
    host = f"127.0.0.1:{self.port}"
    status, body, res = self.request("POST", "/arm", {}, headers={"Origin": "http://evil.example"})
    assert status == 403 and body == {"error": "cross-origin request"} and not rcs.Handler.armed
    assert res.getheader("Access-Control-Allow-Origin") is None
    assert self.request("POST", "/arm", {}, headers={"Origin": f"http://{host}"})[0] == 200
    assert rcs.Handler.armed

  def test_main_starts_disarmed_without_touching_the_opt_in(self):
    rcs.Handler.armed = True
    params = FakeParams({"RemoteControlEnabled": True, "RemoteSteerAngle": b"40", "RemoteSteerLastUpdate": b"123"})

    class NoServe:
      def __init__(self, *args):
        pass

      def __enter__(self):
        return self

      def __exit__(self, *args):
        return False

      def serve_forever(self):
        pass

    with mock.patch.object(rcs, "Params", return_value=params), mock.patch.object(rcs, "ThreadingServer", NoServe):
      rcs.main()
    assert rcs.Handler.armed is False
    assert params.store == {"RemoteControlEnabled": True, "RemoteSteerAngle": b"0", "RemoteSteerLastUpdate": b"0"}


class TestProcessGating(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.proc = managed_processes["remote_control_server"]
    self.car = car.CarParams.new_message()
    self.body = car.CarParams.new_message(notCar=True)

  def test_off_by_default(self):
    params = FakeParams()
    assert not self.proc.should_run(True, params, self.car)
    assert not self.proc.should_run(False, params, self.car)

  def test_runs_only_with_the_param_in_a_started_car(self):
    params = FakeParams({"RemoteControlEnabled": True})
    assert self.proc.enabled
    assert self.proc.should_run(True, params, self.car)
    assert not self.proc.should_run(False, params, self.car)   # offroad
    assert not self.proc.should_run(True, params, self.body)   # not a car
    params.put_bool("RemoteControlEnabled", False)
    assert not self.proc.should_run(True, params, self.car)

  def test_param_is_cleared_at_manager_start(self):
    # the opt-in must not survive a boot
    keys_h = os.path.join(os.path.dirname(params_module.__file__), "params_keys.h")
    with open(keys_h) as f:
      decl = re.search(r'\{"RemoteControlEnabled", \{([^}]*)\}\}', f.read())
    assert decl is not None
    flags, typ, default = (part.strip() for part in decl.group(1).split(","))
    assert flags == "CLEAR_ON_MANAGER_START" and typ == "BOOL" and default == '"0"'
