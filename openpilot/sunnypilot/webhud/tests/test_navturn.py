import types
import unittest
from unittest import mock

from openpilot.sunnypilot.webhud.navturn import parse

NOW = 500 * 10**9


def test_turn():
  f = parse({"type": "routeTurn", "direction": "left", "distance": 120, "tolerance": 5, "age": 1.5, "step": "Turn left onto A61",
             "app": "com.google.android.apps.maps"}, NOW)
  assert f["direction"] == "turnLeft"
  assert f["distance"] == 120.0 and f["distanceTolerance"] == 5.0
  assert f["observedMonoTime"] == NOW - int(1.5e9)
  assert f["turnId"] == parse({"direction": "right", "distance": 3, "step": "Turn left onto A61"}, NOW)["turnId"]
  assert f["turnId"] != parse({"direction": "left", "distance": 3, "step": "Turn left onto B6161"}, NOW)["turnId"]


def test_no_turn():
  for direction in (None, "straight", "uturn"):
    f = parse({"direction": direction, "distance": 400}, NOW)
    assert f["direction"] == "none" and f["distance"] == 0.0


def test_nonsense_refused():
  assert parse({"direction": "left", "distance": "far"}, NOW) is None
  assert parse({"direction": "left", "distance": -5}, NOW) is None
  assert parse({"direction": "left", "distance": 1e6}, NOW) is None


def test_age_kept_sane():
  assert parse({"direction": "left", "distance": 50, "age": -3}, NOW)["observedMonoTime"] == NOW
  assert parse({"direction": "left", "distance": 50, "age": 1e9}, NOW)["observedMonoTime"] == NOW - 120 * 10**9


class TestWebSocketRouteTurn(unittest.TestCase):
  # OceanScape sends the turn over its WebSocket (infotainment.js _sendRouteTurn), not only POST /api/route_turn
  def _send(self, peer):
    from openpilot.sunnypilot.webhud import server
    client = types.SimpleNamespace(peer=peer, push=lambda _: None)
    msg = '{"type": "routeTurn", "direction": "left", "distance": 120, "step": "Turn left"}'
    with mock.patch.object(server.NAV_TURN, "publish", return_value=True) as publish:
      server.HudHandler._ws_message(types.SimpleNamespace(server=None), client, msg)
    return publish

  def test_local_peer_publishes(self):
    publish = self._send("192.168.0.50")
    publish.assert_called_once()
    assert publish.call_args[0][0]["direction"] == "left"

  def test_remote_peer_ignored(self):
    self._send("8.8.8.8").assert_not_called()
    self._send("not an address").assert_not_called()
