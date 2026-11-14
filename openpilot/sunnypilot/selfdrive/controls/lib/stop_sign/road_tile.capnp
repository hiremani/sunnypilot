# The layout of sunnypilot mapd's offline road tiles (pfeiferj/mapd `offline.capnp`), rebuilt from the byte offsets
# the webhud page's reader uses on real tiles (oceanscape static/js/world/osmtile.js decodeTile). Field order below
# is chosen so Cap'n Proto lays the fields out at exactly those offsets; tests/test_stop_sign.py checks it.
# Only the layout matters to a reader, not the names.
@0xe709f8eb910f1a10;

struct Coordinates {
  latitude @0 :Float64;    # data byte 0
  longitude @1 :Float64;   # data byte 8
}

struct Way {
  name @0 :Text;                     # pointer 0
  ref @1 :Text;                      # pointer 1
  maxSpeed @2 :Float64;              # byte 0
  minLat @3 :Float64;                # byte 8
  minLon @4 :Float64;                # byte 16
  maxLat @5 :Float64;                # byte 24
  maxLon @6 :Float64;                # byte 32
  nodes @7 :List(Coordinates);       # pointer 2, in OSM node order
  lanes @8 :UInt8;                   # byte 40
  highway @9 :UInt16;                # byte 42 (osmtile.js HIGHWAY_CLASSES)
  oneWay @10 :Bool;                  # bit 328 (byte 41)
  advisorySpeed @11 :Float64;        # byte 48
  maxSpeedForward @12 :Float64;      # byte 56
  maxSpeedBackward @13 :Float64;     # byte 64
  hazard @14 :Text;                  # pointer 3
  id @15 :Int64;                     # byte 72
}

struct Offline {
  minLat @0 :Float64;
  minLon @1 :Float64;
  maxLat @2 :Float64;
  maxLon @3 :Float64;
  overlap @4 :Float64;
  ways @5 :List(Way);                # pointer 0
}
