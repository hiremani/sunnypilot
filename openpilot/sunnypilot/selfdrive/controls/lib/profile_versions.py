"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Versions of the driver profile and the personal model: every install keeps a copy of what it replaced, so any of the
last KEEP versions can be put back (from the web HUD, or this file's command line) and compared on the Mac
(tools/profile_compare.py).

Two kinds of file, both under the profiles dir (driver_profile.profiles_dir()):
  profile   <dir>/<name>.json            the driver profile (driver_profile.py)
  personal  <dir>/personal/<name>.json   the personal model (personal/, on the personal_model branch)
Versions live in <dir>/.versions/<kind>/<name>/<id>.json with an index.json holding each one's label, source and
time. An id is <UTC yyyymmdd-HHMMSS>-<first 8 of the file's sha256>; a version is stored once per content (installing
the same file again only refreshes nothing). The live file is replaced atomically (temp file + rename), so the
planner, which re-reads it every few seconds, never sees half a file.

Pruning keeps the KEEP newest plus whatever is live now.

  python -m openpilot.sunnypilot.selfdrive.controls.lib.profile_versions list [--kind K] [--name N]
  ... install FILE [--kind K] [--name N] [--label TEXT]
  ... restore ID [--kind K] [--name N]
  ... snapshot [--kind K] [--name N] [--label TEXT]
  ... import-backups [--name N]      (the older .<name>.json.bak-* copies)
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import tempfile
import time

from openpilot.sunnypilot.selfdrive.controls.lib import driver_profile as dp

KINDS = ("profile", "personal")
KEEP = 10
ID_RE = re.compile(r"^\d{8}-\d{6}-[0-9a-f]{8}$")
INDEX = "index.json"


def live_path(kind: str, name: str) -> str:
  if kind not in KINDS:
    raise ValueError(f"bad kind: {kind!r}")
  if kind == "profile":
    return dp.profile_path(name)          # validates the name
  if not dp.valid_name(name):
    raise ValueError(f"bad name: {name!r}")
  return os.path.join(dp.profiles_dir(), "personal", name + ".json")


def versions_dir(kind: str, name: str) -> str:
  live_path(kind, name)                   # validates kind and name
  return os.path.join(dp.profiles_dir(), ".versions", kind, name)


def sha_of(path: str) -> str | None:
  try:
    with open(path, "rb") as f:
      return hashlib.sha256(f.read()).hexdigest()
  except OSError:
    return None


def live_sha(kind: str, name: str) -> str | None:
  try:
    return sha_of(live_path(kind, name))
  except ValueError:
    return None


def valid_content(kind: str, path: str) -> bool:
  """A profile must be one driver_profile.py reads; a personal model must at least be a JSON object."""
  if kind == "profile":
    return dp.read_file(path) is not None
  try:
    with open(path) as f:
      return isinstance(json.load(f), dict)
  except (OSError, ValueError):
    return False


def _read_index(d: str) -> dict:
  try:
    with open(os.path.join(d, INDEX)) as f:
      idx = json.load(f)
    return idx if isinstance(idx, dict) else {}
  except (OSError, ValueError):
    return {}


def _atomic_copy(src: str, dst: str) -> None:
  os.makedirs(os.path.dirname(dst), exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dst), prefix=".tmp-")
  try:
    with os.fdopen(fd, "wb") as out, open(src, "rb") as inp:
      shutil.copyfileobj(inp, out)
      out.flush()
      os.fsync(out.fileno())
    os.replace(tmp, dst)
  except BaseException:
    try:
      os.unlink(tmp)
    except OSError:
      pass
    raise


def _write_index(d: str, idx: dict) -> None:
  fd, tmp = tempfile.mkstemp(dir=d, prefix=".tmp-")
  with os.fdopen(fd, "w") as f:
    json.dump(idx, f, indent=1, sort_keys=True)
  os.replace(tmp, os.path.join(d, INDEX))


def list_versions(kind: str, name: str) -> list[dict]:
  """Newest first: {id, created, label, source, sha, live}. Files missing from the index (or index entries without a
  file) are dropped from the list, not errors."""
  d = versions_dir(kind, name)
  idx = _read_index(d)
  cur = live_sha(kind, name)
  out = []
  try:
    files = [f[:-5] for f in os.listdir(d) if f.endswith(".json") and f != INDEX and ID_RE.match(f[:-5])]
  except OSError:
    return out
  for vid in files:
    m = idx.get(vid, {})
    sha = m.get("sha") or sha_of(os.path.join(d, vid + ".json"))
    out.append({"id": vid, "created": m.get("created", 0.0), "label": m.get("label", ""), "source": m.get("source", ""),
                "sha": sha, "live": sha is not None and sha == cur})
  out.sort(key=lambda v: (v["created"], v["id"]), reverse=True)
  return out


def version_path(kind: str, name: str, vid: str) -> str:
  if not isinstance(vid, str) or not ID_RE.match(vid):
    raise ValueError(f"bad version id: {vid!r}")
  p = os.path.join(versions_dir(kind, name), vid + ".json")
  if not os.path.isfile(p):
    raise ValueError(f"no such version: {vid}")
  return p


def _store(kind: str, name: str, src: str, label: str, source: str, created: float | None = None) -> dict | None:
  """Keep a copy of src as a version (once per content); the entry, or None when src can't be read."""
  sha = sha_of(src)
  if sha is None:
    return None
  d = versions_dir(kind, name)
  os.makedirs(d, exist_ok=True)
  for v in list_versions(kind, name):
    if v["sha"] == sha:
      return v
  created = time.time() if created is None else created   # noqa: TID251  a wall-clock date to show, not a timer
  vid = time.strftime("%Y%m%d-%H%M%S", time.gmtime(created)) + "-" + sha[:8]
  _atomic_copy(src, os.path.join(d, vid + ".json"))
  idx = _read_index(d)
  idx[vid] = {"created": round(created, 3), "label": str(label)[:80], "source": str(source)[:40], "sha": sha}
  _write_index(d, idx)
  prune(kind, name)
  return {"id": vid, **idx[vid], "live": sha == live_sha(kind, name)}


def prune(kind: str, name: str, keep: int = KEEP) -> list[str]:
  """Drop all but the keep newest versions and the live one; the ids dropped."""
  d = versions_dir(kind, name)
  vs = list_versions(kind, name)
  drop = [v["id"] for v in vs[keep:] if not v["live"]]
  if drop:
    idx = _read_index(d)
    for vid in drop:
      try:
        os.unlink(os.path.join(d, vid + ".json"))
      except OSError:
        pass
      idx.pop(vid, None)
    _write_index(d, idx)
  return drop


def snapshot(kind: str, name: str, label: str = "", source: str = "snapshot") -> dict | None:
  """Keep the live file as a version; None when there is no live file."""
  p = live_path(kind, name)
  return _store(kind, name, p, label, source) if os.path.isfile(p) else None


def install(kind: str, name: str, src: str, label: str = "", source: str = "install") -> dict:
  """Make src the live file: the live one is kept first, then src replaces it atomically and is kept too."""
  if not valid_content(kind, src):
    raise ValueError(f"{src} is not a readable {kind} file")
  snapshot(kind, name, "before " + (label or os.path.basename(src)), "auto")
  dst = live_path(kind, name)
  _atomic_copy(src, dst)
  return _store(kind, name, dst, label or os.path.basename(src), source)


def restore(kind: str, name: str, vid: str) -> dict:
  """Put version vid back live (the live file is kept first, if it isn't a version already)."""
  p = version_path(kind, name, vid)
  if not valid_content(kind, p):
    raise ValueError(f"version {vid} is not a readable {kind} file")
  snapshot(kind, name, "before restoring " + vid, "auto")
  _atomic_copy(p, live_path(kind, name))
  return next(v for v in list_versions(kind, name) if v["id"] == vid)


def import_backups(name: str = dp.DEFAULT_NAME) -> list[dict]:
  """The hand-made .<name>.json.bak-<tag> copies next to the profile, as versions (dated by their mtime)."""
  out = []
  d = dp.profiles_dir()
  try:
    files = sorted(f for f in os.listdir(d) if f.startswith(f".{name}.json.bak"))
  except OSError:
    return out
  for f in files:
    p = os.path.join(d, f)
    if valid_content("profile", p):
      e = _store("profile", name, p, f[len(name) + 7:].lstrip("-.") or "backup", "backup", os.path.getmtime(p))
      if e:
        out.append(e)
  return out


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("cmd", choices=("list", "install", "restore", "snapshot", "import-backups"))
  ap.add_argument("arg", nargs="?", help="FILE for install, ID for restore")
  ap.add_argument("--kind", default="profile", choices=KINDS)
  ap.add_argument("--name", default=dp.DEFAULT_NAME)
  ap.add_argument("--label", default="")
  a = ap.parse_args(argv)
  if a.cmd == "install":
    print(json.dumps(install(a.kind, a.name, a.arg, a.label)))
  elif a.cmd == "restore":
    print(json.dumps(restore(a.kind, a.name, a.arg)))
  elif a.cmd == "snapshot":
    print(json.dumps(snapshot(a.kind, a.name, a.label)))
  elif a.cmd == "import-backups":
    print(json.dumps(import_backups(a.name)))
  for v in list_versions(a.kind, a.name):
    when = time.strftime("%Y-%m-%d %H:%M", time.localtime(v["created"])) if v["created"] else "?"
    print(f"{'*' if v['live'] else ' '} {v['id']}  {when}  {v['source']:8s} {v['label']}")


if __name__ == "__main__":
  main()
