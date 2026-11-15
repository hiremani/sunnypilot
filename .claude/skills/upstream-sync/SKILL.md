---
name: upstream-sync
description: Keep the user's GitHub forks (hiremani/sunnypilot, hiremani/opendbc) up to date with the original benkayuk repositories and push the user's branch. Use when asked to get updates from the original repo, push work, or check fork status.
---

# Forks and upstream

| Repo | origin (push here) | upstream (updates from) | branch |
|---|---|---|---|
| `~/Fisker/sunnypilot` | github.com/hiremani/sunnypilot | github.com/benkayuk/sunnypilot | `mallik-fisker` |
| `opendbc_repo` | github.com/hiremani/opendbc | github.com/benkayuk/opendbc | `mallik-fisker` |
| `panda` | benkayuk/panda (no local changes, no fork) | - | - |

`.gitmodules` points opendbc at `hiremani/opendbc` so a fresh clone can resolve the pinned car commits. Both forks are public.

## Pull updates

```
cd ~/Fisker/sunnypilot && git fetch upstream && git merge upstream/master
cd opendbc_repo && git fetch upstream && git merge upstream/master && git push && cd ..
git add opendbc_repo && git commit -m "bump opendbc" && git push
```
- If upstream changes the opendbc line in `.gitmodules`, keep the hiremani URL.
- Conflicts in `opendbc/car/fisker/*` or the turn/stop/profile modules: those are the user's work; resolve keeping both behaviors and run `fisker-tests`.
- After merging, deploy only with the user's go-ahead (`comma-deploy`).

## Pushing

Commit or push only when the user asks. Pushes to the forks are public: scan first for keys or private files (`git diff upstream/master..HEAD | grep -iE '[0-9a-f]{32}|token|password'`; `local_stop_signs.json` and params are never in git).
