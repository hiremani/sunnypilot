---
name: secoc-key
description: Diagnose and recover the Fisker Ocean's SecOC key when steering (or openpilot speed control) does nothing even though lateral shows active. Use when the user says steering stopped working or the car ignores commands.
---

# SecOC key

The EPS and VCU only accept 0x1D0 / 0x121 frames with a valid AES-CMAC (SecOC). card signs them with the car's key; with a wrong key the car controller sends nothing even though `latActive` is True.

1. Check the drive's logMessages for "SecOC key": `SecOC key mismatch` = wrong stored key; `SecOC key verified` = fine. A live SubMaster read of `carParams` right after boot can show defaults; trust the route's rlog carParams and the log message.
2. Recover: move the stored key aside on the device (both `/data/params/d/SecOCKey` and `/cache/params/SecOCKey`, keep a dated backup in `/data/`). At the next start card reads the key from the radar (UDS DID EFF5) and logs "key verified".
3. Never print, copy off the device, or commit the key. It is per car.

Freshness comes from the gateway's 0x20 sync (trip + reset counters); the car controller mirrors OEM counters + 1, and panda substitutes the candidate whose counter the EPS expects next.
