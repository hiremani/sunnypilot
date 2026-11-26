# Facts measured directly on the car's devices in this working session (2026-11-21/22)

Source: read over adb / ssh by the coordinating session; these are first-hand observations, not web or code claims.

- The Pulse board identifies itself over adb as product `rk3588s_t`, model `EVPlay`, board `rk30sdk`, Android 13, a
  userdebug build with `su` present. So the SoC is an RK3588S (confirms what the code audit could only infer).
- Its display is 1080 x 1920 (portrait), reported by Android's input reader for the touch device.
- The HUD app (Oceanscape 1.6.5, three.js WebGL) showed "49 fps" on its own on-screen counter in a screenshot taken while
  parked: top-down view, "Perforated" ground style, one ego car, no traffic. One reading, parked, not a benchmark.
- Four HUD apps were installed side by side on the Pulse; load average on the Pulse was about 6-8 while idle in the car.
- The comma in this car is a comma four: 4 CPU cores visible, 3.6 GB RAM total with about 1.8 GB free while parked,
  89 GB storage (90% full with drive logs).
- History relevant to compute planning: running 10 analysis jobs on the comma while driving once exhausted RAM and killed
  the driving model process; the working rule since is at most 1-2 low-priority jobs on the comma, heavy work on a Mac.
- The comma and the Pulse reach each other over the Pulse's Wi-Fi hotspot in the car; that hotspot isolates other
  clients (a laptop on it could not reach the comma). The HUD connected to the comma's bridge at port 8088.
- The Pulse's touch input comes over an I2C link that drops out when the display sleeps; the vendor touch service's
  recovery needs root it does not get. Relevant only as a reliability note for anything interactive on that screen.
- Only a forward-facing comma camera set (wide + narrow road, plus driver camera) exists; no side or rear cameras are
  connected to either device. The car's factory surround-view video is not tapped.
