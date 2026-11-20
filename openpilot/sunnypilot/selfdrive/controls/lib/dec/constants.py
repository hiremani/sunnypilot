class WMACConstants:
  # Lead detection parameters
  LEAD_WINDOW_SIZE = 6  # Stable detection window
  LEAD_PROB = 0.45  # Balanced threshold for lead detection

  # Slow down detection parameters
  SLOW_DOWN_WINDOW_SIZE = 5  # Responsive but stable
  SLOW_DOWN_PROB = 0.3  # Balanced threshold for slow down scenarios

  # Optimized slow down distance curve - smooth and progressive
  SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55., 60.]
  SLOW_DOWN_DIST = [32., 46., 64., 86., 108., 130., 145., 165.]

  # Slowness detection parameters
  SLOWNESS_WINDOW_SIZE = 10  # Stable slowness detection
  SLOWNESS_PROB = 0.55  # Clear threshold for slowness
  SLOWNESS_CRUISE_OFFSET = 1.025  # Conservative cruise speed offset

  # Stop cue (Fisker, 2026-11-20): the slow-down urgency only counts once the model's plan shows a stop, not just a
  # slowdown: its slowest planned speed under STOP_CUE_RATIO * v_ego (or STOP_CUE_FLOOR m/s), for STOP_CUE_FRAMES model
  # frames in a row. Warm Springs Blvd (route 0000010d, 2026-11-17): the plan slowed from 40 to 16 mph (40%) with no
  # lead, and DEC handed the speed to the model. Replayed over 6.6 h of the driver's drives (dd..11b): hand-overs with
  # no slowdown needed 7.3 -> 2.4 min; no-lead stops from 15+ mph caught 39 -> 37 of 41 (at least 3 s before 5 mph:
  # 21 -> 16). A ratio of 0.7 kept every stop but let a 40% slowdown through.
  STOP_CUE_RATIO = 0.3
  STOP_CUE_FLOOR = 3.0      # m/s
  STOP_CUE_FRAMES = 20      # model frames (1 s)
