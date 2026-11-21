"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Personal model: a small model on top of the frozen driving model that learns how this driver differs from it.

The driving model (modelV2) plans the car's speed the way an average driver would. Every rlog records that plan on
every frame, also while the driver was driving, so the logs hold pairs of (what the model planned, what the driver
did). The personal model learns the difference from the moments the driver chose (cruise off, openpilot not in control
of speed). It never replaces the driving model: it outputs a small, clamped correction.

  features.py  the inputs, computed the same way offline (from logs) and on the car (from live messages)
  model.py     the model: standardise, a small MLP, clamp; save/load as JSON; numpy only
Training and evaluation: sunnypilot/tools/personal_dataset.py, sunnypilot/tools/personal_train.py (run on the Mac).
"""
