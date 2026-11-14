#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.7"
fi

export STAGING_ROOT="/data/safe_staging"

# Fisker Ocean: FW_VERSIONS is intentionally empty (single-model port, CAN fingerprint
# is authoritative) and FW_QUERY_CONFIG has zero requests. Skip the VIN/FW query at
# launch so card comes up promptly on every launch route (systemd, tmux, manual).
# This is global, not Fisker-only: opendbc's car_helpers.fingerprint() reads it before
# the car is known (the VIN/FW query runs ahead of the CAN fingerprint), so on any other
# brand this fork has no VIN and no FW fingerprint. Remove it before using another car.
export SKIP_FW_QUERY=1
