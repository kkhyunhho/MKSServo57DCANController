# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Conventions

For the big picture and all shared conventions — the "one cell, many
devices" architecture, code style, repo skeleton, codename naming, the
shared conda env, testing, and task/commit rules — see **SDLClaude**
(`kkhyunhho/SDLClaude`), the single source of truth. Where this file is
silent, SDLClaude governs.

(The local [`SDLClaude.md`](SDLClaude.md) here is the **historical
origin** of that ruleset, kept for reference; the canonical, maintained
copy is `kkhyunhho/SDLClaude`.)

## What this project is

A **standalone single-axis MKS SERVO57D CAN test harness** — the original
MKSMotor test bed. It drives the motor over a USB2CAN (FTDI) adapter with
**no ESP32 firmware and no touch UI**, which is exactly why it is kept:
for bench situations where the full stack is unwanted — e.g. testing a
single axis, or quick driver checks without the ESP32-S3-BOX-3.

- Driver: [src/mks_motor/](src/mks_motor/), class `MKSMotor` (simpler than
  the full driver). Bench scripts `running_test_z.py` / `running_test_xz.py`
  at the repo root import `from mks_motor import MKSMotor`.
- `mks_motor_oldver.py` is an older backup, kept for history; not imported.
- Scope: **L0 (driver) only** — no L1 server, no L2 ESP UI. (Level = code
  depth; see SDLClaude's `ARCHITECTURE.md`.)

## Relationship to ESP32S3BOX3MotorController

That project holds the **full, more correct** MKS driver (paired-axis
safety interlock, better limit logic, the ESP32 firmware bridge). This
harness's limit logic **lags** it and should be **aligned to the ESP32
version later** (the two are intended to converge).

**Import-name collision:** both expose the package `mks_motor`, so they are
**not co-installed** in the shared `elec` env (where the ESP32 driver
lives). Run this harness standalone — from an isolated env, or with
`PYTHONPATH=src python running_test_z.py`. Its distribution name is
`mks_motor_standalone` to keep that distinction explicit.

## Hardware notes

MKS SERVO57D over a USB2CAN **FTDI** adapter (`0403:6001`). This older
harness uses the **`ftd2xx`** D2XX bindings (not pyftdi — that is the full
ESP32 driver). Adapters are identified by **FTDI serial number** (FTDI
exposes unique serials). On Linux, D2XX needs the in-kernel `ftdi_sio`
driver detached for the FTDI device; the container's `/dev` is a private
tmpfs so USB nodes also go stale after re-enumeration — see [README.md](README.md).
