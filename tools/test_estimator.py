#!/usr/bin/env python3
"""Checks on the attitude-estimator and heading-hold math.

There is no host C++ compiler in this project (only the xtensa cross
toolchain), so this models the arithmetic the firmware performs and asserts
the properties that matter, then greps the source to make sure the firmware
still expresses that arithmetic.  Run it after touching angle_calc() or the
heading loop:

    python tools/test_estimator.py
"""
import pathlib
import re
import sys

SRC = pathlib.Path(__file__).resolve().parent.parent / "esp32_cube_enc"

GYRO_LSB_PER_DPS = 131.0   # +/-250 deg/s range (gyroSens 0)
LOOP_MS = 15
YAW_RATE_MAX = 25.0        # YAW_TURN_RATE: the rate cap the heading loop uses


def old_integrate(gy_raw, loop_ms=LOOP_MS):
    """The original line: GyY * loop_time / 1000 / 65.536.

    GyY and loop_time are both C ints, so the /1000 truncates toward zero
    before the float divide ever runs.
    """
    return int(gy_raw * loop_ms / 1000) / 65.536


def new_integrate(gy_raw, dt):
    """The current line: GyY * dt / GYRO_LSB_PER_DPS, all float."""
    return gy_raw * dt / GYRO_LSB_PER_DPS


def test_old_had_a_dead_zone():
    """Every rate under ~0.5 deg/s used to integrate to exactly zero."""
    for dps in (0.05, 0.1, 0.25, 0.4, 0.5):
        raw = dps * GYRO_LSB_PER_DPS
        assert old_integrate(raw) == 0.0, f"expected dead zone at {dps} deg/s"
    # First rate that escaped it, for the record: 67 counts.
    assert old_integrate(66) == 0.0
    assert old_integrate(67) != 0.0
    assert abs(67 / GYRO_LSB_PER_DPS - 0.51) < 0.01


def test_new_has_no_dead_zone():
    dt = LOOP_MS / 1000.0
    for dps in (0.01, 0.05, 0.1, 0.25, 0.4, 0.5):
        raw = dps * GYRO_LSB_PER_DPS
        got = new_integrate(raw, dt)
        assert got > 0.0, f"{dps} deg/s still integrates to zero"
        # And it is the right size: rate * time.
        assert abs(got - dps * dt) < 1e-9


def test_new_scale_is_correct():
    """Integrating a known rate for a known time gives that many degrees."""
    dt = 0.001
    total = sum(new_integrate(90.0 * GYRO_LSB_PER_DPS, dt) for _ in range(1000))
    assert abs(total - 90.0) < 1e-6, f"90 deg/s for 1 s gave {total} deg"

    # The old formula was built around 65.536 counts/deg/s, the +/-500 deg/s
    # figure, so at the configured +/-250 range it ran at twice the scale the
    # controller's own GyX / 131.0 rate terms assumed.  Compare a single step
    # at a raw value the truncation divides exactly (200 * 15 = 3000), so this
    # isolates the scale error from the dead zone tested above.
    raw = 200
    old_step = old_integrate(raw, LOOP_MS)
    new_step = new_integrate(raw, LOOP_MS / 1000.0)
    assert old_integrate(raw, LOOP_MS) * 65.536 == 3.0, "pick an exact raw value"
    assert abs(old_step / new_step - 2.0) < 0.01, \
        f"expected the old 2x scale error, got {old_step / new_step:.3f}x"


def test_measured_dt_beats_assumed_dt():
    """An iteration that overruns must not lose the angle it turned through."""
    rate_raw = 20.0 * GYRO_LSB_PER_DPS          # 20 deg/s
    actual_ms = 40                              # a 40 ms iteration (HTTP stall)
    truth = 20.0 * actual_ms / 1000.0
    assumed = new_integrate(rate_raw, LOOP_MS / 1000.0)   # old behaviour
    measured = new_integrate(rate_raw, actual_ms / 1000.0)
    assert abs(measured - truth) < 1e-9
    assert abs(assumed - truth) > 0.4, "assumed-dt error should be large here"


def test_dt_clamp_bounds_a_stall():
    """The clamp trades a small known error for an unbounded unknown one."""
    lo, hi = 0.005, 0.045
    for gap_ms, expect in ((0, lo), (15, 0.015), (40, 0.040), (3000, hi)):
        dt = min(max(gap_ms / 1000.0, lo), hi)
        assert abs(dt - expect) < 1e-9


# --- heading hold -----------------------------------------------------


def yaw_rate_cmd(target, yaw, zk1=1.0):
    return max(-YAW_RATE_MAX, min(YAW_RATE_MAX, zk1 * (target - yaw)))


def simulate_turn(deg, zk1=1.0, dt=0.015, steps=4000):
    """Closed loop: the rate command is tracked perfectly by the inner loop."""
    yaw = 0.0
    target = yaw + deg                    # relative, as the firmware does it
    peak = 0.0
    for _ in range(steps):
        rate = yaw_rate_cmd(target, yaw, zk1)
        peak = max(peak, abs(rate))
        yaw += rate * dt
    return yaw, peak


def test_turn_is_relative_and_converges():
    for deg in (90, -90, 180, -45):
        yaw, _ = simulate_turn(deg)
        assert abs(yaw - deg) < 0.5, f"turn {deg} settled at {yaw}"


def test_turn_never_exceeds_the_rate_clamp():
    _, peak = simulate_turn(720)
    assert peak <= YAW_RATE_MAX + 1e-9, f"commanded {peak} deg/s"


def test_no_angle_wrapping():
    """A 720 turn is two full revolutions, not a no-op."""
    yaw, _ = simulate_turn(720)
    assert abs(yaw - 720) < 1.0, f"expected 720, got {yaw}"


def test_hold_is_a_zero_degree_turn():
    yaw, peak = simulate_turn(0)
    assert yaw == 0.0 and peak == 0.0


def test_zk1_zero_disables_the_loop():
    yaw, peak = simulate_turn(90, zk1=0.0)
    assert yaw == 0.0 and peak == 0.0, "zK1=0 must command no rotation"


# --- the firmware still does what is modelled above -------------------


def test_source_has_no_integer_truncation():
    src = (SRC / "functions.cpp").read_text(encoding="utf-8", errors="replace")
    lines = [l for l in src.splitlines()
             if "robot_angle" in l and "+=" in l]
    assert len(lines) == 2, f"expected 2 integration lines, found {len(lines)}"
    for line in lines:
        assert "GYRO_LSB_PER_DPS" in line, line
        assert "dt" in line, line
        assert "65.536" not in line, "the +/-500 deg/s divisor is back: " + line
        assert "/ 1000" not in line, "integer truncation is back: " + line


def test_no_hardcoded_gyro_scale_anywhere():
    """Both halves of the firmware must agree on counts per deg/s."""
    for name in ("functions.cpp", "esp32_cube_enc.cpp"):
        src = (SRC / name).read_text(encoding="utf-8", errors="replace")
        for n, line in enumerate(src.splitlines(), 1):
            if line.lstrip().startswith("//"):
                continue
            assert not re.search(r"/\s*131\.0", line), f"{name}:{n} {line.strip()}"
            assert not re.search(r"/\s*65\.536", line), f"{name}:{n} {line.strip()}"


def test_gain_table_appends_only():
    """loadGains() matches saved values by position, so zK1 must be last."""
    src = (SRC / "web_interface.cpp").read_text(encoding="utf-8", errors="replace")
    names = re.findall(r'\{"(\w+)",\s*&\w+,', src)
    assert names[:11] == ["K1", "K2", "K3", "K4", "zK2", "zK3",
                          "eK1", "eK2", "eK3", "eK4", "tK"], names
    assert names[11] == "zK1", "new gains must be appended, not inserted"
    hdr = (SRC / "ESP32.h").read_text(encoding="utf-8", errors="replace")
    declared = int(re.search(r"#define NUM_GAINS\s+(\d+)", hdr).group(1))
    assert declared == len(names), f"NUM_GAINS {declared} != {len(names)} rows"


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ok   {name}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL {name}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
