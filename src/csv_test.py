"""
CSV (Cyclic Sync Velocity, mode 9) test for Y7.

Demonstrates cycle-rate target velocity updates via PDO instead of SDO.
The master computes a trapezoid profile and writes 0x60FF every bus cycle;
the drive simply tracks it. Telemetry (0x606C, 0x6077, 0x6041) is read
back from TxPDO each cycle.

SAFETY:
- Peak speed limited to TEST_TARGET_RPM (default 200 RPM)
- Total motion ~3 s (ramp-up, hold, ramp-down)
- Ctrl+C ramps to 0 before disabling
- Aborts on FAULT or loss of OPERATION_ENABLED
"""

import pysoem
import struct
import time

from config import (
    ECAT_ADAPTER_NAME, EC_CYCLE_TIME,
    BRAKE_OFF_DELAY, BRAKE_SPEED_LEVEL, BRAKE_ON_DELAY,
    BRAKE_RELEASE_SPEED_RPM, BRAKE_OFF_DELAY_MS, BRAKE_ON_DELAY_MS,
    SPEED_MAX_RPM,
    SW_READY_TO_SWITCH_ON, SW_SWITCHED_ON, SW_OPERATION_ENABLED, SW_FAULT,
    CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OP,
    MODE_CYCLIC_SYNC_VELOCITY,
)
from speed_test import VELOCITY_FACTOR

# Test profile (conservative — please review before raising)
TEST_TARGET_RPM = 200
RAMP_DURATION_S = 1.0
HOLD_DURATION_S = 1.0

# PDO assignment — see ESI for layout
#  RxPDO 0x1601: CW(2) + Mode(1) + TargetTorque(2) + TargetPos(4) + MaxSpeed(4) + TouchProbe(2) + TargetVel(4) = 19 B
#  TxPDO 0x1A01: ErrCode(2) + SW(2) + ModeDisp(1) + PosAct(4) + VelAct(4) + TorqueAct(2) + TPstat(2) + TPpos(4) + TPneg(4) + DigIn(4) = 29 B
RXPDO_INDEX = 0x1601
TXPDO_INDEX = 0x1A01
RXPDO_FMT = '<HbhiIHi'
TXPDO_FMT = '<HHbiihHiiI'

PRINT_EVERY_N_CYCLES = 25  # at 4 ms cycle → ~10 Hz console output


def find_adapter(desc_hint):
    for a in pysoem.find_adapters():
        if desc_hint.lower() in a.desc.decode().lower():
            return a.name
    return None


def make_pdo_config(slave):
    """Build the Pre-Op config callback. pysoem invokes config_func with
    the slave *index* (int), so we close over the slave object here."""
    def pdo_config(_slave_pos):
        slave.sdo_write(0x1C12, 0, struct.pack('B', 0))
        slave.sdo_write(0x1C12, 1, struct.pack('<H', RXPDO_INDEX))
        slave.sdo_write(0x1C12, 0, struct.pack('B', 1))
        slave.sdo_write(0x1C13, 0, struct.pack('B', 0))
        slave.sdo_write(0x1C13, 1, struct.pack('<H', TXPDO_INDEX))
        slave.sdo_write(0x1C13, 0, struct.pack('B', 1))
    return pdo_config


def build_rx(cw, target_vel_raw, max_speed_raw):
    return struct.pack(
        RXPDO_FMT,
        cw,
        MODE_CYCLIC_SYNC_VELOCITY,
        0,                # TargetTorque (unused)
        0,                # TargetPos (unused)
        max_speed_raw,
        0,                # TouchProbe (unused)
        target_vel_raw,
    )


def parse_tx(data):
    err, sw, mode_disp, _pos, vel_raw, trq, _tps, _tpp, _tpn, _din = (
        struct.unpack(TXPDO_FMT, data)
    )
    return sw, vel_raw, trq, err, mode_disp


def cycle(master, drive, rx_bytes):
    drive.output = rx_bytes
    master.send_processdata()
    master.receive_processdata(2000)  # 2 ms wallclock
    return parse_tx(bytes(drive.input))


def walk_cia402(master, drive, max_speed_raw):
    """Walk Shutdown → Switch On → Enable Op via cyclic PDO writes."""
    mode_disp = -1
    for target_sw, cw, label in [
        (SW_READY_TO_SWITCH_ON, CW_SHUTDOWN, "Shutdown"),
        (SW_SWITCHED_ON,        CW_SWITCH_ON, "Switch On"),
        (SW_OPERATION_ENABLED,  CW_ENABLE_OP, "Enable Op"),
    ]:
        deadline = time.perf_counter() + 1.0
        sw = 0
        while time.perf_counter() < deadline:
            sw, _, _, _, mode_disp = cycle(master, drive, build_rx(cw, 0, max_speed_raw))
            if (sw & 0x7F) == target_sw:
                break
            time.sleep(EC_CYCLE_TIME)
        print(f"  {label} (CW 0x{cw:04X}) → SW 0x{sw:04X}  mode_disp={mode_disp}")
        if (sw & 0x7F) != target_sw:
            return False
    return True


def ramp_target(t, ramp_up, hold, ramp_down, peak):
    if t < ramp_up:
        return peak * (t / ramp_up)
    if t < ramp_up + hold:
        return peak
    if t < ramp_up + hold + ramp_down:
        return peak * (1.0 - (t - ramp_up - hold) / ramp_down)
    return 0.0


def main():
    print("=" * 60)
    print(f"CSV (mode 9) test — peak {TEST_TARGET_RPM} RPM, "
          f"ramp {RAMP_DURATION_S}s, hold {HOLD_DURATION_S}s, cycle {EC_CYCLE_TIME*1000:.0f} ms")
    print("=" * 60)

    adapter = find_adapter(ECAT_ADAPTER_NAME)
    if adapter is None:
        print(f"Adapter '{ECAT_ADAPTER_NAME}' not found!")
        return

    master = pysoem.Master()
    master.open(adapter)
    drive = None
    try:
        n = master.config_init()
        if n < 1:
            print("No slaves!")
            return
        drive = master.slaves[0]
        drive.config_func = make_pdo_config(drive)

        master.config_map()
        if master.state_check(pysoem.SAFEOP_STATE, 500_000) != pysoem.SAFEOP_STATE:
            master.read_state()
            print(f"Failed to reach SafeOp (al_status=0x{drive.al_status:X})")
            return

        out_len, in_len = len(drive.output), len(drive.input)
        exp_out, exp_in = struct.calcsize(RXPDO_FMT), struct.calcsize(TXPDO_FMT)
        print(f"PDO mapped: out={out_len}B (expected {exp_out}), in={in_len}B (expected {exp_in})")
        if out_len != exp_out or in_len != exp_in:
            print("  [WARN] PDO size mismatch — struct layout likely off, aborting")
            return

        # SDO-only config (brake) — out of the cyclic path
        drive.sdo_write(BRAKE_SPEED_LEVEL, 0, struct.pack('H', BRAKE_RELEASE_SPEED_RPM))
        drive.sdo_write(BRAKE_OFF_DELAY,   0, struct.pack('H', BRAKE_OFF_DELAY_MS))
        drive.sdo_write(BRAKE_ON_DELAY,    0, struct.pack('H', BRAKE_ON_DELAY_MS))

        # Force mode = CSV via SDO before going cyclic. We *also* send mode in
        # every RxPDO, but writing it here makes sure the drive latches it
        # before Op-state, in case PDO writes alone are not honored as a
        # mode change.
        drive.sdo_write(0x6060, 0, struct.pack('b', MODE_CYCLIC_SYNC_VELOCITY))
        mode_after_sdo = struct.unpack('b', drive.sdo_read(0x6061, 0, 1))[0]
        print(f"Mode after SDO write: 0x6060->9, 0x6061 reports {mode_after_sdo}")

        # Interpolation time period (0x60C2) — required by many drives for
        # CSP/CSV. Sub 1 = value, sub 2 = exponent (10^exp seconds).
        # 4 ms = 4 * 10^-3 s.
        try:
            drive.sdo_write(0x60C2, 1, struct.pack('B', int(EC_CYCLE_TIME * 1000)))
            drive.sdo_write(0x60C2, 2, struct.pack('b', -3))
        except Exception as e:
            print(f"  [WARN] could not set 0x60C2 interpolation period: {e}")

        max_speed_raw = int(SPEED_MAX_RPM / VELOCITY_FACTOR)

        # Distributed Clocks — required by Y7 for CSV/CSP. Without Sync0 the
        # drive accepts state Op-Enabled but won't actuate (target velocity
        # arrives but actual stays ~0). Activate as late as possible to
        # minimize the gap before cyclic data starts flowing.
        sync_ns = int(EC_CYCLE_TIME * 1e9)
        master.config_dc()
        drive.dc_sync(True, sync_ns)
        print(f"DC configured: Sync0 cycle = {sync_ns} ns")

        # Stabilize DC sync in Safe-Op: feed cyclic frames (slave ignores
        # outputs but uses them to align its clock) before requesting Op.
        next_tick = time.perf_counter()
        for _ in range(200):  # ~800 ms of warm-up
            cycle(master, drive, build_rx(0x0000, 0, max_speed_raw))
            next_tick += EC_CYCLE_TIME
            slack = next_tick - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_tick = time.perf_counter()

        master.state = pysoem.OP_STATE
        master.write_state()
        if master.state_check(pysoem.OP_STATE, 500_000) != pysoem.OP_STATE:
            print("Failed to reach Op")
            return
        print("Master at Op")

        if not walk_cia402(master, drive, max_speed_raw):
            print("CiA 402 enable failed — aborting")
            return
        print("Drive enabled; starting CSV profile…")

        total = RAMP_DURATION_S + HOLD_DURATION_S + RAMP_DURATION_S
        t0 = time.perf_counter()
        next_tick = t0
        last_target = 0.0
        cycles = 0
        aborted = False
        try:
            while True:
                t = time.perf_counter() - t0
                if t >= total:
                    break
                target_rpm = ramp_target(t, RAMP_DURATION_S, HOLD_DURATION_S,
                                         RAMP_DURATION_S, TEST_TARGET_RPM)
                last_target = target_rpm
                target_raw = int(target_rpm / VELOCITY_FACTOR)
                sw, vel_raw, trq, err, mode_disp = cycle(
                    master, drive, build_rx(CW_ENABLE_OP, target_raw, max_speed_raw)
                )
                cycles += 1
                if sw & SW_FAULT:
                    print(f"  [FAULT] sw=0x{sw:04X} err=0x{err:04X}")
                    aborted = True
                    break
                if (sw & 0x7F) != SW_OPERATION_ENABLED:
                    print(f"  [STATE LOST] sw=0x{sw:04X}")
                    aborted = True
                    break
                if cycles % PRINT_EVERY_N_CYCLES == 0:
                    print(f"  t={t:5.2f}s  tgt={target_rpm:6.1f} RPM  "
                          f"act={vel_raw * VELOCITY_FACTOR:6.1f} RPM  "
                          f"trq={trq:>5d}  sw=0x{sw:04X}  mode={mode_disp}")
                # Strict periodic timing
                next_tick += EC_CYCLE_TIME
                slack = next_tick - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
                else:
                    next_tick = time.perf_counter()  # missed a tick — resync
        except KeyboardInterrupt:
            print("\n  Ctrl+C — emergency ramp-down")
            t_kill = time.perf_counter()
            kill_dur = 0.3
            while time.perf_counter() - t_kill < kill_dur:
                f = (time.perf_counter() - t_kill) / kill_dur
                rpm = last_target * (1.0 - f)
                cycle(master, drive,
                      build_rx(CW_ENABLE_OP, int(rpm / VELOCITY_FACTOR), max_speed_raw))
                time.sleep(EC_CYCLE_TIME)

        # Hold target=0 for ~50 cycles before disable, so the drive lands gently
        for _ in range(50):
            cycle(master, drive, build_rx(CW_ENABLE_OP, 0, max_speed_raw))
            time.sleep(EC_CYCLE_TIME)
        print(f"Done — {cycles} cycles{' (aborted)' if aborted else ''}.")

    finally:
        if drive is not None:
            try:
                cycle(master, drive, build_rx(0x0000, 0, 0))
                time.sleep(0.05)
            except Exception:
                pass
        try:
            master.state = pysoem.INIT_STATE
            master.write_state()
            time.sleep(0.1)
            master.close()
        except Exception:
            pass
        print("EtherCAT closed.")


if __name__ == "__main__":
    main()
