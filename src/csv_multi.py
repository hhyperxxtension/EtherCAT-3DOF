"""
Multi-drive CSV (Cyclic Sync Velocity, mode 9) controller.

Discovers every Y7 on the bus, sets up PDO mapping + DC sync, walks the
CiA 402 state machine in parallel for all drives, then runs an
interactive controller where you can address each drive individually.

The realtime cycle thread is the only thing that touches the EtherCAT
bus once the drives are enabled: every EC_CYCLE_TIME it rebuilds the
RxPDO for each drive from the shared `targets_rpm` array, exchanges one
frame, and parses the TxPDO of each drive into the shared `status`
array. A separate status thread prints a one-liner with all drives.
The main thread blocks on input() for commands.

UI commands:
    <idx> <rpm>     set drive <idx> to <rpm>          e.g.  0 200
    all <rpm>       set every drive to <rpm>          e.g.  all -100
    <Enter>         stop all drives (0 RPM)
    q               quit (ramps to 0, disables, closes)

SAFETY:
- Targets clamped to ±SPEED_MAX_RPM
- On quit / Ctrl+C, all targets are ramped to 0 before disabling.
- If any drive drops out of OPERATION_ENABLED, it is flagged in the
  status line with a '!' marker (no automatic recovery yet).
"""

import pysoem
import shutil
import struct
import sys
import threading
import time

from config import (
    ECAT_ADAPTER_NAME, EC_CYCLE_TIME,
    BRAKE_OFF_DELAY, BRAKE_SPEED_LEVEL, BRAKE_ON_DELAY,
    BRAKE_RELEASE_SPEED_RPM, BRAKE_OFF_DELAY_MS, BRAKE_ON_DELAY_MS,
    SPEED_MAX_RPM,
    SW_READY_TO_SWITCH_ON, SW_SWITCHED_ON, SW_OPERATION_ENABLED, SW_FAULT,
    CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OP,
    MODE_CYCLIC_SYNC_VELOCITY,
    MAX_TORQUE, POS_TORQUE_LIMIT, NEG_TORQUE_LIMIT, MAX_TORQUE_PERMILLE,
)
from speed_test import VELOCITY_FACTOR

# PDO assignment (same as csv_test.py — Y7 ESI confirmed layouts)
RXPDO_INDEX = 0x1601
TXPDO_INDEX = 0x1A01
RXPDO_FMT = '<HbhiIHi'
TXPDO_FMT = '<HHbiihHiiI'
RXPDO_SIZE = struct.calcsize(RXPDO_FMT)
TXPDO_SIZE = struct.calcsize(TXPDO_FMT)

STATUS_PERIOD_S = 0.2

# Shared state — protected by state_lock. The cycle thread writes status
# and reads targets; the UI thread writes targets and reads status.
state_lock = threading.Lock()
targets_rpm: list[float] = []
status: list[dict] = []
op_enabled: list[bool] = []

# Serializes stdout writes between the status thread and the UI thread.
output_lock = threading.Lock()


def enable_vt_processing():
    """Windows: enable ANSI escape sequence processing on stdout."""
    try:
        import ctypes
        from ctypes import wintypes
        kernel32 = ctypes.windll.kernel32
        h = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = wintypes.DWORD()
        if kernel32.GetConsoleMode(h, ctypes.byref(mode)):
            kernel32.SetConsoleMode(h, mode.value | 0x0004)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:
        pass


def setup_screen(n):
    """Reserve the top N+1 rows for a static status block, route everything
    else into a scroll region below it. Uses DECSTBM (\\x1b[r) — works on
    Windows Terminal and modern xterm-compatible terminals."""
    rows = shutil.get_terminal_size().lines
    with output_lock:
        sys.stdout.write('\x1b[2J')      # clear screen
        sys.stdout.write('\x1b[H')       # cursor to (1,1)
        for _ in range(n):
            sys.stdout.write('\n')       # reserve N rows for status lines
        sys.stdout.write('-' * 70 + '\n')  # row N+1: separator
        sys.stdout.write(f'\x1b[{n + 2};{rows}r')  # scroll region: rows N+2..end
        sys.stdout.write(f'\x1b[{n + 2};1H')       # park cursor at top of scroll area
        sys.stdout.flush()


def restore_screen():
    """Reset scroll region; place cursor on the last terminal row."""
    rows = shutil.get_terminal_size().lines
    with output_lock:
        sys.stdout.write('\x1b[r')                 # full-screen scroll region
        sys.stdout.write(f'\x1b[{rows};1H')        # cursor at bottom
        sys.stdout.write('\n')
        sys.stdout.flush()


def find_adapter(desc_hint):
    for a in pysoem.find_adapters():
        if desc_hint.lower() in a.desc.decode().lower():
            return a.name
    return None


def make_pdo_config(slave):
    """Per-slave PDO reassignment hook. Runs in Pre-Op during config_map."""
    def cb(_slave_pos):
        slave.sdo_write(0x1C12, 0, struct.pack('B', 0))
        slave.sdo_write(0x1C12, 1, struct.pack('<H', RXPDO_INDEX))
        slave.sdo_write(0x1C12, 0, struct.pack('B', 1))
        slave.sdo_write(0x1C13, 0, struct.pack('B', 0))
        slave.sdo_write(0x1C13, 1, struct.pack('<H', TXPDO_INDEX))
        slave.sdo_write(0x1C13, 0, struct.pack('B', 1))
    return cb


def build_rx(cw, target_vel_raw, max_speed_raw):
    return struct.pack(
        RXPDO_FMT, cw, MODE_CYCLIC_SYNC_VELOCITY,
        0, 0, max_speed_raw, 0, target_vel_raw,
    )


def parse_tx(data):
    err, sw, mode_disp, _pos, vel_raw, trq, _tps, _tpp, _tpn, _din = (
        struct.unpack(TXPDO_FMT, data)
    )
    return {'sw': sw, 'vel_raw': vel_raw, 'trq': trq,
            'mode_disp': mode_disp, 'err': err}


def configure_drive_sdo(drive):
    """Per-drive SDO config done in Safe-Op before going cyclic."""
    drive.sdo_write(BRAKE_SPEED_LEVEL, 0, struct.pack('H', BRAKE_RELEASE_SPEED_RPM))
    drive.sdo_write(BRAKE_OFF_DELAY,   0, struct.pack('H', BRAKE_OFF_DELAY_MS))
    drive.sdo_write(BRAKE_ON_DELAY,    0, struct.pack('H', BRAKE_ON_DELAY_MS))
    drive.sdo_write(0x6060, 0, struct.pack('b', MODE_CYCLIC_SYNC_VELOCITY))
    # Safety torque cap (‰ of rated torque) — applied before enable. The Y7
    # uses 0x60E0/0x60E1 as the active limit in these modes (0x6072 is not
    # enforced), so write all three.
    try:
        drive.sdo_write(POS_TORQUE_LIMIT, 0, struct.pack('<H', MAX_TORQUE_PERMILLE))
        drive.sdo_write(NEG_TORQUE_LIMIT, 0, struct.pack('<H', MAX_TORQUE_PERMILLE))
        drive.sdo_write(MAX_TORQUE,       0, struct.pack('<H', MAX_TORQUE_PERMILLE))
        pos = struct.unpack('<H', drive.sdo_read(POS_TORQUE_LIMIT, 0, 2))[0]
        neg = struct.unpack('<H', drive.sdo_read(NEG_TORQUE_LIMIT, 0, 2))[0]
        print(f"  torque limit 0x60E0/0x60E1 = {pos}/{neg} (permille rated)")
    except Exception as e:
        print(f"  [WARN] torque-limit write failed: {e}")
    try:
        drive.sdo_write(0x60C2, 1, struct.pack('B', int(EC_CYCLE_TIME * 1000)))
        drive.sdo_write(0x60C2, 2, struct.pack('b', -3))
    except Exception as e:
        print(f"  [WARN] 0x60C2 write failed: {e}")


def exchange_one(master, drives, cw, max_speed_raw, target_raws=None):
    """Single bus cycle: write RxPDO for every drive, send, receive,
    return list of parsed TxPDOs. Caller is the *only* user of the bus
    at this point — no other thread should be holding pysoem state."""
    for i, d in enumerate(drives):
        tr = 0 if target_raws is None else target_raws[i]
        d.output = build_rx(cw, tr, max_speed_raw)
    master.send_processdata()
    master.receive_processdata(2000)
    return [parse_tx(bytes(d.input)) for d in drives]


def walk_cia402_multi(master, drives, max_speed_raw):
    """Drive every slave through Shutdown → Switch On → Enable Op in
    parallel. Advance to next CW only when ALL drives reach the
    expected statusword bits."""
    steps = [
        (SW_READY_TO_SWITCH_ON, CW_SHUTDOWN, "Shutdown"),
        (SW_SWITCHED_ON,        CW_SWITCH_ON, "Switch On"),
        (SW_OPERATION_ENABLED,  CW_ENABLE_OP, "Enable Op"),
    ]
    next_tick = time.perf_counter()
    for target_sw, cw, label in steps:
        deadline = time.perf_counter() + 1.5
        snap = []
        while time.perf_counter() < deadline:
            snap = exchange_one(master, drives, cw, max_speed_raw)
            if all((s['sw'] & 0x7F) == target_sw for s in snap):
                break
            next_tick += EC_CYCLE_TIME
            slack = next_tick - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_tick = time.perf_counter()
        sws = ' '.join(f"D{i}=0x{s['sw']:04X}" for i, s in enumerate(snap))
        print(f"  {label} (CW 0x{cw:04X}) → {sws}")
        if not all((s['sw'] & 0x7F) == target_sw for s in snap):
            return False
    return True


def cycle_loop(master, drives, max_speed_raw, stop_event):
    """Realtime cycle: every EC_CYCLE_TIME, refresh outputs for all
    drives from `targets_rpm`, exchange a frame, write `status`."""
    n = len(drives)
    next_tick = time.perf_counter()
    while not stop_event.is_set():
        with state_lock:
            tgts = list(targets_rpm)
        target_raws = [int(t / VELOCITY_FACTOR) for t in tgts]
        snap = exchange_one(master, drives, CW_ENABLE_OP, max_speed_raw, target_raws)
        with state_lock:
            for i in range(n):
                status[i] = snap[i]
                if (snap[i]['sw'] & 0x7F) != SW_OPERATION_ENABLED:
                    op_enabled[i] = False
        next_tick += EC_CYCLE_TIME
        slack = next_tick - time.perf_counter()
        if slack > 0:
            time.sleep(slack)
        else:
            next_tick = time.perf_counter()


def status_loop(stop_event, n):
    """Refresh one row per drive at fixed absolute rows 1..N. Save/restore
    cursor around the update so the UI typing position is preserved."""
    while not stop_event.wait(STATUS_PERIOD_S):
        with state_lock:
            tgts = list(targets_rpm)
            snap = [dict(s) for s in status]
            ok = list(op_enabled)
        with output_lock:
            sys.stdout.write('\x1b[s')   # save cursor (wherever the UI left it)
            for i in range(n):
                s = snap[i]
                vel = s['vel_raw'] * VELOCITY_FACTOR
                mark = ' ' if ok[i] else '!'
                line = (
                    f"[D{i}{mark}] tgt={tgts[i]:>+7.1f} RPM  "
                    f"act={vel:>+7.1f} RPM  trq={s['trq']:>+5d}  "
                    f"sw=0x{s['sw']:04X}  mode={s['mode_disp']}"
                )
                sys.stdout.write(f'\x1b[{i + 1};1H')  # row i+1, col 1
                sys.stdout.write(line + '\x1b[K')      # write + clear to EOL
            sys.stdout.write('\x1b[u')   # restore cursor
            sys.stdout.flush()


def _say(msg):
    """Thread-safe print into the scroll region (below the status block)."""
    with output_lock:
        sys.stdout.write(msg + '\n')
        sys.stdout.flush()


def _prompt():
    with output_lock:
        sys.stdout.write('> ')
        sys.stdout.flush()


def ui_loop(stop_event, n):
    """Blocking command prompt. Runs on the main thread inside the
    scroll region; the status block above is untouched."""
    _say("Commands: <idx> <rpm> | all <rpm> | <Enter>=stop all | q=quit")
    while not stop_event.is_set():
        _prompt()
        try:
            raw = sys.stdin.readline()
        except KeyboardInterrupt:
            _say("")
            break
        if not raw:  # EOF
            break
        line = raw.strip()
        if line in ("q", "quit", "exit"):
            break
        if line == "":
            with state_lock:
                for i in range(n):
                    targets_rpm[i] = 0.0
            _say("  → all stop")
            continue
        parts = line.split()
        if len(parts) != 2:
            _say("  format: <idx|all> <rpm>")
            continue
        try:
            rpm = float(parts[1])
        except ValueError:
            _say("  rpm must be a number")
            continue
        rpm = max(-float(SPEED_MAX_RPM), min(float(SPEED_MAX_RPM), rpm))
        if parts[0] == "all":
            with state_lock:
                for i in range(n):
                    targets_rpm[i] = rpm
            _say(f"  → all = {rpm} RPM")
            continue
        try:
            idx = int(parts[0])
        except ValueError:
            _say("  idx must be int or 'all'")
            continue
        if not 0 <= idx < n:
            _say(f"  idx out of range (0..{n - 1})")
            continue
        with state_lock:
            targets_rpm[idx] = rpm
        _say(f"  → D{idx} = {rpm} RPM")
    stop_event.set()


def ramp_to_zero(master, drives, max_speed_raw, duration_s=0.4):
    """Linearly ramp every drive's target to 0 over duration_s. Runs in
    the main thread *after* the cycle thread has been stopped, so we
    own the bus exclusively here."""
    with state_lock:
        start = list(targets_rpm)
    steps = max(1, int(duration_s / EC_CYCLE_TIME))
    next_tick = time.perf_counter()
    for k in range(steps + 1):
        f = 1.0 - k / steps
        raws = [int(start[i] * f / VELOCITY_FACTOR) for i in range(len(drives))]
        exchange_one(master, drives, CW_ENABLE_OP, max_speed_raw, raws)
        next_tick += EC_CYCLE_TIME
        slack = next_tick - time.perf_counter()
        if slack > 0:
            time.sleep(slack)
        else:
            next_tick = time.perf_counter()


def main():
    enable_vt_processing()
    print("=" * 70)
    print("CSV multi-drive controller")
    print("=" * 70)

    adapter = find_adapter(ECAT_ADAPTER_NAME)
    if adapter is None:
        print(f"Adapter '{ECAT_ADAPTER_NAME}' not found!")
        return

    master = pysoem.Master()
    master.open(adapter)
    drives = []
    try:
        n = master.config_init()
        if n < 1:
            print("No slaves on the bus!")
            return
        drives = list(master.slaves)
        print(f"Discovered {n} slave(s):")
        for i, d in enumerate(drives):
            print(f"  D{i}: {d.name}  product=0x{d.id:08X}")
            d.config_func = make_pdo_config(d)

        master.config_map()
        if master.state_check(pysoem.SAFEOP_STATE, 500_000) != pysoem.SAFEOP_STATE:
            master.read_state()
            print("Failed to reach SafeOp:")
            for i, d in enumerate(drives):
                print(f"  D{i}: al_status=0x{d.al_status:X}")
            return

        for i, d in enumerate(drives):
            out_len, in_len = len(d.output), len(d.input)
            if out_len != RXPDO_SIZE or in_len != TXPDO_SIZE:
                print(f"  D{i}: PDO size mismatch out={out_len}/{RXPDO_SIZE} "
                      f"in={in_len}/{TXPDO_SIZE} — aborting")
                return
        print(f"PDO mapped: out={RXPDO_SIZE}B / in={TXPDO_SIZE}B per drive")

        for d in drives:
            configure_drive_sdo(d)

        max_speed_raw = int(SPEED_MAX_RPM / VELOCITY_FACTOR)

        sync_ns = int(EC_CYCLE_TIME * 1e9)
        master.config_dc()
        for d in drives:
            d.dc_sync(True, sync_ns)
        print(f"DC configured: Sync0 cycle = {sync_ns} ns")

        # Warm-up cycles in SafeOp to align DC clocks before requesting Op.
        next_tick = time.perf_counter()
        for _ in range(200):
            exchange_one(master, drives, 0x0000, max_speed_raw)
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

        if not walk_cia402_multi(master, drives, max_speed_raw):
            print("CiA 402 enable failed — aborting")
            return
        print("All drives enabled.")

        # Initialize shared state now that we know N.
        for _ in range(n):
            targets_rpm.append(0.0)
            status.append({'sw': 0, 'vel_raw': 0, 'trq': 0, 'mode_disp': 0, 'err': 0})
            op_enabled.append(True)

        # From here on, status block (rows 1..N) is reserved; UI scrolls below.
        setup_screen(n)

        stop_event = threading.Event()
        t_cycle = threading.Thread(
            target=cycle_loop, args=(master, drives, max_speed_raw, stop_event),
            daemon=True,
        )
        t_status = threading.Thread(
            target=status_loop, args=(stop_event, n), daemon=True,
        )
        t_cycle.start()
        t_status.start()

        try:
            ui_loop(stop_event, n)
        finally:
            stop_event.set()
            t_cycle.join(timeout=1.0)
            t_status.join(timeout=1.0)
            restore_screen()
            # Cycle thread is gone — main thread owns the bus again.
            ramp_to_zero(master, drives, max_speed_raw)

    finally:
        # Final bus cleanup: disable voltage on every drive, then Init.
        try:
            if drives:
                exchange_one(master, drives, 0x0000, 0)
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
