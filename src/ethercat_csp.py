"""
Multi-drive Cyclic Sync Position (CSP, mode 8) bus layer.

Refactor of csv_multi.py for *position* control. Reuses the proven Y7 PDO
mapping (RxPDO 0x1601 / TxPDO 0x1A01) and struct layouts — that RxPDO
already carries Target position (0x607A) and the TxPDO carries Position
actual (0x6064), so only the mode (8 instead of 9) and which field we
write change.

Exposed as a class so controller.py / a GUI can drive it:
    bus = CSPBus()
    bus.connect()                      # discover, enable, hold at current pos
    cur = bus.actual_counts()          # raw 0x6064 per drive
    bus.set_goal_counts(idx, counts)   # request a move (slew-rate limited)
    bus.disconnect()

CSP SAFETY MODEL
- On enable the target position is latched to the *current* actual position
  of every drive (read during warm-up), so the drive holds still instead of
  snapping to 0.
- Commands set a `goal`; the realtime thread walks `target` toward `goal` by
  at most `max_step` counts/cycle (from CSP_JOG_RPM). A large goal change can
  never become a single-cycle position jump.
- The drive's 0x6080 (max motor speed) is a hard cap underneath that.
- On disconnect the target is frozen at the current actual, then voltage is
  disabled (brake engages), then the bus goes to Init.

Run directly for an interactive CALIBRATION tool (moves hardware!):
    python ethercat_csp.py
Commands:
    <idx> deg <value>   move joint <idx> by <value> degrees (relative)
    <idx> rev <value>   move drive <idx> by <value> MOTOR revolutions
    <idx> cnt <value>   move drive <idx> by <value> raw counts
    home                print current actual counts (capture home_counts)
    <Enter> / stop      freeze every drive at its current position
    q                   quit (freeze, disable, close)
Use it to: confirm ENCODER_COUNTS_PER_REV (command `rev 1`, check the shaft
turns exactly once / the joint moves 360/gear degrees), find JOINT_DIR (does
a +deg command move the joint the CCW-positive way?), and read home_counts.
"""

import collections
import shutil
import struct
import sys
import threading
import time

import pysoem

from config import (
    ECAT_ADAPTER_NAME, EC_CYCLE_TIME,
    BRAKE_OFF_DELAY, BRAKE_SPEED_LEVEL, BRAKE_ON_DELAY,
    BRAKE_RELEASE_SPEED_RPM, BRAKE_OFF_DELAY_MS, BRAKE_ON_DELAY_MS,
    SPEED_MAX_RPM, CSP_JOG_RPM,
    SW_READY_TO_SWITCH_ON, SW_SWITCHED_ON, SW_OPERATION_ENABLED, SW_FAULT,
    CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OP,
    MODE_CYCLIC_SYNC_POSITION,
    ENCODER_COUNTS_PER_REV, GEAR_RATIO,
    MAX_TORQUE, POS_TORQUE_LIMIT, NEG_TORQUE_LIMIT, MAX_TORQUE_PERMILLE,
)
from speed_test import VELOCITY_FACTOR

# Same fixed PDO mapping csv_multi uses (Y7 ESI confirmed).
RXPDO_INDEX = 0x1601
TXPDO_INDEX = 0x1A01
RXPDO_FMT = '<HbhiIHi'   # cw, mode, target_torque, target_pos, max_speed, tp, target_vel
TXPDO_FMT = '<HHbiihHiiI'  # err, sw, mode_disp, pos, vel, trq, ..., din
RXPDO_SIZE = struct.calcsize(RXPDO_FMT)
TXPDO_SIZE = struct.calcsize(TXPDO_FMT)


def find_adapter(desc_hint):
    for a in pysoem.find_adapters():
        if desc_hint.lower() in a.desc.decode().lower():
            return a.name
    return None


def _pdo_config(slave):
    """Per-slave PDO reassignment, runs in Pre-Op during config_map."""
    def cb(_pos):
        slave.sdo_write(0x1C12, 0, struct.pack('B', 0))
        slave.sdo_write(0x1C12, 1, struct.pack('<H', RXPDO_INDEX))
        slave.sdo_write(0x1C12, 0, struct.pack('B', 1))
        slave.sdo_write(0x1C13, 0, struct.pack('B', 0))
        slave.sdo_write(0x1C13, 1, struct.pack('<H', TXPDO_INDEX))
        slave.sdo_write(0x1C13, 0, struct.pack('B', 1))
    return cb


def build_rx(cw, target_pos_raw, max_speed_raw):
    """RxPDO for CSP: mode 8, position field = target, velocity field = 0."""
    return struct.pack(RXPDO_FMT, cw, MODE_CYCLIC_SYNC_POSITION,
                       0, int(target_pos_raw), int(max_speed_raw), 0, 0)


def parse_tx(data):
    err, sw, mode_disp, pos, vel, trq, _tps, _tpp, _tpn, _din = (
        struct.unpack(TXPDO_FMT, data)
    )
    return {'sw': sw, 'pos': pos, 'vel': vel, 'trq': trq,
            'mode_disp': mode_disp, 'err': err}


class CSPBus:
    """Owns the EtherCAT master and the realtime position-streaming thread."""

    def __init__(self, adapter_name=ECAT_ADAPTER_NAME, cycle_time=EC_CYCLE_TIME,
                 jog_rpm=CSP_JOG_RPM):
        self.adapter_name = adapter_name
        self.cycle_time = cycle_time
        # Max counts the target may advance per cycle (motor-side jog cap).
        self.max_step = max(1, int(jog_rpm / 60.0 * ENCODER_COUNTS_PER_REV * cycle_time))
        # Hard per-cycle cap while playing a pre-planned trajectory. The plan
        # already respects joint velocity limits; this only catches a bad jump
        # (e.g. a setpoint bug). Generous: ~2000 motor RPM.
        self.hard_max_step = max(1, int(2000.0 / 60.0 * ENCODER_COUNTS_PER_REV * cycle_time))
        self.max_speed_raw = int(SPEED_MAX_RPM / VELOCITY_FACTOR)

        self.master = None
        self.drives = []
        self.n = 0

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._cycle_thread = None
        # Shared state (guarded by _lock):
        self._target = []   # counts currently commanded on the bus
        self._goal = []     # counts the cycle thread is walking toward (jog)
        self._queue = collections.deque()  # pre-planned count-vectors to play
        self._status = []   # last parsed TxPDO per drive
        self._faulted = []

    # ── bring-up ────────────────────────────────────────────────────────
    def _exchange(self, cw, targets):
        for i, d in enumerate(self.drives):
            d.output = build_rx(cw, targets[i], self.max_speed_raw)
        self.master.send_processdata()
        self.master.receive_processdata(2000)
        return [parse_tx(bytes(d.input)) for d in self.drives]

    def _configure_sdo(self, d):
        d.sdo_write(BRAKE_SPEED_LEVEL, 0, struct.pack('H', BRAKE_RELEASE_SPEED_RPM))
        d.sdo_write(BRAKE_OFF_DELAY,   0, struct.pack('H', BRAKE_OFF_DELAY_MS))
        d.sdo_write(BRAKE_ON_DELAY,    0, struct.pack('H', BRAKE_ON_DELAY_MS))
        d.sdo_write(0x6060, 0, struct.pack('b', MODE_CYCLIC_SYNC_POSITION))
        # Safety torque cap (‰ of rated torque) — applied before enable. The
        # Y7 uses 0x60E0/0x60E1 as the active limit in these modes (0x6072 is
        # not enforced), so write all three.
        try:
            d.sdo_write(POS_TORQUE_LIMIT, 0, struct.pack('<H', MAX_TORQUE_PERMILLE))
            d.sdo_write(NEG_TORQUE_LIMIT, 0, struct.pack('<H', MAX_TORQUE_PERMILLE))
            d.sdo_write(MAX_TORQUE,       0, struct.pack('<H', MAX_TORQUE_PERMILLE))
            pos = struct.unpack('<H', d.sdo_read(POS_TORQUE_LIMIT, 0, 2))[0]
            neg = struct.unpack('<H', d.sdo_read(NEG_TORQUE_LIMIT, 0, 2))[0]
            print(f"  torque limit 0x60E0/0x60E1 = {pos}/{neg} (permille rated)")
        except Exception as e:
            print(f"  [WARN] torque-limit write failed: {e}")
        try:
            d.sdo_write(0x60C2, 1, struct.pack('B', int(self.cycle_time * 1000)))
            d.sdo_write(0x60C2, 2, struct.pack('b', -3))
        except Exception as e:
            print(f"  [WARN] 0x60C2 write failed: {e}")

    def _walk_cia402(self, targets):
        """Enable every drive in lockstep, holding target = current actual."""
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
                snap = self._exchange(cw, targets)
                if all((s['sw'] & 0x7F) == target_sw for s in snap):
                    break
                next_tick += self.cycle_time
                slack = next_tick - time.perf_counter()
                time.sleep(slack) if slack > 0 else None
                if slack <= 0:
                    next_tick = time.perf_counter()
            sws = ' '.join(f"D{i}=0x{s['sw']:04X}" for i, s in enumerate(snap))
            print(f"  {label} (CW 0x{cw:04X}) -> {sws}")
            if not all((s['sw'] & 0x7F) == target_sw for s in snap):
                return False
        return True

    def connect(self):
        adapter = find_adapter(self.adapter_name)
        if adapter is None:
            raise RuntimeError(f"Adapter '{self.adapter_name}' not found")

        self.master = pysoem.Master()
        self.master.open(adapter)

        self.n = self.master.config_init()
        if self.n < 1:
            raise RuntimeError("No slaves on the bus")
        self.drives = list(self.master.slaves)
        print(f"Discovered {self.n} slave(s):")
        for i, d in enumerate(self.drives):
            print(f"  D{i}: {d.name}  product=0x{d.id:08X}")
            d.config_func = _pdo_config(d)

        self.master.config_map()
        if self.master.state_check(pysoem.SAFEOP_STATE, 500_000) != pysoem.SAFEOP_STATE:
            self.master.read_state()
            raise RuntimeError("Failed to reach SafeOp: " +
                               ', '.join(f"D{i} al=0x{d.al_status:X}"
                                         for i, d in enumerate(self.drives)))
        for i, d in enumerate(self.drives):
            if len(d.output) != RXPDO_SIZE or len(d.input) != TXPDO_SIZE:
                raise RuntimeError(
                    f"D{i} PDO size mismatch out={len(d.output)}/{RXPDO_SIZE} "
                    f"in={len(d.input)}/{TXPDO_SIZE}")
        print(f"PDO mapped: out={RXPDO_SIZE}B / in={TXPDO_SIZE}B per drive")

        for d in self.drives:
            self._configure_sdo(d)

        sync_ns = int(self.cycle_time * 1e9)
        self.master.config_dc()
        for d in self.drives:
            d.dc_sync(True, sync_ns)
        print(f"DC configured: Sync0 = {sync_ns} ns")

        # Warm-up in SafeOp with controlword 0; read each drive's actual
        # position so we can latch the target to it before enabling.
        next_tick = time.perf_counter()
        snap = []
        seed = [0] * self.n
        for k in range(200):
            snap = self._exchange(0x0000, seed)
            seed = [s['pos'] for s in snap]   # follow actual while idle
            next_tick += self.cycle_time
            slack = next_tick - time.perf_counter()
            time.sleep(slack) if slack > 0 else None
            if slack <= 0:
                next_tick = time.perf_counter()
        targets = [s['pos'] for s in snap]
        print("Latched targets to actual position: " +
              ', '.join(f"D{i}={p}" for i, p in enumerate(targets)))

        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        if self.master.state_check(pysoem.OP_STATE, 500_000) != pysoem.OP_STATE:
            raise RuntimeError("Failed to reach Op")
        print("Master at Op")

        if not self._walk_cia402(targets):
            raise RuntimeError("CiA 402 enable failed")
        print("All drives enabled (holding position).")

        with self._lock:
            self._target = list(targets)
            self._goal = list(targets)
            self._status = [dict(s) for s in snap]
            self._faulted = [False] * self.n

        self._stop.clear()
        self._cycle_thread = threading.Thread(target=self._cycle_loop, daemon=True)
        self._cycle_thread.start()

    # ── realtime loop ───────────────────────────────────────────────────
    def _cycle_loop(self):
        next_tick = time.perf_counter()
        while not self._stop.is_set():
            with self._lock:
                target = list(self._target)
                if self._queue:
                    # Playing a planned trajectory: follow its setpoints, with
                    # only the hard safety cap. Keep goal synced so we hold the
                    # last point once the queue drains.
                    desired = self._queue.popleft()
                    self._goal = list(desired)
                    step = self.hard_max_step
                else:
                    desired = list(self._goal)   # jog / hold
                    step = self.max_step
            for i in range(self.n):
                delta = desired[i] - target[i]
                if delta > step:
                    delta = step
                elif delta < -step:
                    delta = -step
                target[i] += delta
            snap = self._exchange(CW_ENABLE_OP, target)
            with self._lock:
                self._target = target
                self._status = snap
                for i in range(self.n):
                    if (snap[i]['sw'] & 0x7F) != SW_OPERATION_ENABLED:
                        self._faulted[i] = True
            next_tick += self.cycle_time
            slack = next_tick - time.perf_counter()
            time.sleep(slack) if slack > 0 else None
            if slack <= 0:
                next_tick = time.perf_counter()

    # ── public API ──────────────────────────────────────────────────────
    def actual_counts(self):
        with self._lock:
            return [s['pos'] for s in self._status]

    def status(self):
        with self._lock:
            return [dict(s) for s in self._status], list(self._target), \
                   list(self._goal), list(self._faulted)

    def set_goal_counts(self, idx, counts):
        with self._lock:
            self._goal[idx] = int(counts)

    def move_by_counts(self, idx, delta):
        """Set goal = current actual + delta (relative jog)."""
        with self._lock:
            cur = self._status[idx]['pos']
            self._goal[idx] = cur + int(delta)

    def play_trajectory(self, count_vectors):
        """Queue a list of per-drive count-vectors for the RT thread to play
        out, one per cycle. Replaces any trajectory currently in flight."""
        for v in count_vectors:
            if len(v) != self.n:
                raise ValueError(f"each vector needs {self.n} counts")
        with self._lock:
            self._queue = collections.deque(count_vectors)

    def is_busy(self):
        """True while a queued trajectory is still being played."""
        with self._lock:
            return len(self._queue) > 0

    def freeze(self):
        """Stop motion now: drop any queued trajectory and hold current pos."""
        with self._lock:
            self._queue.clear()
            for i in range(self.n):
                self._goal[i] = self._status[i]['pos']

    def disconnect(self):
        if self._cycle_thread is not None:
            self.freeze()
            time.sleep(0.1)
            self._stop.set()
            self._cycle_thread.join(timeout=1.0)
            self._cycle_thread = None
        try:
            if self.drives:
                self._exchange(0x0000, [s['pos'] for s in self._status]
                               if self._status else [0] * self.n)
                time.sleep(0.05)
        except Exception:
            pass
        try:
            if self.master is not None:
                self.master.state = pysoem.INIT_STATE
                self.master.write_state()
                time.sleep(0.1)
                self.master.close()
        except Exception:
            pass
        print("EtherCAT closed.")


# ── interactive calibration tool ────────────────────────────────────────
def _counts_for(kind, value, idx):
    """Relative-move size in raw counts for a command of the given kind."""
    if kind == "cnt":
        return int(value)
    if kind == "rev":   # motor revolutions
        return int(round(value * ENCODER_COUNTS_PER_REV))
    if kind == "deg":   # joint degrees → counts via gear ratio
        return int(round(value / 360.0 * GEAR_RATIO[idx] * ENCODER_COUNTS_PER_REV))
    raise ValueError(kind)


# Serializes stdout writes between the monitor thread and the prompt.
_output_lock = threading.Lock()


def _enable_vt_processing():
    """Windows: enable ANSI escape sequence processing on stdout."""
    try:
        import ctypes
        from ctypes import wintypes
        kernel32 = ctypes.windll.kernel32
        h = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = wintypes.DWORD()
        if kernel32.GetConsoleMode(h, ctypes.byref(mode)):
            kernel32.SetConsoleMode(h, mode.value | 0x0004)
    except Exception:
        pass


def _setup_screen(n):
    """Reserve the top n+1 rows for a static per-drive status block; route
    everything else into a scroll region below it (same idea as csv_multi)."""
    rows = shutil.get_terminal_size().lines
    with _output_lock:
        sys.stdout.write('\x1b[2J')          # clear screen
        sys.stdout.write('\x1b[H')           # cursor to (1,1)
        for _ in range(n):
            sys.stdout.write('\n')           # reserve n rows
        sys.stdout.write('-' * 100 + '\n')   # separator on row n+1
        sys.stdout.write(f'\x1b[{n + 2};{rows}r')  # scroll region n+2..end
        sys.stdout.write(f'\x1b[{n + 2};1H')       # park cursor in scroll area
        sys.stdout.flush()


def _restore_screen():
    rows = shutil.get_terminal_size().lines
    with _output_lock:
        sys.stdout.write('\x1b[r')               # full-screen scroll region
        sys.stdout.write(f'\x1b[{rows};1H')
        sys.stdout.write('\n')
        sys.stdout.flush()


def _monitor(bus, stop, n):
    """One row per drive at fixed rows 1..n; preserve the prompt cursor."""
    while not stop.wait(0.2):
        snap, target, goal, faulted = bus.status()
        with _output_lock:
            sys.stdout.write('\x1b[s')           # save cursor
            for i in range(n):
                s = snap[i]
                mark = '!' if faulted[i] else ' '
                line = (f"[D{i}{mark}] pos={s['pos']:>+12d}  tgt={target[i]:>+12d}  "
                        f"goal={goal[i]:>+12d}  trq={s['trq']:>+5d}  "
                        f"sw=0x{s['sw']:04X}  mode={s['mode_disp']}")
                sys.stdout.write(f'\x1b[{i + 1};1H')   # row i+1
                sys.stdout.write(line + '\x1b[K')       # write + clear to EOL
            sys.stdout.write('\x1b[u')           # restore cursor
            sys.stdout.flush()


def _say(msg):
    with _output_lock:
        sys.stdout.write(msg + '\n')
        sys.stdout.flush()


def _prompt():
    with _output_lock:
        sys.stdout.write('csp> ')
        sys.stdout.flush()


def main():
    _enable_vt_processing()
    print("=" * 70)
    print("CSP multi-drive - CALIBRATION tool (moves hardware)")
    print("=" * 70)
    bus = CSPBus()
    try:
        bus.connect()
    except Exception as e:
        print(f"connect failed: {e}")
        bus.disconnect()
        return

    n = bus.n
    _setup_screen(n)
    _say(f"slew cap: {bus.max_step} counts/cycle ({CSP_JOG_RPM} motor RPM)")
    _say("Commands: <idx> deg|rev|cnt <value> | home | <Enter>=freeze | q")

    stop_mon = threading.Event()
    mon = threading.Thread(target=_monitor, args=(bus, stop_mon, n), daemon=True)
    mon.start()
    try:
        while True:
            _prompt()
            try:
                raw = sys.stdin.readline()
            except KeyboardInterrupt:
                break
            if not raw:                       # EOF
                break
            raw = raw.strip()
            if raw in ("q", "quit", "exit"):
                break
            if raw == "" or raw == "stop":
                bus.freeze()
                _say("  -> freeze all")
                continue
            if raw == "home":
                _say("  home counts: " +
                     ', '.join(f"D{i}={c}" for i, c in enumerate(bus.actual_counts())))
                continue
            parts = raw.split()
            if len(parts) != 3:
                _say("  format: <idx> deg|rev|cnt <value>")
                continue
            try:
                idx = int(parts[0])
                kind = parts[1]
                value = float(parts[2])
                delta = _counts_for(kind, value, idx)
            except (ValueError, KeyError):
                _say("  bad command")
                continue
            if not 0 <= idx < bus.n:
                _say(f"  idx out of range (0..{bus.n - 1})")
                continue
            bus.move_by_counts(idx, delta)
            _say(f"  -> D{idx} move {delta:+d} counts ({parts[1]} {value})")
    finally:
        stop_mon.set()
        mon.join(timeout=1.0)
        _restore_screen()
        bus.disconnect()


if __name__ == "__main__":
    main()
