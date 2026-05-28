import pysoem
import struct
import time
from config import (
    ECAT_ADAPTER_NAME,
    CONTROLWORD, STATUSWORD, MODE_OP, MODE_OP_DISPLAY,
    TARGET_VELOCITY, VELOCITY_ACTUAL, TORQUE_ACTUAL,
    PROFILE_ACCEL, PROFILE_DECEL, MAX_MOTOR_SPEED, GEAR_RATIO,
    ENCODER_COUNTS_PER_REV,
    BRAKE_OFF_DELAY, BRAKE_SPEED_LEVEL, BRAKE_ON_DELAY,
    CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OP, CW_DISABLE_VOLTAGE,
    CW_FAULT_RESET,
    SW_READY_TO_SWITCH_ON, SW_SWITCHED_ON, SW_OPERATION_ENABLED, SW_FAULT,
    MODE_PROFILE_VELOCITY,
    SPEED_MAX_RPM,
    BRAKE_RELEASE_SPEED_RPM, BRAKE_OFF_DELAY_MS, BRAKE_ON_DELAY_MS,
    DEFAULT_ACCEL_RPM, DEFAULT_DECEL_RPM,
)


def find_adapter(desc_hint):
    adapters = pysoem.find_adapters()
    for a in adapters:
        if desc_hint.lower() in a.desc.decode().lower():
            return a.name
    print("Available adapters:")
    for a in adapters:
        print(f"  {a.desc.decode()} -> {a.name}")
    return None


def sdo_write(drive, index, data):
    ret = drive.sdo_write(index, 0, data)
    if ret == 0:
        print(f"  [WARN] SDO write 0x{index:04X} returned 0")


def sdo_read(drive, index, size):
    return drive.sdo_read(index, 0, size)


def wait_for_sw(drive, mask, value, timeout=2.0):
    start = time.time()
    while time.time() - start < timeout:
        try:
            sw = struct.unpack('<H', sdo_read(drive, STATUSWORD, 2))[0]
            if (sw & mask) == value:
                return sw
        except Exception:
            pass
        time.sleep(0.05)
    sw = struct.unpack('<H', sdo_read(drive, STATUSWORD, 2))[0]
    return sw


def state_machine_enable(drive):
    sw = struct.unpack('<H', sdo_read(drive, STATUSWORD, 2))[0]
    print(f"  Initial statusword: 0x{sw:04X}")

    if sw & SW_FAULT:
        print("  Fault detected, resetting...")
        sdo_write(drive, CONTROLWORD, struct.pack('<H', CW_FAULT_RESET))
        time.sleep(0.2)
        sdo_write(drive, CONTROLWORD, struct.pack('<H', 0x0000))
        time.sleep(0.3)

    sw = struct.unpack('<H', sdo_read(drive, STATUSWORD, 2))[0]
    if (sw & SW_OPERATION_ENABLED) == SW_OPERATION_ENABLED:
        print("  Already enabled")
        return True

    print("  Shutdown -> Ready to Switch On")
    sdo_write(drive, CONTROLWORD, struct.pack('<H', CW_SHUTDOWN))
    sw = wait_for_sw(drive, 0x007F, SW_READY_TO_SWITCH_ON)
    if (sw & 0x007F) != SW_READY_TO_SWITCH_ON:
        print(f"  Failed to reach Ready to Switch On (sw=0x{sw:04X})")
        return False
    print(f"    Status: 0x{sw:04X}")

    print("  Switch On -> Switched On")
    sdo_write(drive, CONTROLWORD, struct.pack('<H', CW_SWITCH_ON))
    sw = wait_for_sw(drive, 0x007F, SW_SWITCHED_ON)
    if (sw & 0x007F) != SW_SWITCHED_ON:
        print(f"  Failed to reach Switched On (sw=0x{sw:04X})")
        return False
    print(f"    Status: 0x{sw:04X}")

    print("  Enable Operation (Servo ON)")
    sdo_write(drive, CONTROLWORD, struct.pack('<H', CW_ENABLE_OP))
    sw = wait_for_sw(drive, 0x007F, SW_OPERATION_ENABLED)
    if (sw & 0x007F) != SW_OPERATION_ENABLED:
        print(f"  Failed to enable (sw=0x{sw:04X})")
        return False
    print(f"    Status: 0x{sw:04X}  - Servo ON, brake should release")
    return True


def configure_brake(drive):
    print("Configuring brake parameters…")
    sdo_write(drive, BRAKE_SPEED_LEVEL, struct.pack('H', BRAKE_RELEASE_SPEED_RPM))
    sdo_write(drive, BRAKE_OFF_DELAY, struct.pack('H', BRAKE_OFF_DELAY_MS))
    sdo_write(drive, BRAKE_ON_DELAY, struct.pack('H', BRAKE_ON_DELAY_MS))
    print(f"  Brake release speed: {BRAKE_RELEASE_SPEED_RPM} RPM")
    print(f"  Brake OFF delay: {BRAKE_OFF_DELAY_MS} ms")
    print(f"  Brake ON delay: {BRAKE_ON_DELAY_MS} ms")


def set_mode(drive, mode):
    sdo_write(drive, MODE_OP, struct.pack('b', mode))
    time.sleep(0.2)
    actual = struct.unpack('b', sdo_read(drive, MODE_OP_DISPLAY, 1))[0]
    print(f"  Mode set: {mode}, mode display: {actual}")
    return actual == mode


def read_gear_ratio(drive):
    # 0x6091:1 = motor revolutions, 0x6091:2 = shaft revolutions
    try:
        motor_revs = struct.unpack('I', drive.sdo_read(GEAR_RATIO, 1, 4))[0]
        shaft_revs = struct.unpack('I', drive.sdo_read(GEAR_RATIO, 2, 4))[0]
        if motor_revs == 0 or shaft_revs == 0:
            raise ValueError("zero in gear ratio")
        return motor_revs, shaft_revs
    except Exception as e:
        print(f"  [WARN] Could not read 0x6091 Gear Ratio ({e}); falling back to 1:1")
        return 1, 1


def compute_raw_per_rpm(drive):
    motor_revs, shaft_revs = read_gear_ratio(drive)
    raw_per_rpm = ENCODER_COUNTS_PER_REV * motor_revs / shaft_revs / 60.0
    print(f"  Encoder: {ENCODER_COUNTS_PER_REV} counts/rev, "
          f"gear {motor_revs}/{shaft_revs} -> {raw_per_rpm:.3f} raw per RPM")
    return raw_per_rpm


def write_target_velocity(drive, raw_target):
    """Write 0x60FF and toggle controlword bit 4 to latch the new setpoint on Y7.

    Bit 4 in PV is "Reserved" per CiA 402, but empirically Y7 needs the
    0x000F → 0x001F → 0x000F transition to latch a new target velocity.
    Without it the drive keeps using its previous internal setpoint.
    """
    sdo_write(drive, TARGET_VELOCITY, struct.pack('i', raw_target))
    sdo_write(drive, CONTROLWORD, struct.pack('<H', 0x001F))
    time.sleep(0.05)
    sdo_write(drive, CONTROLWORD, struct.pack('<H', 0x000F))
    time.sleep(0.05)
    readback = struct.unpack('i', sdo_read(drive, TARGET_VELOCITY, 4))[0]
    if readback != raw_target:
        print(f"  [WARN] target velocity mismatch: wrote {raw_target}, read {readback}")
    return readback


def read_actual_velocity(drive):
    raw = sdo_read(drive, VELOCITY_ACTUAL, 4)
    return struct.unpack('i', raw)[0]


def read_actual_torque(drive):
    raw = sdo_read(drive, TORQUE_ACTUAL, 2)
    return struct.unpack('H', raw)[0]


def main():
    print("=" * 60)
    print("EtherCAT 3DOF - Speed Test (Prototype)")
    print("=" * 60)

    adapter_name = find_adapter(ECAT_ADAPTER_NAME)
    if adapter_name is None:
        print(f"Adapter '{ECAT_ADAPTER_NAME}' not found!")
        return

    master = pysoem.Master()
    print(f"Opening adapter: {adapter_name}")
    master.open(adapter_name)

    try:
        slave_count = master.config_init()
        print(f"Found {slave_count} slave(s)")
        if slave_count < 1:
            print("No slaves found!")
            return

        drive = master.slaves[0]
        print(f"  Slave 0: {drive.name}")
        print(f"    Manufacturer: 0x{drive.man:08X}")
        print(f"    Product code: 0x{drive.id:08X}")
        print(f"    Revision: 0x{drive.rev:08X}")

        master.config_map()
        print("PDO mapping configured")

        master.state = pysoem.OP_STATE
        time.sleep(0.5)
        print(f"Master state: {master.state}")
        if master.state != pysoem.OP_STATE:
            print("Failed to reach Operational state!")
            return

        raw_per_rpm = compute_raw_per_rpm(drive)

        configure_brake(drive)

        print("Setting Profile Velocity mode before enable…")
        if not set_mode(drive, MODE_PROFILE_VELOCITY):
            print("Warning: mode may not be set correctly")

        # Safety: clear any stale target velocity left in 0x60FF from a previous
        # session (other test scripts in this repo do not zero it on exit).
        # Done BEFORE Servo ON so the motor starts from a known 0 setpoint.
        sdo_write(drive, TARGET_VELOCITY, struct.pack('i', 0))
        stale = struct.unpack('i', sdo_read(drive, TARGET_VELOCITY, 4))[0]
        print(f"  Pre-enable target velocity cleared (read-back: {stale})")

        if not state_machine_enable(drive):
            print("Failed to enable drive!")
            return

        print("Setting Profile Velocity mode…")
        if not set_mode(drive, MODE_PROFILE_VELOCITY):
            print("Warning: mode may not be set correctly")

        accel_raw = int(DEFAULT_ACCEL_RPM * raw_per_rpm)
        decel_raw = int(DEFAULT_DECEL_RPM * raw_per_rpm)
        max_speed_raw = int(SPEED_MAX_RPM * raw_per_rpm)
        sdo_write(drive, PROFILE_ACCEL, struct.pack('I', accel_raw))
        sdo_write(drive, PROFILE_DECEL, struct.pack('I', decel_raw))
        sdo_write(drive, MAX_MOTOR_SPEED, struct.pack('I', max_speed_raw))
        print(f"  Accel: {DEFAULT_ACCEL_RPM} rpm/s (raw {accel_raw})")
        print(f"  Decel: {DEFAULT_DECEL_RPM} rpm/s (raw {decel_raw})")
        print(f"  Max speed: {SPEED_MAX_RPM} RPM (raw {max_speed_raw})")
        print("-" * 60)
        print("Interactive speed control")
        print("  Enter speed in RPM (0-4500), Enter=Stop, Ctrl+C=Exit")
        print("-" * 60)

        current_speed = 0
        while True:
            try:
                user_input = input(f"\nSpeed [{current_speed}] > ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting…")
                break

            if user_input == "":
                target_speed = 0
            else:
                try:
                    target_speed = int(user_input)
                except ValueError:
                    print("  Invalid input, enter a number")
                    continue

            target_speed = max(0, min(target_speed, SPEED_MAX_RPM))
            current_speed = target_speed

            raw_target = int(target_speed * raw_per_rpm)
            print(f"  Setting target: {target_speed} RPM (raw {raw_target})")
            write_target_velocity(drive, raw_target)
            time.sleep(0.2)

            actual_vel_raw = read_actual_velocity(drive)
            actual_rpm = actual_vel_raw / raw_per_rpm
            actual_trq = read_actual_torque(drive)
            sw = struct.unpack('<H', sdo_read(drive, STATUSWORD, 2))[0]
            print(f"  Actual vel: {actual_rpm:.1f} RPM (raw {actual_vel_raw})  |  Torque: {actual_trq}  |  SW: 0x{sw:04X}")

            if sw & SW_FAULT:
                print("  FAULT detected!")
                break

    finally:
        print("\nShutting down…")
        if 'drive' in locals() and drive is not None:
            try:
                # Zero target velocity FIRST so we don't disable mid-motion and
                # don't leave a stale setpoint for the next session.
                sdo_write(drive, TARGET_VELOCITY, struct.pack('i', 0))
                time.sleep(0.1)
                sdo_write(drive, CONTROLWORD, struct.pack('<H', CW_DISABLE_VOLTAGE))
                time.sleep(0.3)
                print("  Servo OFF, brake engaged")
            except Exception:
                pass
        if 'master' in locals() and master is not None:
            try:
                master.state = pysoem.INIT_STATE
                time.sleep(0.2)
                master.close()
                print("  EtherCAT closed")
            except Exception:
                pass


if __name__ == "__main__":
    main()
