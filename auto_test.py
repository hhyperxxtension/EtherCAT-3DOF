import pysoem, struct, time

ECAT_ADAPTER_NAME = "Realtek PCIe GbE Family Controller"

def find_adapter(desc):
    for a in pysoem.find_adapters():
        if desc.lower() in a.desc.decode().lower():
            return a.name
    return None

adapter = find_adapter(ECAT_ADAPTER_NAME)
master = pysoem.Master()
master.open(adapter)
master.config_init()
master.config_map()
slave = master.slaves[0]
# Set mode to profile velocity (3)
slave.sdo_write(0x6060, 0, struct.pack('b', 3))
time.sleep(0.2)
# Enable drive (shutdown, switch on, enable op)
for cw in [0x0006, 0x0007, 0x000F]:
    slave.sdo_write(0x6040, 0, struct.pack('<H', cw))
    time.sleep(0.3)
print('Drive enabled, status', hex(struct.unpack('<H', slave.sdo_read(0x6041,0,2))[0]))
# Set accel/decel high
slave.sdo_write(0x6083, 0, struct.pack('I', 1000000))
slave.sdo_write(0x6084, 0, struct.pack('I', 1000000))
# Test speeds
for rpm in [0, 10, 20, 50, 100, 200, 500, 1000, 2000]:
    # write target velocity (raw = rpm, adjust later if scaling needed)
    slave.sdo_write(0x60FF, 0, struct.pack('i', rpm))
    # toggle new setpoint
    slave.sdo_write(0x6040, 0, struct.pack('<H', 0x001F))
    time.sleep(0.1)
    slave.sdo_write(0x6040, 0, struct.pack('<H', 0x000F))
    time.sleep(0.5)
    raw_actual = struct.unpack('i', slave.sdo_read(0x606C,0,4))[0]
    torque = struct.unpack('H', slave.sdo_read(0x6077,0,2))[0]
    print(f'RPM set {rpm}: actual_raw={raw_actual}, torque={torque}')

# Stop and disable
slave.sdo_write(0x60FF, 0, struct.pack('i', 0))
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x001F))
time.sleep(0.1)
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x000F))
time.sleep(0.5)
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x0000))
master.close()
