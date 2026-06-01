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
# Set profile velocity mode
slave.sdo_write(0x6060, 0, struct.pack('b', 3))
time.sleep(0.2)
# Enable drive
for cw in [0x0006, 0x0007, 0x000F]:
    slave.sdo_write(0x6040, 0, struct.pack('<H', cw))
    time.sleep(0.3)
print('Enabled, status', hex(struct.unpack('<H', slave.sdo_read(0x6041,0,2))[0]))
# High accel/decel
slave.sdo_write(0x6083, 0, struct.pack('I', 1000000))
slave.sdo_write(0x6084, 0, struct.pack('I', 1000000))

raw_vals = [10000, 50000, 100000, 200000, 500000, 1000000]
for raw in raw_vals:
    slave.sdo_write(0x60FF, 0, struct.pack('i', raw))
    # toggle setpoint
    slave.sdo_write(0x6040, 0, struct.pack('<H', 0x001F))
    time.sleep(0.1)
    slave.sdo_write(0x6040, 0, struct.pack('<H', 0x000F))
    time.sleep(1)
    actual = struct.unpack('i', slave.sdo_read(0x606C,0,4))[0]
    torque = struct.unpack('H', slave.sdo_read(0x6077,0,2))[0]
    print(f'raw set {raw}: actual_raw={actual}, torque={torque}')

# Stop
slave.sdo_write(0x60FF, 0, struct.pack('i', 0))
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x001F))
time.sleep(0.1)
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x000F))
time.sleep(0.5)
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x0000))
master.close()
