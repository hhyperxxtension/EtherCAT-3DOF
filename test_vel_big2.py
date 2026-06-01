import pysoem, struct, time

ECAT_ADAPTER_NAME = "Realtek PCIe GbE Family Controller"

def find_adapter(desc_hint):
    for a in pysoem.find_adapters():
        if desc_hint.lower() in a.desc.decode().lower():
            return a.name
    return None

adapter = find_adapter(ECAT_ADAPTER_NAME)
master = pysoem.Master()
master.open(adapter)
master.config_init()
master.config_map()
slave = master.slaves[0]
# Set profile velocity mode and enable drive
slave.sdo_write(0x6060, 0, struct.pack('b', 3))
time.sleep(0.2)
for cw in [0x0006, 0x0007, 0x000F]:
    slave.sdo_write(0x6040, 0, struct.pack('<H', cw))
    time.sleep(0.3)
print('Enabled, status', hex(struct.unpack('<H', slave.sdo_read(0x6041,0,2))[0]))

targets = [5000, 10000, 20000, 50000]
for t in targets:
    slave.sdo_write(0x60FF, 0, struct.pack('i', t))
    # trigger new setpoint
    slave.sdo_write(0x6040, 0, struct.pack('<H', 0x001F))
    time.sleep(0.5)
    raw_actual = struct.unpack('i', slave.sdo_read(0x606C,0,4))[0]
    torque = struct.unpack('H', slave.sdo_read(0x6077,0,2))[0]
    status = struct.unpack('<H', slave.sdo_read(0x6041,0,2))[0]
    print(f'Target {t}: actual={raw_actual}, torque={torque}, status=0x{status:04X}')

slave.sdo_write(0x6040, 0, struct.pack('<H', 0x0000))
master.close()
