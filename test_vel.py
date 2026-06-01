import pysoem, struct, time, binascii

ECAT_ADAPTER_NAME = "Realtek PCIe GbE Family Controller"

def find_adapter(desc_hint):
    for a in pysoem.find_adapters():
        if desc_hint.lower() in a.desc.decode().lower():
            return a.name
    return None

adapter = find_adapter(ECAT_ADAPTER_NAME)
if not adapter:
    raise RuntimeError('Adapter not found')

master = pysoem.Master()
master.open(adapter)
master.config_init()
master.config_map()
slave = master.slaves[0]

# Set mode to Profile Velocity (3)
slave.sdo_write(0x6060, 0, struct.pack('b', 3))
time.sleep(0.2)

# Enable drive (use SDO controlword sequence)
for cw in [0x0006, 0x0007, 0x000F]:
    slave.sdo_write(0x6040, 0, struct.pack('<H', cw))
    time.sleep(0.3)

print('Drive enabled, statusword:', hex(struct.unpack('<H', slave.sdo_read(0x6041,0,2))[0]))

targets = [10,20,50,100,200]
for t in targets:
    # write target velocity
    slave.sdo_write(0x60FF, 0, struct.pack('i', t))
    # read back target
    raw_target = struct.unpack('i', slave.sdo_read(0x60FF,0,4))[0]
    # read actual velocity
    raw_actual = struct.unpack('i', slave.sdo_read(0x606C,0,4))[0]
    print(f'TargetRPM {t} => raw_target {raw_target}, raw_actual {raw_actual}')
    time.sleep(0.5)

# Shutdown
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x0000))
master.close()
