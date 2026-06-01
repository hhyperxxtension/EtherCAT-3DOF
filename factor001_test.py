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
# Set velocity factor to 0.001 (raw unit = 0.001 rpm?)
slave.sdo_write(0x6081, 0, struct.pack('f', 0.001))
print('Velocity factor set to 0.001')
# Set mode and enable drive
slave.sdo_write(0x6060, 0, struct.pack('b', 3))
time.sleep(0.2)
for cw in [0x0006,0x0007,0x000F]:
    slave.sdo_write(0x6040, 0, struct.pack('<H', cw))
    time.sleep(0.2)
print('Drive enabled')
# High accel/decel
slave.sdo_write(0x6083, 0, struct.pack('I', 1000000))
slave.sdo_write(0x6084, 0, struct.pack('I', 1000000))
# Test RPM values with proper scaling (raw = rpm/0.001 => rpm*1000)
for rpm in [10,20,50,100,200,500,1000,2000,3000,4000]:
    raw = int(rpm / 0.001)  # rpm * 1000
    slave.sdo_write(0x60FF, 0, struct.pack('i', raw))
    # toggle setpoint
    slave.sdo_write(0x6040, 0, struct.pack('<H', 0x001F))
    time.sleep(0.1)
    slave.sdo_write(0x6040, 0, struct.pack('<H', 0x000F))
    time.sleep(0.5)
    actual = struct.unpack('i', slave.sdo_read(0x606C,0,4))[0]
    torque = struct.unpack('H', slave.sdo_read(0x6077,0,2))[0]
    print(f'Set RPM {rpm}: raw={raw}, actual_raw={actual}, torque={torque}')
# Stop
slave.sdo_write(0x60FF, 0, struct.pack('i', 0))
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x001F))
time.sleep(0.1)
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x000F))
time.sleep(0.5)
slave.sdo_write(0x6040, 0, struct.pack('<H', 0x0000))
master.close()
