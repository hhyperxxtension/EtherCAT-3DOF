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
# Read factor
raw = slave.sdo_read(0x6081, 0, 4)
val = struct.unpack('f', raw)[0]
print('Current factor (raw):', val)
# Write a known factor (0.001) and read back
slave.sdo_write(0x6081, 0, struct.pack('f', 0.001))
raw2 = slave.sdo_read(0x6081, 0, 4)
print('Written 0.001, read back:', struct.unpack('f', raw2)[0])
master.close()
