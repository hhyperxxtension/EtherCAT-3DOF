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

def read_obj(idx, size):
    try:
        raw = slave.sdo_read(idx, 0, size)
        return raw
    except Exception as e:
        print(f'Error reading 0x{idx:04X}:', e)
        return None

# try common sizes
for idx in [0x6080, 0x6081, 0x6082, 0x6085, 0x6086]:
    data = read_obj(idx, 4)
    if data:
        # attempt to interpret as float and int
        try:
            f = struct.unpack('f', data)[0]
            i = struct.unpack('i', data)[0]
            print(f'0x{idx:04X}: float={f}, int={i}')
        except Exception:
            print(f'0x{idx:04X}: raw={data.hex()}')

master.close()
