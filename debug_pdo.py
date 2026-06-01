import pysoem, binascii, time
adapter='\\\\Device\\\\NPF_{ED435CCD-CB5C-43D8-B4D7-001B78ADFCC4}'
m=pysoem.Master()
m.open(adapter)
m.config_init()
m.config_map()
slave=m.slaves[0]
print('output size:',len(slave.output))
print('input size:',len(slave.input))
print('output bytes:',binascii.hexlify(slave.output))
print('input bytes:',binascii.hexlify(slave.input))
