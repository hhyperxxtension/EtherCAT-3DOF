#!/usr/bin/env python3
"""
EtherCAT bus scanner – prints the slaves that are present.

It uses pysoem (the same library that the existing demos use) and does
only read‑only operations:
* finds an adapter matching the description in src/config.py
* opens the master, runs the configuration init
* enumerates every slave and prints its basic identification data
"""

import sys
import pysoem
from src.config import ECAT_ADAPTER_NAME



def find_adapter(desc_hint: str) -> str | None:
    """Return the first adapter whose description contains *desc_hint*."""
    for a in pysoem.find_adapters():
        if desc_hint.lower() in a.desc.decode().lower():
            return a.name
    return None


def main() -> None:
    adapter = find_adapter(ECAT_ADAPTER_NAME)
    if not adapter:
        sys.stderr.write(f"Adapter '{ECAT_ADAPTER_NAME}' not found.\n")
        sys.exit(1)

    master = pysoem.Master()
    print(f"Opening adapter: {adapter}")
    master.open(adapter)

    # Discover the slaves on the bus
    slave_cnt = master.config_init()
    print(f"\nFound {slave_cnt} EtherCAT slave(s):\n")

    for i, s in enumerate(master.slaves):
        print(f"Slave {i}:")
        print(f"  Name          : {s.name}")
        print(f"  Manufacturer  : 0x{s.man:08X}")
        print(f"  Product code  : 0x{s.id:08X}")
        print(f"  Revision      : 0x{s.rev:08X}")
        print("-" * 40)



    # Clean shutdown
    master.close()


if __name__ == "__main__":
    main()
