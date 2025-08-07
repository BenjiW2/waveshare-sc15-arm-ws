#!/usr/bin/env python3
from STservo_sdk import PortHandler
from STservo_sdk.scscl import scscl            # helper class

PORT = "/dev/cu.usbmodem5A460835741"   # ← your adapter path
BAUD = 1_000_000

OLD_ID = 1    # current (temporary) ID
NEW_ID = 4    # set 3, 4, 5 for the other joints

ph = PortHandler(PORT)
assert ph.openPort() and ph.setBaudRate(BAUD), "Serial init failed"
sc = scscl(ph)                               # protocol-1.0 helper

# 1️⃣ unlock
sc.unLockEprom(OLD_ID)

# 2️⃣ write new ID into address 0x05
sc.write1ByteTxRx(OLD_ID, 5, NEW_ID)

# 3️⃣ commit all buffered bytes to flash (addr 0x28 ← 1)
sc.write1ByteTxRx(OLD_ID, 40, 1)

# 4️⃣ relock
sc.LockEprom(OLD_ID)

print(f"ID {OLD_ID} → {NEW_ID} queued.  >>> NOW pull that servo’s 7.8 V,")
print("wait 2 s, plug back in, and ping the NEW ID to confirm it survives reboot.")
