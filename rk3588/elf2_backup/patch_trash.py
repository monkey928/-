#!/usr/bin/env python3
"""Fix trash_detector.py BinaryFrame to send STM32 CMD_SET_GOAL"""
import shutil

shutil.copy('/home/elf/trash_detector.py', '/home/elf/trash_detector.py.bak3')
with open('/home/elf/trash_detector.py', 'r') as f:
    content = f.read()

# Replace the entire pack() method to use CMD_SET_GOAL (0x02)
old_pack = '''    def pack(self) -> bytes:
        """STM32 protocol: AA 55 type len payload SUM-checksum"""
        flags = (self.category & 0x01) | ((self.valid & 0x01) << 1)
        payload  = struct.pack('<B',  flags)
        payload += struct.pack('<H',  self.u)
        payload += struct.pack('<H',  self.v)
        payload += struct.pack('<h',  self.dx)
        payload += struct.pack('<h',  self.dy)
        payload += struct.pack('<h',  self.x_mm)
        payload += struct.pack('<h',  self.y_mm)
        payload += struct.pack('<H',  self.arrive_ms)
        cmd_type = 0x06
        length = len(payload)
        checksum = (cmd_type + length) & 0xFF
        for b in payload:
            checksum = (checksum + b) & 0xFF
        return self.HEADER + bytes([cmd_type, length]) + payload + bytes([checksum])'''

new_pack = '''    def pack(self) -> bytes:
        """STM32 CMD_SET_GOAL (0x02): AA 55 02 06 x_mm y_mm arrive_ms checksum"""
        payload  = struct.pack('<h',  self.x_mm)
        payload += struct.pack('<h',  self.y_mm)
        payload += struct.pack('<H',  self.arrive_ms)
        cmd_type = 0x02  # CMD_SET_GOAL
        length = len(payload)  # 6
        checksum = (cmd_type + length) & 0xFF
        for b in payload:
            checksum = (checksum + b) & 0xFF
        return self.HEADER + bytes([cmd_type, length]) + payload + bytes([checksum])'''

if old_pack in content:
    content = content.replace(old_pack, new_pack)
    print('[OK] BinaryFrame.pack() now sends CMD_SET_GOAL (0x02)')
else:
    print('[FAIL] old pack() not found - may already be fixed')
    # check what's there
    import re
    m = re.search(r'def pack\(self\) -> bytes:.*?return self\.HEADER', content, re.DOTALL)
    if m:
        print('Found pack() but different from expected:')
        print(m.group()[:200])

with open('/home/elf/trash_detector.py', 'w') as f:
    f.write(content)

# Verify
print('\nVerification:')
with open('/home/elf/trash_detector.py', 'r') as f:
    for i, line in enumerate(f, 1):
        if 'cmd_type = 0x02' in line or 'CMD_SET_GOAL' in line:
            print(f'  Line {i}: {line.strip()}')
        if 'checksum = (cmd_type + length)' in line:
            print(f'  Line {i}: {line.strip()} (SUM checksum)')
